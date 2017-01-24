/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>


#include "platform.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/utils.h"

#include "sensors/barometer.h"
#include "sensors/sonar.h"

#include "rx/rx.h"

#include "fc/rc_controls.h"
#include "io/motors.h"

#include "flight/pid.h"
#include "flight/imu.h"

#include "fc/runtime_config.h"


int32_t setVelocity = 0;
uint8_t velocityControl = 0;
int32_t errorVelocityI = 0;
int32_t errorAccelerateI = 0;
int32_t altHoldThrottleAdjustment = 0;
int32_t AltHold;
int32_t vario = 0;                      // variometer in cm/s


static barometerConfig_t *barometerConfig;
static pidProfile_t *pidProfile;
static rcControlsConfig_t *rcControlsConfig;
static motorConfig_t *motorConfig;

void configureAltitudeHold(
        pidProfile_t *initialPidProfile,
        barometerConfig_t *intialBarometerConfig,
        rcControlsConfig_t *initialRcControlsConfig,
        motorConfig_t *initialMotorConfig
)
{
    pidProfile = initialPidProfile;
    barometerConfig = intialBarometerConfig;
    rcControlsConfig = initialRcControlsConfig;
    motorConfig = initialMotorConfig;
}

#if defined(BARO) || defined(SONAR) || defined(USE_ACC_ALT_HOLD)

static int16_t initialThrottleHold;
static int32_t EstAlt;                // in cm
static float accAlt = 0.0f;
static float vel = 0.0f;
static uint8_t initialStickPos = 0; // flag for Throttle Stick move to center

// 40hz update rate (20hz LPF on acc)
#define BARO_UPDATE_FREQUENCY_40HZ (1000 * 25)

#define DEGREES_80_IN_DECIDEGREES 800

static void applyMultirotorAltHold(rxConfig_t *rxConfig)
{
    static uint8_t isAltHoldChanged = 0;
    // multirotor alt hold
    if (FLIGHT_MODE(ALT_HOLD_MODE)) {
        setVelocity = 0;
        if (ABS(rcData[THROTTLE] - rxConfig->midrc) > rcControlsConfig->alt_hold_deadband) {
            if (!initialStickPos) {
                initialStickPos = 1;
            } else {
                // set velocity proportional to stick movement +100 throttle gives ~ +50 mm/s
                setVelocity = (rcData[THROTTLE] - rxConfig->midrc) / 2;
            }
        }
        rcCommand[THROTTLE] = constrain(initialThrottleHold + altHoldThrottleAdjustment, motorConfig->minthrottle, motorConfig->maxthrottle);
        return;
    }

    if (rcControlsConfig->alt_hold_fast_change) {
        // rapid alt changes
        if (ABS(rcData[THROTTLE] - initialThrottleHold) > rcControlsConfig->alt_hold_deadband) {
            errorVelocityI = 0;
            isAltHoldChanged = 1;
            rcCommand[THROTTLE] += (rcData[THROTTLE] > initialThrottleHold) ? -rcControlsConfig->alt_hold_deadband : rcControlsConfig->alt_hold_deadband;
        } else {
            if (isAltHoldChanged) {
                AltHold = EstAlt;
                isAltHoldChanged = 0;
            }
            rcCommand[THROTTLE] = constrain(initialThrottleHold + altHoldThrottleAdjustment, motorConfig->minthrottle, motorConfig->maxthrottle);
        }
    } else {
        // slow alt changes, mostly used for aerial photography
        if (ABS(rcData[THROTTLE] - initialThrottleHold) > rcControlsConfig->alt_hold_deadband) {
            // set velocity proportional to stick movement +100 throttle gives ~ +50 cm/s
            setVelocity = (rcData[THROTTLE] - initialThrottleHold) / 2;
            velocityControl = 1;
            isAltHoldChanged = 1;
        } else if (isAltHoldChanged) {
            AltHold = EstAlt;
            velocityControl = 0;
            isAltHoldChanged = 0;
        }
        rcCommand[THROTTLE] = constrain(initialThrottleHold + altHoldThrottleAdjustment, motorConfig->minthrottle, motorConfig->maxthrottle);
    }
}

static void applyFixedWingAltHold(airplaneConfig_t *airplaneConfig)
{
    // handle fixedwing-related althold. UNTESTED! and probably wrong
    // most likely need to check changes on pitch channel and 'reset' althold similar to
    // how throttle does it on multirotor

    rcCommand[PITCH] += altHoldThrottleAdjustment * airplaneConfig->fixedwing_althold_dir;
}

void applyAltHold(rxConfig_t *rxConfig, airplaneConfig_t *airplaneConfig)
{
    if (STATE(FIXED_WING)) {
        applyFixedWingAltHold(airplaneConfig);
    } else {
        applyMultirotorAltHold(rxConfig);
    }
}

bool isThrustFacingDownwards(attitudeEulerAngles_t *attitude)
{
    return ABS(attitude->values.roll) < DEGREES_80_IN_DECIDEGREES && ABS(attitude->values.pitch) < DEGREES_80_IN_DECIDEGREES;
}

int32_t altitudeHoldGetEstimatedAltitude(void)
{
    return EstAlt;
}

#endif

#if defined(BARO) || defined(SONAR)

void updateAltHoldState(void)
{
    // Baro alt hold activate
    if (!IS_RC_MODE_ACTIVE(BOXBARO)) {
        DISABLE_FLIGHT_MODE(BARO_MODE);
        return;
    }

    if (!FLIGHT_MODE(BARO_MODE)) {
        ENABLE_FLIGHT_MODE(BARO_MODE);
        AltHold = EstAlt;
        initialThrottleHold = rcData[THROTTLE];
        errorVelocityI = 0;
        altHoldThrottleAdjustment = 0;
    }
}

void updateSonarAltHoldState(void)
{
    // Sonar alt hold activate
    if (!IS_RC_MODE_ACTIVE(BOXSONAR)) {
        DISABLE_FLIGHT_MODE(SONAR_MODE);
        return;
    }

    if (!FLIGHT_MODE(SONAR_MODE)) {
        ENABLE_FLIGHT_MODE(SONAR_MODE);
        AltHold = EstAlt;
        initialThrottleHold = rcData[THROTTLE];
        errorVelocityI = 0;
        altHoldThrottleAdjustment = 0;
    }
}

int32_t calculateAltHoldThrottleAdjustment(int32_t vel_tmp, float accZ_tmp, float accZ_old)
{
    int32_t result = 0;
    int32_t error;
    int32_t setVel;

    if (!isThrustFacingDownwards(&attitude)) {
        return result;
    }

    // Altitude P-Controller

    if (!velocityControl) {
        error = constrain(AltHold - EstAlt, -500, 500);
        error = applyDeadband(error, 10); // remove small P parameter to reduce noise near zero position
        setVel = constrain((pidProfile->P8[PIDALT] * error / 128), -300, +300); // limit velocity to +/- 3 m/s
    } else {
        setVel = setVelocity;
    }
    // Velocity PID-Controller

    // P
    error = setVel - vel_tmp;
    result = constrain((pidProfile->P8[PIDVEL] * error / 32), -300, +300);

    // I
    errorVelocityI += (pidProfile->I8[PIDVEL] * error);
    errorVelocityI = constrain(errorVelocityI, -(8192 * 200), (8192 * 200));
    result += errorVelocityI / 8192;     // I in range +/-200

    // D
    result -= constrain(pidProfile->D8[PIDVEL] * (accZ_tmp + accZ_old) / 512, -150, 150);

    return result;
}

void calculateEstimatedAltitude(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimeUs;
    uint32_t dTime;
    int32_t baroVel;
    float dt;
    float vel_acc;
    int32_t vel_tmp;
    float accZ_tmp;
    static float accZ_old = 0.0f;
//    static float vel = 0.0f;
//    static float accAlt = 0.0f;
    static int32_t lastBaroAlt;

#ifdef SONAR
    int32_t sonarAlt = SONAR_OUT_OF_RANGE;
    static int32_t baroAlt_offset = 0;
    float sonarTransition;
#endif

    dTime = currentTimeUs - previousTimeUs;
    if (dTime < BARO_UPDATE_FREQUENCY_40HZ)
        return;

    previousTimeUs = currentTimeUs;

#ifdef BARO
    if (!isBaroCalibrationComplete()) {
        performBaroCalibrationCycle();
        vel = 0;
        accAlt = 0;
    }

    baro.BaroAlt = baroCalculateAltitude();
#else
    baro.BaroAlt = 0;
#endif

#ifdef SONAR
    sonarAlt = sonarRead();
    sonarAlt = sonarCalculateAltitude(sonarAlt, getCosTiltAngle());

    if (sonarAlt > 0 && sonarAlt < sonarCfAltCm) {
        // just use the SONAR
        baroAlt_offset = baro.BaroAlt - sonarAlt;
        baro.BaroAlt = sonarAlt;
    } else {
        baro.BaroAlt -= baroAlt_offset;
        if (sonarAlt > 0  && sonarAlt <= sonarMaxAltWithTiltCm) {
            // SONAR in range, so use complementary filter
            sonarTransition = (float)(sonarMaxAltWithTiltCm - sonarAlt) / (sonarMaxAltWithTiltCm - sonarCfAltCm);
            baro.BaroAlt = sonarAlt * sonarTransition + baro.BaroAlt * (1.0f - sonarTransition);
        }
    }
#endif

    dt = accTimeSum * 1e-6f; // delta acc reading time in seconds

    // Integrator - velocity, cm/sec
    if (accSumCount) {
        accZ_tmp = (float)accSum[2] / (float)accSumCount;
    } else {
        accZ_tmp = 0;
    }
    vel_acc = accZ_tmp * accVelScale * (float)accTimeSum;

    // Integrator - Altitude in cm
    accAlt += (vel_acc * 0.5f) * dt + vel * dt;                                                                 // integrate velocity to get distance (x= a/2 * t^2)
    accAlt = accAlt * barometerConfig->baro_cf_alt + (float)baro.BaroAlt * (1.0f - barometerConfig->baro_cf_alt);    // complementary filter for altitude estimation (baro & acc)
    vel += vel_acc;

#ifdef DEBUG_ALT_HOLD
    debug[1] = accSum[2] / accSumCount; // acceleration
    debug[2] = vel;                     // velocity
    debug[3] = accAlt;                  // height
#endif

    imuResetAccelerationSum();

#ifdef BARO
    if (!isBaroCalibrationComplete()) {
        return;
    }
#endif

#ifdef SONAR
    if (sonarAlt > 0 && sonarAlt < sonarCfAltCm) {
        // the sonar has the best range
        EstAlt = baro.BaroAlt;
    } else {
        EstAlt = accAlt;
    }
#else
    EstAlt = accAlt;
#endif

    baroVel = (baro.BaroAlt - lastBaroAlt) * 1000000.0f / dTime;
    lastBaroAlt = baro.BaroAlt;

    baroVel = constrain(baroVel, -1500, 1500);  // constrain baro velocity +/- 1500cm/s
    baroVel = applyDeadband(baroVel, 10);       // to reduce noise near zero

    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    vel = vel * barometerConfig->baro_cf_vel + baroVel * (1.0f - barometerConfig->baro_cf_vel);
    vel_tmp = lrintf(vel);

    // set vario
    vario = applyDeadband(vel_tmp, 5);

    altHoldThrottleAdjustment = calculateAltHoldThrottleAdjustment(vel_tmp, accZ_tmp, accZ_old);

    accZ_old = accZ_tmp;
}
#endif

#if defined(USE_ACC_ALT_HOLD)
static uint8_t hoverTest = 0;
int32_t calculateAltHoldThrottleAdjustmentACC(int32_t vel_tmp, float accZ_tmp, float accZ_old)
{
    int32_t result = 0;
    int32_t error;
    int32_t setVel = setVelocity;

    uint8_t P, I, D;

    if (!isThrustFacingDownwards(&attitude)) {
        return result;
    }

    // Altitude P-Controller
    // not yet :(


    // Velocity PID-Controller
    P = pidProfile->P8[PIDVEL];
    I = pidProfile->I8[PIDVEL];
    D = pidProfile->D8[PIDVEL];

    // P
    error = setVel - vel_tmp;
    result = constrain((P * error / 128), -600, +600);

    // I
    errorVelocityI += (I * error);
    errorVelocityI = constrain(errorVelocityI, -(128 * 600), (128 * 600));
    result += errorVelocityI / 128;     // I in range +/-600

    // D
    result -= constrain(D * (accZ_tmp + accZ_old) / 256, -(256 * 300), (256 * 300));



    // Accelerate PID-Controller
    P = pidProfile->P8[PIDALT];
    I = pidProfile->I8[PIDALT];
    D = pidProfile->D8[PIDALT];

    // P
    error = result - accZ_tmp;
    result = constrain((P * error / 64), -600, +600);

    // I
    errorAccelerateI += (I * error);
    errorAccelerateI = constrain(errorAccelerateI, -(256 * 600), (256 * 600));
    result += errorAccelerateI / 256;     // I in range +/-600

    // D
    result -= constrain(D * (accZ_tmp - accZ_old) / 256, -(256 * 300), (256 * 300));

    return result;
}

void calculateEstimatedAltitudeACC(timeUs_t currentTime)
{
    UNUSED(currentTime);
    float dt;
    float vel_acc;
    int32_t vel_tmp;
    float accZ_tmp;
    static float accZ_old = 0.0f;

    dt = accTimeSum * 1e-6f; // delta acc reading time in seconds

    // Integrator - velocity, cm/sec
    if (accSumCount) {
        accZ_tmp = (float)accSum[2] / (float)accSumCount;
    } else {
        accZ_tmp = 0;
    }
    vel_acc = accZ_tmp * accVelScale * (float)accTimeSum * 10.0f;

    // Integrator - Altitude in mm
    accAlt += (vel_acc * 0.5f) * dt + vel * dt;                                                                 // integrate velocity to get distance (x= a/2 * t^2)
    vel += vel_acc;
    vel *= 0.92f;   // FIXME simple fix velocity integrate error

#ifdef DEBUG_ALT_HOLD
    debug[1] = accZ_tmp;                        // acceleration
    debug[2] = vel * 10.0f;                     // velocity
    debug[3] = accAlt * 10.0f;                  // height
#endif

    imuResetAccelerationSum();
    EstAlt = accAlt / 10;

    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    vel_tmp = lrintf(vel);

    // set vario
    vario = applyDeadband(vel_tmp, 5);

    altHoldThrottleAdjustment = calculateAltHoldThrottleAdjustmentACC(vel_tmp, accZ_tmp, accZ_old);

    accZ_old = accZ_tmp;
}

void resetACCVel(void)
{
    vel = 0.0f;
    accAlt = 0.0f;
}

void updateACCAltHoldState(void)
{
    // ACC alt hold activate
    if (!IS_RC_MODE_ACTIVE(BOXALTHOLD)) {
        DISABLE_FLIGHT_MODE(ALT_HOLD_MODE);
        return;
    }

    if (!FLIGHT_MODE(ALT_HOLD_MODE)) {
        ENABLE_FLIGHT_MODE(ALT_HOLD_MODE);
        AltHold = EstAlt * 10; // cm -> mm
        initialThrottleHold = rcData[THROTTLE];
        errorVelocityI = 0;
        errorAccelerateI = 0;
        altHoldThrottleAdjustment = 0;
        initialStickPos = 0;
        hoverTest = 0;
    }
}
#endif

