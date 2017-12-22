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

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/altitude.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "rx/rx.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"
#include "sensors/sonar.h"


int32_t AltHold;
static int32_t estimatedVario = 0;                      // variometer in cm/s
static int32_t estimatedAltitude = 0;                // in cm

static float vel = 0.0f; // in cm/s
static float accAlt = 0.0f; // in cm


enum {
    DEBUG_ALTITUDE_ACC,
    DEBUG_ALTITUDE_VEL,
    DEBUG_ALTITUDE_HEIGHT
};
// 40hz update rate (20hz LPF on acc)
#define BARO_UPDATE_FREQUENCY_40HZ (1000 * 25)

#if defined(USE_ALT_HOLD)

PG_REGISTER_WITH_RESET_TEMPLATE(airplaneConfig_t, airplaneConfig, PG_AIRPLANE_CONFIG, 0);

PG_RESET_TEMPLATE(airplaneConfig_t, airplaneConfig,
    .fixedwing_althold_reversed = false
);

static int32_t setVelocity = 0;
static uint8_t velocityControl = 0;
static int32_t errorVelocityI = 0;
static int32_t altHoldThrottleAdjustment = 0;
static int16_t initialThrottleHold;


// from APM
#define _THST_HOVER_DEFAULT    0.35f   // the estimated hover throttle, 0 ~ 1
#define _THST_HOVER_TC         10.0f   // time constant used to update estimated hover throttle, 0 ~ 1
#define _THST_HOVER_MIN        0.125f  // minimum possible hover throttle
#define _THST_HOVER_MAX        0.6875f // maximum possible hover throttle
static float throttleHover = _THST_HOVER_DEFAULT; // throttle, 0 ~ 1
static float acc_errI = 0.0f;


#define DEGREES_80_IN_DECIDEGREES 800

static void applyMultirotorAltHold(void)
{
    static uint8_t isAltHoldChanged = 0;
    // multirotor alt hold
    if (rcControlsConfig()->alt_hold_fast_change) {
        // rapid alt changes
        if (ABS(rcData[THROTTLE] - initialThrottleHold) > rcControlsConfig()->alt_hold_deadband) {
            errorVelocityI = 0;
            isAltHoldChanged = 1;
            rcCommand[THROTTLE] += (rcData[THROTTLE] > initialThrottleHold) ? -rcControlsConfig()->alt_hold_deadband : rcControlsConfig()->alt_hold_deadband;
        } else {
            if (isAltHoldChanged) {
                AltHold = estimatedAltitude;
                isAltHoldChanged = 0;
            }
            rcCommand[THROTTLE] = constrain(getHoverThrottle() + altHoldThrottleAdjustment, PWM_RANGE_MIN, PWM_RANGE_MAX);
        }
    } else {
        // slow alt changes, mostly used for aerial photography
        if (ABS(rcData[THROTTLE] - rxConfig()->midrc) > rcControlsConfig()->alt_hold_deadband) {
            // set velocity proportional to stick movement +100 throttle gives ~ +50 cm/s
            setVelocity = (rcData[THROTTLE] - rxConfig()->midrc) / 2;
            setVelocity = applyDeadband(setVelocity, rcControlsConfig()->alt_hold_deadband);
            velocityControl = 1;
            isAltHoldChanged = 1;
        } else if (isAltHoldChanged) {
            AltHold = estimatedAltitude;
            velocityControl = 0;
            isAltHoldChanged = 0;
        }
        rcCommand[THROTTLE] = constrain(getHoverThrottle() + altHoldThrottleAdjustment, PWM_RANGE_MIN, PWM_RANGE_MAX);
    }
}

static void applyFixedWingAltHold(void)
{
    // handle fixedwing-related althold. UNTESTED! and probably wrong
    // most likely need to check changes on pitch channel and 'reset' althold similar to
    // how throttle does it on multirotor

    rcCommand[PITCH] += altHoldThrottleAdjustment * GET_DIRECTION(airplaneConfig()->fixedwing_althold_reversed);
}

void applyAltHold(void)
{
    if (STATE(FIXED_WING)) {
        applyFixedWingAltHold();
    } else {
        applyMultirotorAltHold();
    }
}

void updateAltHoldState(void)
{
    // Baro alt hold activate
    if (!IS_RC_MODE_ACTIVE(BOXBARO)) {
        DISABLE_FLIGHT_MODE(BARO_MODE);
        return;
    }

    if (!FLIGHT_MODE(BARO_MODE)) {
        ENABLE_FLIGHT_MODE(BARO_MODE);
        AltHold = estimatedAltitude;
        initialThrottleHold = rcData[THROTTLE];
        errorVelocityI = 0;
        altHoldThrottleAdjustment = 0;
        acc_errI = 0.0f;
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
        AltHold = estimatedAltitude;
        initialThrottleHold = rcData[THROTTLE];
        errorVelocityI = 0;
        altHoldThrottleAdjustment = 0;
    }
}

bool isThrustFacingDownwards(attitudeEulerAngles_t *attitude)
{
    return ABS(attitude->values.roll) < DEGREES_80_IN_DECIDEGREES && ABS(attitude->values.pitch) < DEGREES_80_IN_DECIDEGREES;
}

int32_t calculateAltHoldThrottleAdjustment(int32_t alt, float vel, float accZ)
{
    int32_t error;
    float result;
    float setVel;
    float setAcc;

    static pt1Filter_t accFilter;
    static float accZ_old = 0.0f;
//    static float acc_errI = 0.0f;

    if (!isThrustFacingDownwards(&attitude)) {
        return 0;
    }

    // Altitude P-Controller
    // APM def = 1.0, range = 1.0 ~ 3.0
    // BF 64, scale = 64
    if (!velocityControl) {
        error = constrain(AltHold - alt, -500, 500);
        error = applyDeadband(error, 10); // remove small P parameter to reduce noise near zero position
        setVel = constrain((currentPidProfile->pid[PID_ALT].P * error / 128), -300, +300); // limit velocity to +/- 3 m/s
    } else {
        setVel = setVelocity;
    }
    // Velocity P-Controller

    // P
    // APM def = 5.0, range = 1.0 ~ 8.0
    // BF 80, scale = 16
    setAcc = setVel - vel;
    setAcc = constrainf((currentPidProfile->pid[PID_ALT].D / 16.0f) * setAcc, -250.0f, +250.0f);

    // calculate rate error and filter with cut off frequency of 2 Hz
    setAcc = pt1FilterApply4(&accFilter, setAcc, 2, 0.025f);

    // Acceleration PID-Controller

    // P
    // APM def = 0.5, range = 0.5 ~ 1.5 (inc = 0.05)
    // BF 16, scale = 256
    float accZ_err = setAcc - (accZ * acc1GScale * 100.0f); // in cm/s/s
    result = constrainf((currentPidProfile->pid[PID_VEL].P * accZ_err / 256.0f), -300.0f, +300.0f);

    // I
    // APM def = 1.0, range = 0.0 ~ 3.0, IMAX = 800
    // BF 32, scale = 1024
    acc_errI += (currentPidProfile->pid[PID_VEL].I / 1024.0f) * accZ_err;
    acc_errI = constrainf(acc_errI, -800.0f, 800.0f);     // I in range +/-800
    result += acc_errI;

    // D
    // APM def = 0.0, range = 0.0 ~ 0.4
    // BF 0, scale = 32
    result -= constrainf(currentPidProfile->pid[PID_VEL].D * (accZ + accZ_old) / 512, -150.0f, 150.0f);

//printf("[accZ]%.3f,%.3f,%.3f,%.3f,%.3f\n", setVel, setAcc, accZ_err, acc_errI, dT);

    accZ_old = accZ;

    return result;
}
/*
int32_t calculateAltHoldThrottleAdjustment(float alt, float vel, float accZ_tmp)
{
    int32_t result = 0;
    int32_t error;
    int32_t setVel;

    static float accZ_old = 0.0f;


    if (!isThrustFacingDownwards(&attitude)) {
        return result;
    }

    // Altitude P-Controller

    if (!velocityControl) {
        error = constrain(AltHold - alt, -500, 500);
        //error = applyDeadband(error, 10); // remove small P parameter to reduce noise near zero position
        setVel = constrain((currentPidProfile->pid[PID_ALT].P * error / 128), -300, +300); // limit velocity to +/- 3 m/s
    } else {
        setVel = setVelocity;
    }
    // Velocity PID-Controller

    // P
    error = setVel - vel;
    result = constrain((currentPidProfile->pid[PID_VEL].P * error / 32), -300, +300);

    // I
    errorVelocityI += (currentPidProfile->pid[PID_VEL].I * error);
    errorVelocityI = constrain(errorVelocityI, -(8192 * 800), (8192 * 800));
    result += errorVelocityI / 8192;     // I in range +/-800

    // D
    result -= constrain(currentPidProfile->pid[PID_VEL].D * (accZ_tmp + accZ_old) / 512, -150, 150);

    accZ_old = accZ_tmp;

    return result;
}
*/
void calculateHoverThrottle(uint32_t deltaT)
{
    const uint16_t thmin = motorConfig()->minthrottle;
    const uint16_t throttle = getThrottle();

    // calc average throttle if we are in a level hover
    // climb_rate < 60 cm/s
    // roll & pitch  < 5 deg
    if (ARMING_FLAG(ARMED) && throttle > thmin && ABS(vel) < 30 && ABS(attitude.values.roll) < 50 && ABS(attitude.values.pitch) < 50) {
        const float dt = deltaT * 10e-6f;
        throttleHover = constrain(throttleHover + dt * (throttle - throttleHover) / (dt + _THST_HOVER_TC), _THST_HOVER_MIN, _THST_HOVER_MAX);
    }
}

uint16_t getHoverThrottle()
{
    return (uint16_t)(throttleHover * (PWM_RANGE_MAX - PWM_RANGE_MIN)) + PWM_RANGE_MIN;
}
#endif // USE_ALT_HOLD

#if defined(USE_BARO) || defined(USE_SONAR)
void calculateEstimatedAltitude(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimeUs = 0;
    const uint32_t dTime = currentTimeUs - previousTimeUs;
    if (dTime < BARO_UPDATE_FREQUENCY_40HZ) {
        return;
    }
    previousTimeUs = currentTimeUs;



    int32_t baroAlt = 0;
#ifdef USE_BARO
    if (sensors(SENSOR_BARO)) {
        if (!isBaroCalibrationComplete()) {
            performBaroCalibrationCycle();
            vel = 0;
            accAlt = 0;
        } else {
            baroAlt = baroCalculateAltitude();
            estimatedAltitude = baroAlt;
        }
    }
#endif

#ifdef USE_SONAR
    if (sensors(SENSOR_SONAR)) {
        int32_t sonarAlt = sonarCalculateAltitude(sonarRead(), getCosTiltAngle());
        if (sonarAlt > 0 && sonarAlt >= sonarCfAltCm && sonarAlt <= sonarMaxAltWithTiltCm) {
            // SONAR in range, so use complementary filter
            float sonarTransition = (float)(sonarMaxAltWithTiltCm - sonarAlt) / (sonarMaxAltWithTiltCm - sonarCfAltCm);
            sonarAlt = (float)sonarAlt * sonarTransition + baroAlt * (1.0f - sonarTransition);
            estimatedAltitude = sonarAlt;
        }
    }
#endif

    float accZ_tmp = 0;
#ifdef USE_ACC
    if (sensors(SENSOR_ACC)) {
        const float dt = accTimeSum * 1e-6f; // delta acc reading time in seconds

        // Integrator - velocity, cm/sec
        if (accSumCount) {
            accZ_tmp = (float)accSum[2] / accSumCount;
        }
        const float vel_acc = accZ_tmp * accVelScale * (float)accTimeSum;

        // Integrator - Altitude in cm
        accAlt += (vel_acc * 0.5f) * dt + vel * dt;  // integrate velocity to get distance (x= a/2 * t^2)
        accAlt = accAlt * CONVERT_PARAMETER_TO_FLOAT(barometerConfig()->baro_cf_alt) + (float)baroAlt * (1.0f - CONVERT_PARAMETER_TO_FLOAT(barometerConfig()->baro_cf_alt));    // complementary filter for altitude estimation (baro & acc)
        vel += vel_acc;
        estimatedAltitude = accAlt;
    }
#endif

    DEBUG_SET(DEBUG_ALTITUDE, DEBUG_ALTITUDE_ACC, accSum[2] / accSumCount);
    DEBUG_SET(DEBUG_ALTITUDE, DEBUG_ALTITUDE_VEL, vel);
    DEBUG_SET(DEBUG_ALTITUDE, DEBUG_ALTITUDE_HEIGHT, accAlt);

    imuResetAccelerationSum();

    int32_t baroVel = 0;
#ifdef USE_BARO
    if (sensors(SENSOR_BARO)) {
        if (!isBaroCalibrationComplete()) {
            return;
        }

        static int32_t lastBaroAlt = 0;
        baroVel = (baroAlt - lastBaroAlt) * 1000000.0f / dTime;
        lastBaroAlt = baroAlt;

        baroVel = constrain(baroVel, -1500, 1500);  // constrain baro velocity +/- 1500cm/s
        baroVel = applyDeadband(baroVel, 10);       // to reduce noise near zero
    }
#endif // USE_BARO

    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    vel = vel * CONVERT_PARAMETER_TO_FLOAT(barometerConfig()->baro_cf_vel) + baroVel * (1.0f - CONVERT_PARAMETER_TO_FLOAT(barometerConfig()->baro_cf_vel));
    int32_t vel_tmp = lrintf(vel);

    // set vario
    estimatedVario = applyDeadband(vel_tmp, 5);

#ifdef USE_ALT_HOLD
printf("[ALT_HOLD]%.3f,%.3f,%.3f\n", accAlt, vel, accZ_tmp);
    altHoldThrottleAdjustment = calculateAltHoldThrottleAdjustment(accAlt, vel, accZ_tmp);
#else
    UNUSED(accZ_tmp);
#endif
}
#endif // USE_BARO || USE_SONAR

int32_t getEstimatedAltitude(void)
{
    return estimatedAltitude;
}

int32_t getEstimatedVario(void)
{
    return estimatedVario;
}

void setAltitude(float altitude, uint32_t dTime)
{
    static float lastAlt = 0;

    accAlt = altitude;
    vel = (accAlt - lastAlt) * 1000000.0f / dTime;

    lastAlt = accAlt;
}


