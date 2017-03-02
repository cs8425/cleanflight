/*
 * This file is part of Betaflight.
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

#include <stdint.h>

#include "platform.h"

#include "common/streambuf.h"
#include "common/utils.h"

#include "msp/msp_extcmd.h"

#ifdef USE_EXTCMD

#ifdef USE_NAV
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "config/config_master.h"

#include "drivers/system.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"

#include "rx/rx.h"

#define NAV_X 0
#define NAV_Y 1
#define NAV_Z 2

int32_t NAV_curr[3];
int16_t NAV_currhead;
int32_t NAV_hold[3];
int16_t NAV_holdhead;

#define NAV_PID_ROLL    0
#define NAV_PID_PITCH   1
#define NAV_PID_YAW     2
#define NAV_PID_ALT     3
#define NAV_PID_VALT    4
#define NAV_PID_TOTAL   5
pidi_t NAV_pid[NAV_PID_TOTAL];

static int32_t doPIDI(pidi_t *pid, int32_t target, int32_t current, timeUs_t dt)
{
	int32_t result;
	pid->prevError = pid->error;
	pid->target = target;

	// P
	pid->error = target - current;
	result = pid->kp * pid->error;

	// I
	pid->integ += pid->ki * pid->error * dt;
	if(pid->integMax) pid->integ = constrain(pid->integ, -pid->integMax, pid->integMax);
	result += pid->integ;

	// D
	result -= (pid->kd * (pid->error + pid->prevError)) / (2 * dt);

	return result / pid->scale;
}


//int16_t rcCommand[4];           // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
static timeUs_t previousTimeUs = 0; //micros();
static int32_t last_X = 0;
static int32_t last_Y = 0;
static int32_t last_Z = 0;
void applyPosHold(void)
{
	/*if (!IS_RC_MODE_ACTIVE(BOXGPSHOLD)) {
		return;
	}*/
	if (!FLIGHT_MODE(BOXGPSHOLD)) {
		return;
	}


	timeUs_t currentTimeUs = micros();
	timeUs_t dt = currentTimeUs - previousTimeUs;
	previousTimeUs = currentTimeUs;



	float sin_yaw_y = sin_approx(DECIDEGREES_TO_DEGREES(attitude.values.yaw) * 0.0174532925f);
	float cos_yaw_x = cos_approx(DECIDEGREES_TO_DEGREES(attitude.values.yaw) * 0.0174532925f);
//rcCommand[ROLL] = (NAV_curr[NAV_X] * cos_yaw_x - NAV_curr[NAV_Y] * sin_yaw_y) / 10;
//rcCommand[PITCH] = (NAV_curr[NAV_X] * sin_yaw_y + NAV_curr[NAV_Y] * cos_yaw_x) / 10;


	// set velocity proportional to stick movement +100 throttle gives ~ +50 mm/s
	int32_t altV = doPIDI(&NAV_pid[NAV_PID_VALT], (rcData[THROTTLE] - rxConfig()->midrc) / 2, last_Z - NAV_curr[NAV_Z], dt);
	rcCommand[THROTTLE] = constrain(motorConfig()->minthrottle + altV, motorConfig()->minthrottle, motorConfig()->maxthrottle);

#ifdef DEBUG_NAV
    debug[1] = NAV_curr[NAV_Z];
    debug[2] = altV;
    debug[3] = dt;
#endif

	last_X = NAV_curr[NAV_X];
	last_Y = NAV_curr[NAV_Y];
	last_Z = NAV_curr[NAV_Z];

}

void updatePosHoldState(void)
{
    // Pos hold activate
    if (!IS_RC_MODE_ACTIVE(BOXGPSHOLD)) {
        DISABLE_FLIGHT_MODE(GPS_HOLD_MODE);
        return;
    }

    if (!FLIGHT_MODE(BOXGPSHOLD)) {
        ENABLE_FLIGHT_MODE(GPS_HOLD_MODE);
		previousTimeUs = micros();
		last_X = NAV_curr[NAV_X];
		last_Y = NAV_curr[NAV_Y];
		last_Z = NAV_curr[NAV_Z];
    }
}
#endif

void mspExtInit(void)
{
#ifdef USE_NAV
	int i;
	for(i=0; i<3; i++){
		NAV_curr[i] = 0;
		NAV_hold[i] = 0;
	}

	// init PIDs
	for(i=0; i<NAV_PID_TOTAL; i++){
		NAV_pid[i].target = 0;
		NAV_pid[i].error = 0;
		NAV_pid[i].deriv = 0;
		NAV_pid[i].prevError = 0;
		NAV_pid[i].integ = 0;
		NAV_pid[i].dt = 0;
		NAV_pid[i].scale = 0;
	}

	NAV_pid[NAV_PID_VALT].kp = 60;
	NAV_pid[NAV_PID_VALT].ki = 35;
	NAV_pid[NAV_PID_VALT].kd = 5;
	NAV_pid[NAV_PID_VALT].integMax = 600;
	NAV_pid[NAV_PID_VALT].scale = 128;

#endif
}

//MSP_RESULT_ACK, MSP_RESULT_ERROR, MSP_RESULT_NO_REPLY
mspResult_e mspProcessExtCommand(uint16_t cmd, sbuf_t *dst, sbuf_t *src, mspPostProcessFnPtr *mspPostProcessFn)
{
	UNUSED(mspPostProcessFn);
	int ret = MSP_RESULT_ACK;
	uint16_t packSn = sbufReadU16(src);
	uint8_t flag;

	switch (cmd) {
#ifdef USE_NAV
	case MSPEX_SET_TPOS:
		for(int i=0; i<3; i++){
			NAV_hold[i] = (int32_t)sbufReadU32(src);
			sbufWriteU32(dst, (uint32_t)NAV_hold[i]);
		}
		NAV_holdhead = (int16_t)sbufReadU16(src);
		sbufWriteU16(dst, (uint16_t)NAV_holdhead);
		break;

	case MSPEX_SET_CPOS:
		flag = sbufReadU8(src);
		if(flag & 0x01) NAV_curr[NAV_X] = (int32_t)sbufReadU32(src);
		if(flag & 0x02) NAV_curr[NAV_Y] = (int32_t)sbufReadU32(src);
		if(flag & 0x04) NAV_curr[NAV_Z] = (int32_t)sbufReadU32(src);
		if(flag & 0x08) NAV_currhead = (int16_t)sbufReadU16(src);
		sbufWriteU8(dst, 'A');
		sbufWriteU8(dst, 'C');
		break;
#endif

	default:
		// we do not know how to handle the (valid) message, indicate error MSP $M!
		return MSP_RESULT_ERROR;
	}

	return ret;
}
#endif

