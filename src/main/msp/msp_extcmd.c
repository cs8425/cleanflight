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
#include <math.h>
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

#define NAV_PID_X       0
#define NAV_PID_Y       1
#define NAV_PID_YAW     2
#define NAV_PID_ALT     3
#define NAV_PID_VALT    4
#define NAV_PID_TOTAL   5
pidi_t NAV_pid[NAV_PID_TOTAL];

static uint8_t initialStickPos = 0; // flag for Throttle Stick move to center

static int32_t doPIDI(pidi_t *pid, int32_t target, int32_t current, timeUs_t dt)
{
	int32_t result;
	pid->target = target;

	// P
	pid->error = target - current;
	result = pid->kp * pid->error;

	// I
	pid->integ += pid->ki * ((pid->error + pid->prevError) / 2) * dt;
	if(pid->integMax) pid->integ = constrain(pid->integ, -pid->integMax, pid->integMax);
	result += pid->integ;

	// D
	pid->deriv = (pid->error - pid->prevError) / dt;
	result += pid->kd * pid->deriv;

	pid->prevError = pid->error;

	return result / pid->scale;
}


//int16_t rcCommand[4];           // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
static timeUs_t previousTimeUs = 0;
static int32_t last_X = 0;
static int32_t last_Y = 0;
static int32_t last_Z = 0;
void applyPosHold(void)
{
	/*if (!IS_RC_MODE_ACTIVE(BOXGPSHOLD)) {
		return;
	}*/
	if (!FLIGHT_MODE(GPS_HOLD_MODE)) {
		return;
	}

	timeUs_t currentTimeUs = micros();
	timeUs_t dt = currentTimeUs - previousTimeUs;
//	if(dt < 20*1000) return;	// 50Hz
	previousTimeUs = currentTimeUs;

	int32_t dx = NAV_curr[NAV_X] - last_X;
	int32_t dy = NAV_curr[NAV_Y] - last_Y;
	int32_t dz = NAV_curr[NAV_Z] - last_Z;

//	const float radDiff = DECIDEGREES_TO_RADIANS(attitude.values.yaw);
	const float radDiff = -DECIDEGREES_TO_RADIANS(NAV_currhead - NAV_holdhead);
	const float cosDiff = cosf(radDiff);
	const float sinDiff = sinf(radDiff);

//	float hold_y = doPIDI(&NAV_pid[NAV_PID_Y], 0, dy, 1);
//	float hold_x = doPIDI(&NAV_pid[NAV_PID_X], 0, -dx, 1);
	float hold_y = doPIDI(&NAV_pid[NAV_PID_Y], NAV_curr[NAV_Y], NAV_hold[NAV_Y], 1);
	float hold_x = -doPIDI(&NAV_pid[NAV_PID_X], NAV_curr[NAV_X], NAV_hold[NAV_X], 1);

	// rcData? rcCommand?
	rcCommand[ROLL] = (rcData[ROLL] - rxConfig()->midrc) + constrain(hold_y * cosDiff + hold_x * sinDiff, -100, 100);
	rcCommand[PITCH] = (rcData[PITCH] - rxConfig()->midrc) + constrain(hold_x * cosDiff - hold_y * sinDiff, -100, 100);

	// set velocity proportional to stick movement +100 throttle gives ~ +50 mm/s
	int32_t altV = doPIDI(&NAV_pid[NAV_PID_VALT], (rcData[THROTTLE] - rxConfig()->midrc) / 2, dz, 1);
	/*if (ABS(rcData[THROTTLE] - rxConfig()->midrc) > rcControlsConfig()->alt_hold_deadband) {
		if (initialStickPos) {
			rcCommand[THROTTLE] = constrain(motorConfig()->minthrottle + altV, motorConfig()->minthrottle, motorConfig()->maxthrottle);
		}
	}*/

#ifdef DEBUG_NAV
	debug[0] = rcCommand[PITCH];
	debug[1] = NAV_currhead - NAV_holdhead;
	debug[2] = hold_x;
	debug[3] = hold_y;
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
		initialStickPos = 0;
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
		NAV_pid[i].scale = 1;

		NAV_pid[i].kp = 1;
		NAV_pid[i].ki = 0;
		NAV_pid[i].kd = 0;
	}

	NAV_pid[NAV_PID_VALT].kp = 60;
	NAV_pid[NAV_PID_VALT].ki = 70;
	NAV_pid[NAV_PID_VALT].kd = 35;
	NAV_pid[NAV_PID_VALT].integMax = 700 * 64;
	NAV_pid[NAV_PID_VALT].scale = 64;

	NAV_pid[NAV_PID_X].kp = 1;
	NAV_pid[NAV_PID_X].ki = 0;
	NAV_pid[NAV_PID_X].kd = 0;
	NAV_pid[NAV_PID_X].integMax = 0;
	NAV_pid[NAV_PID_X].scale = 64;

	NAV_pid[NAV_PID_Y].kp = 1;
	NAV_pid[NAV_PID_Y].ki = 0;
	NAV_pid[NAV_PID_Y].kd = 0;
	NAV_pid[NAV_PID_Y].integMax = 0;
	NAV_pid[NAV_PID_Y].scale = 64;
#endif
}

//MSP_RESULT_ACK, MSP_RESULT_ERROR, MSP_RESULT_NO_REPLY
mspResult_e mspProcessExtCommand(uint16_t cmd, sbuf_t *dst, sbuf_t *src, mspPostProcessFnPtr *mspPostProcessFn)
{
	UNUSED(mspPostProcessFn);
	int ret = MSP_RESULT_ACK;
	uint16_t packSn = sbufReadU16(src);
	uint8_t flag;
	uint8_t id;

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
		sbufWriteU8(dst, 'C');
		break;

	case MSPEX_SET_PID:
		id = sbufReadU8(src);
		if(id < NAV_PID_TOTAL){
			flag = sbufReadU8(src);
			if(flag & 0x01) NAV_pid[id].kp = (int32_t)sbufReadU32(src);
			if(flag & 0x02) NAV_pid[id].ki = (int32_t)sbufReadU32(src);
			if(flag & 0x04) NAV_pid[id].kd = (int32_t)sbufReadU32(src);
			if(flag & 0x08) NAV_pid[id].integMax = (int32_t)sbufReadU32(src);
			if(flag & 0x10) NAV_pid[id].scale = (int32_t)sbufReadU32(src);
			if(flag & 0x20){
				NAV_pid[id].error = 0;
				NAV_pid[id].deriv = 0;
				NAV_pid[id].prevError = 0;
				NAV_pid[id].integ = 0;
			}
			sbufWriteU16(dst, MSPEX_SET_PID);
			sbufWriteU8(dst, 'A');
		}else{
			sbufWriteU16(dst, MSPEX_SET_PID);
			sbufWriteU8(dst, 'E');
		}
		break;
#endif

	default:
		// we do not know how to handle the (valid) message, indicate error MSP $M!
		return MSP_RESULT_ERROR;
	}

	return ret;
}
#endif

