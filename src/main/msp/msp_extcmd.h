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

#pragma once

#ifdef USE_EXTCMD
#include "msp/msp.h"


#ifdef USE_NAV
#include "common/time.h"

#define MSPEX_SET_TPOS               01    //in message      set target location
#define MSPEX_SET_CPOS               02    //in message      set current location
#define MSPEX_SET_PID                03    //in message      set PID
extern int32_t NAV_curr[3];
extern int32_t NAV_hold[3];

// thank's crazyflie
// https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/interface/pid.h
typedef struct
{
	float target;       //< set point
	float error;        //< error
	float deriv;        //< derivative
	float prevError;    //< previous error
	float integ;        //< integral
	float integMax;     //< integral limit
	float kp;           //< proportional gain
	float ki;           //< integral gain
	float kd;           //< derivative gain
	timeUs_t dt;         //< delta-time dt
} pidf_t;

typedef struct
{
	int32_t target;       //< set point
	int32_t error;        //< error
	int32_t deriv;        //< derivative
	int32_t prevError;    //< previous error
	int32_t integ;        //< integral
	int32_t integMax;     //< integral limit
	int32_t kp;           //< proportional gain
	int32_t ki;           //< integral gain
	int32_t kd;           //< derivative gain
	int32_t scale;        //< all gain
	timeUs_t dt;           //< delta-time dt
} pidi_t;

void applyPosHold(void);
void updatePosHoldState(void);

#endif

void mspExtInit(void);
mspResult_e mspProcessExtCommand(uint16_t cmd, sbuf_t *dst, sbuf_t *src, mspPostProcessFnPtr *mspPostProcessFn);

#endif
