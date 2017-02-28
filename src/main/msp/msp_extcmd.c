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
int32_t NAV_curr[3];
int32_t NAV_hold[3];

void applyPosHold(void)
{
#ifdef DEBUG_NAV
    debug[1] = NAV_curr[0];
    debug[2] = NAV_curr[1];
    debug[3] = NAV_curr[2];
#endif

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
#endif
}

//MSP_RESULT_ACK, MSP_RESULT_ERROR, MSP_RESULT_NO_REPLY
mspResult_e mspProcessExtCommand(uint16_t cmd, sbuf_t *dst, sbuf_t *src, mspPostProcessFnPtr *mspPostProcessFn)
{
	UNUSED(mspPostProcessFn);
	int ret = MSP_RESULT_ACK;

	switch (cmd) {
#ifdef USE_NAV
	case MSPEX_SET_TPOS:
		for(int i=0; i<3; i++){
			NAV_hold[i] = (int32_t)sbufReadU32(src);
			sbufWriteU32(dst, (uint32_t)NAV_hold[i]);
		}
		break;

	case MSPEX_SET_CPOS:
		for(int i=0; i<3; i++){
			NAV_curr[i] = (int32_t)sbufReadU32(src);
		}
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

