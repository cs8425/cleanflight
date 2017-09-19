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


#include <platform.h>

#ifdef USE_RX_OBC
//#include <stdio.h>

#include "common/maths.h"
#include "config/feature.h"

#include "drivers/io.h"
#include "drivers/time.h"

#include "rx/rx.h"
#include "rx/rx_onboard.h"


/*
currentRx: 0 =>  OBC, 1 => RX1

ReadRawRC:
use | RX1-timeout | OBC-timeout | RX1-change | OBC-change | currentRx |
=======================================================================
OBC |           F |           F |          F |          X |         0 |
RX1 |           F |           F |          T |          X |         0 |
-----------------------------------------------------------------------
RX1 |           F |           F |          X |          X |         1 |
-----------------------------------------------------------------------
RX1 |           F |           T |          X |          X |         X |
OBC |           T |           F |          X |          X |         X |
-----------------------------------------------------------------------
ERR |           T |           T |          X |          X |         X |
-----------------------------------------------------------------------

FrameStatus:
out  | RX1-timeout | OBC-timeout | RX1-update | OBC-update | currentRx |
========================================================================
PEND |           F |           F |          F |          F |         0 |
COMP |           F |           F |          T |          F |         0 |
COMP |           F |           F |          X |          T |         0 |
------------------------------------------------------------------------
PEND |           F |           F |          F |          X |         1 |
COMP |           F |           F |          T |          X |         1 |
------------------------------------------------------------------------
PEND |           F |           T |          F |          X |         X |
COMP |           F |           T |          T |          X |         X |
PEND |           T |           F |          X |          F |         X |
COMP |           T |           F |          X |          T |         X |
------------------------------------------------------------------------
PEND |           T |           T |          X |          X |         X |
------------------------------------------------------------------------


out  | RX1-timeout | OBC-timeout | RX1-update | OBC-update | currentRx |
========================================================================
PEND |           F |           F |          F |          F |         0 |
PEND |           F |           F |          F |          F |         1 |
------------------------------------------------------------------------
PEND |           F |           F |          F |          T |         1 |
COMP |           F |           F |          F |          T |         0 |
------------------------------------------------------------------------
COMP |           F |           F |          T |          F |         0 |
COMP |           F |           F |          T |          F |         1 |
COMP |           F |           F |          T |          T |         0 |
COMP |           F |           F |          T |          T |         1 |
------------------------------------------------------------------------
PEND |           F |           T |          F |          X |         X |
COMP |           F |           T |          T |          X |         X |
------------------------------------------------------------------------
PEND |           T |           F |          X |          F |         X |
COMP |           T |           F |          X |          T |         X |
------------------------------------------------------------------------
PEND |           T |           T |          X |          X |         X |
------------------------------------------------------------------------

*/

static rxOBCRuntimeConfig_t rxOBCRuntimeConfig;

static rxAutoSwitch_t OBCStatus;
static rxAutoSwitch_t RXStatus;

static bool rxOBCFrameDone = false;

static uint32_t changeMap = 0;
static uint8_t currentRx = 0; // for force switch, 1 >> fall-back

uint16_t rxOBCReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t channel)
{
    if (channel >= rxRuntimeConfig->channelCount) {
        return 0;
    }

    // channel T1234, force return RX
    if (channel > 2) {
        return RXStatus.frame[channel];
    }

    if (OBCStatus.timeout) {
        return RXStatus.frame[channel];
    }

    if (RXStatus.timeout) {
        return OBCStatus.frame[channel];
    }

    // force use RX
    if (currentRx == 1) {
        return RXStatus.frame[channel];
    }

    uint32_t mask = (1 << channel);
    if (changeMap & mask) {
        changeMap &= ~mask;
        return RXStatus.frame[channel];
    } else {
        return OBCStatus.frame[channel];
    }
}

/*
 * Returns true if the RX has received new data.
 * Called from updateRx in rx.c, updateRx called from taskUpdateRxCheck.
 * If taskUpdateRxCheck returns true, then taskUpdateRxMain will shortly be called.
 */
static uint8_t rxOBCFrameStatus(void)
{
    uint8_t status = 0; // bit: RX1-timeout | OBC-timeout | RX1-update | OBC-update
    timeMs_t nowMs = millis();

    // check OBC
    if (rxOBCFrameDone) {
        status |= 1 << 0;
        rxOBCFrameDone = false;
    }
    // OBC timeout check
    if (cmpTimeUs(nowMs, OBCStatus.lastUpdate) > OBC_RX_TIMEOUT) {
        status |= 1 << 2;
        OBCStatus.timeout = true;
//printf("[RX]OBC timeout!!\n");
    }


    // check fall-back RX
    const uint8_t frameStatus = rxOBCRuntimeConfig.rxRuntime.rcFrameStatusFn();
    if (frameStatus & RX_FRAME_COMPLETE) {
        status |= 1 << 1;

        rxRuntimeConfig_t *rxRuntimeConfig = &rxOBCRuntimeConfig.rxRuntime;
        rcReadRawDataFnPtr rcReadRawFn = rxRuntimeConfig->rcReadRawFn;
        int channelCount = rxRuntimeConfig->channelCount;

        for (int i = 0; i < channelCount; i++) {
            uint16_t newData = rcReadRawFn(rxRuntimeConfig, i);

            // check change
/*            if (ABS(newData - rxFrame[i]) >= OBC_DEADBAND) {
                changeMap |= (1 << i);
            }*/

            RXStatus.frame[i] = newData;
        }

        if (RXStatus.frame[rxOBCRuntimeConfig.switchChannel] > OBC_SWITCH_TH) {
            currentRx = 0;
//printf("[RX]auto mode!!\n");
        } else {
            currentRx = 1;
        }

        RXStatus.lastUpdate = nowMs;
        RXStatus.timeout = false;
    }

    // RX timeout check
    if (cmpTimeUs(nowMs, RXStatus.lastUpdate) > OBC_RX_TIMEOUT) {
        status |= 1 << 3;
        RXStatus.timeout = true;
//printf("[RX]RX timeout!!\n");
    }


//  no update
    if (status == 0) return RX_FRAME_PENDING;

//  (OBCStatus.timeout && RXStatus.timeout)
    if ((status & 0x0c) == 0x0c) return RX_FRAME_PENDING;

//  RXStatus.timeout
    if (status & 0x08) {
        return (status & 0x01) ? RX_FRAME_COMPLETE : RX_FRAME_PENDING;
    }

//  OBCStatus.timeout
    if (status & 0x04) {
        return (status & 0x02) ? RX_FRAME_COMPLETE : RX_FRAME_PENDING;
    }

//  RX update
    if (status & 0x02) return RX_FRAME_COMPLETE;

//  OBC update
    if (status & 0x01) {
        return (currentRx == 0) ? RX_FRAME_COMPLETE : RX_FRAME_PENDING;
    }


    // should never goes here !!
//printf("[RX]ERROR!!\n");
    while(1);
}

/*
 * Set and initialize the RX protocol
 */
bool rxOBCInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{

    rxOBCRuntimeConfig.rxRuntime = *rxRuntimeConfig;

    rxOBCRuntimeConfig.switchChannel = 6;
    for(uint8_t i=0; i<RX_MAPPABLE_CHANNEL_COUNT; i++) {
//printf("[RX]rcmap[%d] = %d\n", i, rxConfig->rcmap[i]);
        if (rxConfig->rcmap[i] == OBC_SWITCH_CH) {
            rxOBCRuntimeConfig.switchChannel = i;
            break;
        }
    }
//printf("[RX]switchChannel [%d] = %d\n", rxConfig->rcmap[rxOBCRuntimeConfig.switchChannel], rxOBCRuntimeConfig.switchChannel);

    rxRuntimeConfig->rcReadRawFn = rxOBCReadRawRC;
    rxRuntimeConfig->rcFrameStatusFn = rxOBCFrameStatus;

    return true;
}

void rxOBCFrameReceive(uint16_t *frame, int channelCount)
{
    for (int i = 0; i < channelCount; i++) {
        OBCStatus.frame[i] = frame[i];
    }

    // Any channels not provided will be reset to zero
    for (int i = channelCount; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        OBCStatus.frame[i] = 0;
    }

    OBCStatus.lastUpdate = millis();
    OBCStatus.timeout = false;

    rxOBCFrameDone = true;
}

#endif
