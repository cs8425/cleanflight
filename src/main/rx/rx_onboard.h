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

// RX auto switching for On-Board-Computer

#pragma once

#include "common/time.h"

#define OBC_DEADBAND 50
#define OBC_RX_TIMEOUT 400 // 400mS
#define OBC_SWITCH_TH 1600
#define OBC_SWITCH_CH 6 // RPYT1234 => 0, 1, 2, 3, 4, 5, 6

typedef struct rxOBCRuntimeConfig_s {
    rxRuntimeConfig_t   rxRuntime; // fall-back RX
    uint8_t            switchChannel; // input channel for switch input RX
} rxOBCRuntimeConfig_t;

typedef struct rxAutoSwitch_s {
    timeMs_t   lastUpdate;
    bool      timeout; // RX frame timeout
    uint16_t  frame[MAX_SUPPORTED_RC_CHANNEL_COUNT]; // input channel data
} rxAutoSwitch_t;

bool rxOBCInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig);
void rxOBCFrameReceive(uint16_t *frame, int channelCount);



