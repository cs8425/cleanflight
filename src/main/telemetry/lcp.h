/*
 * Lcp.h
 *
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

#pragma once

typedef enum {
    LCP_IDLE,
    LCP_HEADER_START,
    LCP_HEADER_HSIZE,
    LCP_HEADER_LSIZE,
    LCP_HEADER_HCMD,
    LCP_HEADER_LCMD,
    LCP_COMMAND_END,
    LCP_COMMAND_RECEIVED
} lcpState_e;

typedef struct lcpPacket_s {
    sbuf_t buf;
    uint16_t cmd;
    uint16_t dataSize;
    uint8_t state;
} lcpPacket_t;

void initLcpTelemetry(void);
void handleLcpTelemetry(void);
void checkLcpTelemetryState(void);

void freeLcpTelemetryPort(void);
void configureLcpTelemetryPort(void);

