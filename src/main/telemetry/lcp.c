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

/*
 * LightTelemetry from KipK
 *
 * Minimal one way telemetry protocol for really bitrates (1200/2400 bauds).
 * Effective for ground OSD, groundstation HUD and Antenna tracker
 * http://www.wedontneednasa.com/2014/02/lighttelemetry-v2-en-route-to-ground-osd.html
 *
 * This implementation is for LTM v2 > 2400 baud rates
 *
 * Cleanflight implementation by Jonathan Hudson
 */

//#include <stdio.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef TELEMETRY

#include "build/build_config.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"
#include "common/streambuf.h"

#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "io/serial.h"
#include "io/gimbal.h"
#include "io/gps.h"
//#include "io/ledstrip.h"
//#include "io/beeper.h"

#include "rx/rx.h"
#include "rx/msp.h"
#include "rx/rx_onboard.h"

//#include "flight/mixer.h"
//#include "flight/pid.h"
#include "flight/imu.h"
//#include "flight/failsafe.h"
#include "flight/altitude.h"
//#include "flight/navigation.h"

#include "telemetry/telemetry.h"
#include "telemetry/lcp.h"


#define TELEMETRY_LCP_INITIAL_PORT_MODE MODE_RXTX
#define LCP_CYCLETIME   100

static serialPort_t *lcpPort;
static serialPortConfig_t *portConfig;
static bool lcpEnabled;
static portSharing_e lcpPortSharing;
static uint8_t lcp_crc;

#define LCP_PORT_INBUF_SIZE 512
static uint8_t lcp_inBuf[LCP_PORT_INBUF_SIZE];
static lcpPacket_t pkt;

static void lcp_initialise_packet(uint8_t ltm_id)
{
    lcp_crc = 0;
    serialWrite(lcpPort, '$');
    serialWrite(lcpPort, 'L');
    serialWrite(lcpPort, ltm_id);
}

static void lcp_serialise_8(uint8_t v)
{
    serialWrite(lcpPort, v);
    lcp_crc ^= v;
}

static void lcp_serialise_16(uint16_t v)
{
    lcp_serialise_8((uint8_t)v);
    lcp_serialise_8((v >> 8));
}

static void lcp_serialise_32(uint32_t v)
{
    lcp_serialise_8((uint8_t)v);
    lcp_serialise_8((v >> 8));
    lcp_serialise_8((v >> 16));
    lcp_serialise_8((v >> 24));
}

static void lcp_finalise(void)
{
    serialWrite(lcpPort, lcp_crc);
    serialWrite(lcpPort, '\n');
}

/*
 * GPS G-frame 5Hhz at > 2400 baud
 * LAT LON SPD ALT SAT/FIX
 */
/*static void ltm_gframe(void)
{
#if defined(GPS)
    uint8_t gps_fix_type = 0;
    int32_t ltm_alt;

    if (!sensors(SENSOR_GPS))
        return;

    if (!STATE(GPS_FIX))
        gps_fix_type = 1;
    else if (GPS_numSat < 5)
        gps_fix_type = 2;
    else
        gps_fix_type = 3;

    ltm_initialise_packet('G');
    ltm_serialise_32(GPS_coord[LAT]);
    ltm_serialise_32(GPS_coord[LON]);
    ltm_serialise_8((uint8_t)(GPS_speed / 100));

#if defined(BARO) || defined(SONAR)
    ltm_alt = (sensors(SENSOR_SONAR) || sensors(SENSOR_BARO)) ? getEstimatedAltitude() : GPS_altitude * 100;
#else
    ltm_alt = GPS_altitude * 100;
#endif
    ltm_serialise_32(ltm_alt);
    ltm_serialise_8((GPS_numSat << 2) | gps_fix_type);
    ltm_finalise();
#endif
}
*/
/*
 * Sensors S-frame 5Hhz at > 2400 baud
 * VBAT(mv)  Current(ma)   RSSI  AIRSPEED  ARM/FS/FMOD
 * Flight mode(0-19):
 *     0: Manual, 1: Rate, 2: Attitude/Angle, 3: Horizon,
 *     4: Acro, 5: Stabilized1, 6: Stabilized2, 7: Stabilized3,
 *     8: Altitude Hold, 9: Loiter/GPS Hold, 10: Auto/Waypoints,
 *     11: Heading Hold / headFree, 12: Circle, 13: RTH, 14: FollowMe,
 *     15: LAND, 16:FlybyWireA, 17: FlybywireB, 18: Cruise, 19: Unknown
 */

/*static void ltm_sframe(void)
{
    uint8_t lt_flightmode;
    uint8_t lt_statemode;
    if (FLIGHT_MODE(PASSTHRU_MODE))
        lt_flightmode = 0;
    else if (FLIGHT_MODE(GPS_HOME_MODE))
        lt_flightmode = 13;
    else if (FLIGHT_MODE(GPS_HOLD_MODE))
        lt_flightmode = 9;
    else if (FLIGHT_MODE(HEADFREE_MODE))
        lt_flightmode = 4;
    else if (FLIGHT_MODE(BARO_MODE))
        lt_flightmode = 8;
    else if (FLIGHT_MODE(ANGLE_MODE))
        lt_flightmode = 2;
    else if (FLIGHT_MODE(HORIZON_MODE))
        lt_flightmode = 3;
    else
        lt_flightmode = 1;      // Rate mode

    lt_statemode = (ARMING_FLAG(ARMED)) ? 1 : 0;
    if (failsafeIsActive())
        lt_statemode |= 2;
    ltm_initialise_packet('S');
    ltm_serialise_16(getBatteryVoltage() * 100);    //vbat converted to mv
    ltm_serialise_16(0);             //  current, not implemented
    ltm_serialise_8((uint8_t)((rssi * 254) / 1023));        // scaled RSSI (uchar)
    ltm_serialise_8(0);              // no airspeed
    ltm_serialise_8((lt_flightmode << 2) | lt_statemode);
    ltm_finalise();
}*/

/*
 * Attitude A-frame - 10 Hz at > 2400 baud
 *  PITCH ROLL HEADING
 */
static void lcp_aframe(void)
{
    lcp_initialise_packet('A');
    lcp_serialise_16(DECIDEGREES_TO_DEGREES(attitude.values.pitch));
    lcp_serialise_16(DECIDEGREES_TO_DEGREES(attitude.values.roll));
    lcp_serialise_16(DECIDEGREES_TO_DEGREES(attitude.values.yaw));
    lcp_finalise();
}

/*
 * G-frame 5Hhz
 * ALT, VSPD
 */
static void lcp_gframe(void)
{
    lcp_initialise_packet('G');
    lcp_serialise_32(getEstimatedAltitude()); // ALT
    lcp_serialise_32(getEstimatedVario()); // VSPD
    lcp_finalise();

}

static void process_lcp(void)
{

    static uint8_t lcp_scheduler;
    lcp_aframe();

    if (lcp_scheduler & 1) lcp_gframe();

    lcp_scheduler++;
    lcp_scheduler %= 10;

}

static void lcpProcessReceivedData(lcpPacket_t *lcpPkt, uint8_t c)
{
    static uint8_t checksum; // XOR: len + cmd + data
    static uint16_t offset;

    if (lcpPkt->state == LCP_IDLE) {
        if (c == '$') {
            lcpPkt->state = LCP_HEADER_START;
        } else {
            return;
        }
    } else if (lcpPkt->state == LCP_HEADER_START) { // low byte
        lcpPkt->dataSize = c;
        checksum = 0;
        checksum ^= c;
        lcpPkt->state = LCP_HEADER_LSIZE;
    } else if (lcpPkt->state == LCP_HEADER_LSIZE) { // high byte
        lcpPkt->dataSize |= (c << 8);

        if (lcpPkt->dataSize > LCP_PORT_INBUF_SIZE) {
            lcpPkt->state = LCP_IDLE;
        } else {
            offset = 0;
            checksum ^= c;
            lcpPkt->state = LCP_HEADER_HSIZE;
        }
    } else if (lcpPkt->state == LCP_HEADER_HSIZE) { // low byte
        lcpPkt->cmd = c;
        checksum ^= c;
        lcpPkt->state = LCP_HEADER_LCMD;
    } else if (lcpPkt->state == LCP_HEADER_LCMD) { // high byte
        lcpPkt->cmd |= (c << 8);
        checksum ^= c;
        lcpPkt->state = LCP_HEADER_HCMD;
    } else if (lcpPkt->state == LCP_HEADER_HCMD && offset < lcpPkt->dataSize) {
        checksum ^= c;
        lcp_inBuf[offset++] = c;
    } else if (lcpPkt->state == LCP_HEADER_HCMD && offset >= lcpPkt->dataSize) {
        if (checksum == c) {
            lcpPkt->state = LCP_COMMAND_END;
        } else {
            lcpPkt->state = LCP_IDLE;
        }
    } else if (lcpPkt->state == LCP_COMMAND_END) {
        if (c == '\n') {
            lcpPkt->state = LCP_COMMAND_RECEIVED;
        } else {
            lcpPkt->state = LCP_IDLE;
        }
    }
    return;
}

void handleLcpTelemetry(void)
{
    static uint32_t ltm_lastCycleTime;
    uint32_t now;
    if (!lcpEnabled)
        return;
    if (!lcpPort)
        return;
    now = millis();
    if ((now - ltm_lastCycleTime) >= LCP_CYCLETIME) {
        process_lcp();
        ltm_lastCycleTime = now;
    }

    while (serialRxBytesWaiting(lcpPort)) {
        const uint8_t c = serialRead(lcpPort);
        lcpProcessReceivedData(&pkt, c);
//		printf("got LCP char: %x, %u\n", c, pkt.state);

        if (pkt.state == LCP_COMMAND_RECEIVED) {
//			printf("got LCP CMD: %u, %u\n", pkt.cmd, pkt.dataSize);

            sbuf_t src = {
                .ptr = lcp_inBuf,
                .end = lcp_inBuf + LCP_PORT_INBUF_SIZE,
            };

            if (pkt.cmd == 0x01) { // raw RC command
                uint8_t channelCount = pkt.dataSize / sizeof(uint16_t);
                uint16_t frame[MAX_SUPPORTED_RC_CHANNEL_COUNT];
                for (int i = 0; i < channelCount; i++) {
                    frame[i] = sbufReadU16(&src);
                }
//                rxMspFrameReceive(frame, channelCount);
                rxOBCFrameReceive(frame, channelCount);
            }

            if (pkt.cmd == 0x02) { // set Altitude command
                int32_t alt = (int32_t) sbufReadU32(&src);
                int32_t vspd = (int32_t) sbufReadU32(&src);
                setEstimatedAltitude(alt);
                //setEstimatedVario(vspd);
                UNUSED(vspd);
            }

            pkt.state = LCP_IDLE;
            break; // process one command at a time so as not to block.
        }
    }
}

void freeLcpTelemetryPort(void)
{
    closeSerialPort(lcpPort);
    lcpPort = NULL;
    lcpEnabled = false;
}

void initLcpTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_LCP);
    lcpPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_LCP);
}

void configureLcpTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }
    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        baudRateIndex = BAUD_57600;
    }
    lcpPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_LCP, NULL, baudRates[baudRateIndex], TELEMETRY_LCP_INITIAL_PORT_MODE, SERIAL_NOT_INVERTED);
    if (!lcpPort)
        return;
    lcpEnabled = true;
}

void checkLcpTelemetryState(void)
{
    if (portConfig && telemetryCheckRxPortShared(portConfig)) {
        if (!lcpEnabled && telemetrySharedPort != NULL) {
            lcpPort = telemetrySharedPort;
            lcpEnabled = true;
        }
    } else {
        bool newTelemetryEnabledValue = telemetryDetermineEnabledState(lcpPortSharing);
        if (newTelemetryEnabledValue == lcpEnabled)
            return;
        if (newTelemetryEnabledValue)
            configureLcpTelemetryPort();
        else
            freeLcpTelemetryPort();
    }
}
#endif
