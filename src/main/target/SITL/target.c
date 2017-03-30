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

#include <stdint.h>
#include <stdio.h>

//#include <platform.h>
#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/serial.h"

#include "drivers/timer.h"
#include "drivers/timer_def.h"
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {};

void systemInit(void) {
}

void systemReset(void){
}
void systemResetToBootloader(void) {
}

void ledInit(void) {
}
void timerInit(void) {
}
void serialInit(void) {
}
void motorDevInit(void) {
}
void servoDevInit(void) {
}
void dashboardInit(void) {
}
void failureMode(void) {
	printf("[failureMode]!!!");
	while(1);
}

void delayMicroseconds(uint32_t cnt) {
	UNUSED(cnt);
}
void delay(uint32_t cnt) {
	UNUSED(cnt);
}

uint32_t micros() {
	return 0;
}

uint32_t millis() {
	return 0;
}

/*bool i2cWrite(I2CDevice dev, uint8_t addr, uint8_t reg, uint8_t val) {
	UNUSED(dev);
	UNUSED(addr);
	UNUSED(reg);
	UNUSED(val);
	return true;
}*/

void serialPrint(serialPort_t *instance, const char *str) {
	UNUSED(instance);
	puts(str);
}
void serialWrite(serialPort_t *instance, uint8_t ch) {
	UNUSED(instance);
	putchar(ch);
}


