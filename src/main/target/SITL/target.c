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
#define printf printf
#define sprintf sprintf

#include <errno.h>
//#include <poll.h>
//#include <sys/time.h>
//#include <unistd.h>
#include <time.h>

#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/serial.h"
#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/light_led.h"

#include "drivers/timer.h"
#include "drivers/timer_def.h"
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {};

static struct timespec start_time;

void systemInit(void) {
	clock_gettime(CLOCK_MONOTONIC, &start_time);
	printf("[system]Init...\n");
	puts("[system]Init...2\n");
}

void systemReset(void){
	printf("[system]Reset!\n");
}
void systemResetToBootloader(void) {
	printf("[system]ResetToBootloader!\n");
}

// drivers/light_led.c
void ledInit(const statusLedConfig_t *statusLedConfig) {
	UNUSED(statusLedConfig);
	printf("[led]Init...\n");
}
void timerInit(void) {
	printf("[timer]Init...\n");
}
void timerStart(void) {
}
void motorDevInit(const motorDevConfig_t *motorConfig, uint16_t idlePulse, uint8_t motorCount) {
}
void servoDevInit(const servoDevConfig_t *servoConfig) {
}
void failureMode(failureMode_e mode) {
	printf("[failureMode]!!! %d\n", mode);
	while(1);
}


// Thanks ArduPilot
uint64_t micros64() {
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return 1.0e6*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) -
			(start_time.tv_sec +
			(start_time.tv_nsec*1.0e-9)));
}
uint64_t millis64() {
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return 1.0e3*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) -
			(start_time.tv_sec +
			(start_time.tv_nsec*1.0e-9)));
}

uint32_t micros(void) {
	return micros64() & 0xFFFFFFFF;
}

uint32_t millis(void) {
	return millis64() & 0xFFFFFFFF;
}

void microsleep(uint32_t usec) {
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = usec*1000UL;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) ;
}
void delayMicroseconds(uint32_t us) {
	microsleep(us);
}
void delay(uint32_t ms) {
	uint64_t start = millis64();

	while ((millis64() - start) < ms) {
		microsleep(1000);
	}
}


bool pwmMotorsEnabled = true;
static pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];
pwmOutputPort_t *pwmGetMotors(void) {
	return motors;
}
bool pwmAreMotorsEnabled(void) {
	return pwmMotorsEnabled;
}
void pwmWriteMotor(uint8_t index, uint16_t value) {

}
void pwmShutdownPulsesForAllMotors(uint8_t motorCount) {
}
void pwmCompleteMotorUpdate(uint8_t motorCount) {
}
void pwmWriteServo(uint8_t index, uint16_t value) {
}

uint16_t adcGetChannel(uint8_t channel) {
	UNUSED(channel);
	return 0;
}

char _estack;
char _Min_Stack_Size;

// fake EEPROM
extern uint8_t __config_start;
extern uint32_t __FLASH_CONFIG_Size;
//static uint8_t *eeprom;
static FILE *eepromFd = NULL;
const char *EEPROM_FILE = "eeprom.bin";

void FLASH_Unlock(void) {
	uint8_t * const eeprom = &__config_start;

	// open or create
	eepromFd = fopen(EEPROM_FILE,"r+");
	if(eepromFd != NULL) {
		long lSize;
		int c;

		// obtain file size:
		fseek(eepromFd , 0 , SEEK_END);
		lSize = ftell(eepromFd);
		rewind(eepromFd);

		printf("[FLASH_Unlock]size = %d\n", lSize);
		for(int i=0; i<(int)&__FLASH_CONFIG_Size; i++){
			c = fgetc(eepromFd);
			if(c == EOF) break;
			eeprom[i] = c;
		}
		//fread(eeprom, (size_t)1, (size_t)lSize, eepromFd);
	}else{
		eepromFd = fopen(EEPROM_FILE, "w+");
		ftruncate(fileno(eepromFd), &__FLASH_CONFIG_Size);
	}
}
void FLASH_Lock(void) {
	// flush & close
	if(eepromFd != NULL) {
		const uint8_t *eeprom = &__config_start;
		fseek(eepromFd, 0, SEEK_SET);
		fwrite(eeprom, sizeof(uint8_t), (size_t)&__FLASH_CONFIG_Size, eepromFd);
		fclose(eepromFd);
		eepromFd = NULL;
	}
}

FLASH_Status FLASH_EraseSector(uint32_t FLASH_Sector, uint8_t VoltageRange) {
	UNUSED(FLASH_Sector);
	UNUSED(VoltageRange);
	return FLASH_COMPLETE;
}
FLASH_Status FLASH_ErasePage(uint32_t Page_Address) {
//	UNUSED(Page_Address);
	printf("[FLASH_ErasePage]%x\n", Page_Address);
	return FLASH_COMPLETE;
}
FLASH_Status FLASH_ProgramWord(uint32_t addr, uint32_t Data) {
//	const uint32_t *fakeEEPROM = &__config_start;
//	printf("[FLASH_ProgramWord]%p,%x:%x = %x\n", addr, (addr - (uint32_t)fakeEEPROM), addr, Data);
	*((uint32_t*)addr) = Data;
//	printf("[FLASH_ProgramWord]%p = %x\n", addr, *((uint32_t*)addr));
	return FLASH_COMPLETE;
}


