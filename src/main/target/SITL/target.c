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
#include <stdlib.h>
#include <math.h>
#define printf printf
#define sprintf sprintf

#include <errno.h>
#include <time.h>
#include <pthread.h>

#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/serial.h"
#include "drivers/serial_tcp.h"
#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/light_led.h"

#include "drivers/timer.h"
#include "drivers/timer_def.h"
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {};

#include "drivers/accgyro_fake.h"
#include "flight/imu.h"

#include "target/SITL/dyad/src/dyad.h"
#include "target/SITL/udplink.h"

static fdm_packet fdmPkt;
static servo_packet pwmPkt;

static struct timespec start_time;
static pthread_t tcpWorker, udpWorker;
static bool workerRunning = true;
static udpLink_t stateLink, pwmLink;

#define ACC_SCALE (512 / 9.81)
#define GYRO_SCALE ((180.0 / M_PI) * 8192 / 2000.0)
void updateState(const fdm_packet* pkt) {
	int16_t x,y,z;
	x = -pkt->imu_linear_acceleration_xyz[0] * ACC_SCALE;
	y = -pkt->imu_linear_acceleration_xyz[1] * ACC_SCALE;
	z = -pkt->imu_linear_acceleration_xyz[2] * ACC_SCALE;
	fakeAccSet(x, y, z);
//	printf("[acc]%lf,%lf,%lf\n", pkt->imu_linear_acceleration_xyz[0], pkt->imu_linear_acceleration_xyz[1], pkt->imu_linear_acceleration_xyz[2]);

	x = pkt->imu_angular_velocity_rpy[0] * GYRO_SCALE;
	y = -pkt->imu_angular_velocity_rpy[1] * GYRO_SCALE;
	z = -pkt->imu_angular_velocity_rpy[2] * GYRO_SCALE;
	fakeGyroSet(x, y, z);
//	printf("[gyr]%lf,%lf,%lf\n", pkt->imu_angular_velocity_rpy[0], pkt->imu_angular_velocity_rpy[1], pkt->imu_angular_velocity_rpy[2]);

//	q.w = pkt->imu_orientation_quat[0];
//	pos.x = pkt->position_xyz[0];
//	spd.x = pkt->velocity_xyz[0];
}

static void* udpThread(void* data) {
	UNUSED(data);
	int n = 0;

	while (workerRunning) {
		n = udpRecv(&stateLink, &fdmPkt, sizeof(fdm_packet), 100);
		if(n == sizeof(fdm_packet)) {
//			printf("[data]new fdm %d\n", n);
			updateState(&fdmPkt);
		}
	}

	printf("udpThread end!!\n");
	return NULL;
}

static void* tcpThread(void* data) {
	UNUSED(data);

	dyad_init();
	dyad_setTickInterval(0.2f);
	dyad_setUpdateTimeout(0.5f);

	while (workerRunning) {
		dyad_update();
	}

	dyad_shutdown();
	printf("tcpThread end!!\n");
	return NULL;
}

// system
void systemInit(void) {
	int ret;

	clock_gettime(CLOCK_MONOTONIC, &start_time);
	printf("[system]Init...\n");

	SystemCoreClock = 500 * 1e6;
	FLASH_Unlock();

	ret = pthread_create(&tcpWorker, NULL, tcpThread, NULL);
	if(ret != 0) {
		printf("Create tcpWorker error!\n");
		exit(1);
	}

	ret = udpInit(&pwmLink, "127.0.0.1", 9002, false);
	printf("init PwnOut UDP link...%d\n", ret);

	ret = udpInit(&stateLink, NULL, 9003, true);
	printf("start UDP server...%d\n", ret);

	ret = pthread_create(&udpWorker, NULL, udpThread, NULL);
	if(ret != 0) {
		printf("Create udpWorker error!\n");
		exit(1);
	}
}

void systemReset(void){
	printf("[system]Reset!\n");
	workerRunning = false;
	pthread_join(tcpWorker, NULL);
	pthread_join(udpWorker, NULL);
	exit(0);
}
void systemResetToBootloader(void) {
	printf("[system]ResetToBootloader!\n");
	workerRunning = false;
	pthread_join(tcpWorker, NULL);
	pthread_join(udpWorker, NULL);
	exit(0);
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
void failureMode(failureMode_e mode) {
	printf("[failureMode]!!! %d\n", mode);
	while(1);
}


// Time part
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


// PWM part
bool pwmMotorsEnabled = false;
static pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];
static pwmOutputPort_t servos[MAX_SUPPORTED_SERVOS];

// real value to send
static uint16_t motorsPwm[MAX_SUPPORTED_MOTORS];
static uint16_t servosPwm[MAX_SUPPORTED_SERVOS];
static uint16_t idlePulse;

void motorDevInit(const motorDevConfig_t *motorConfig, uint16_t _idlePulse, uint8_t motorCount) {
	UNUSED(motorConfig);
	UNUSED(motorCount);

	idlePulse = _idlePulse;

	for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
		motors[motorIndex].enabled = true;
	}
	pwmMotorsEnabled = true;
}
void servoDevInit(const servoDevConfig_t *servoConfig) {
	UNUSED(servoConfig);
	for (uint8_t servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
		servos[servoIndex].enabled = true;
	}
}

pwmOutputPort_t *pwmGetMotors(void) {
	return motors;
}
bool pwmAreMotorsEnabled(void) {
	return pwmMotorsEnabled;
}
void pwmWriteMotor(uint8_t index, uint16_t value) {
	motorsPwm[index] = value - idlePulse;
//	motorsPwm[index] = value;
}
void pwmShutdownPulsesForAllMotors(uint8_t motorCount) {
	UNUSED(motorCount);
	pwmMotorsEnabled = false;
}
void pwmCompleteMotorUpdate(uint8_t motorCount) {
	UNUSED(motorCount);
	// send to simulator
	// for gazebo8 ArduCopterPlugin remap, range = [0.0, 1.0]
	pwmPkt.motor_speed[3] = motorsPwm[0] / 1000.0f;
	pwmPkt.motor_speed[0] = motorsPwm[1] / 1000.0f;
	pwmPkt.motor_speed[1] = motorsPwm[2] / 1000.0f;
	pwmPkt.motor_speed[2] = motorsPwm[3] / 1000.0f;

//	printf("[pwm]%u:%u,%u,%u,%u\n", idlePulse, motorsPwm[0], motorsPwm[1], motorsPwm[2], motorsPwm[3]);
	udpSend(&pwmLink, &pwmPkt, sizeof(servo_packet));
}
void pwmWriteServo(uint8_t index, uint16_t value) {
	servosPwm[index] = value;
}

// ADC part
uint16_t adcGetChannel(uint8_t channel) {
	UNUSED(channel);
	return 0;
}

// stack part
char _estack;
char _Min_Stack_Size;

// fake EEPROM
extern uint8_t __config_start;
extern uint32_t __FLASH_CONFIG_Size;
static FILE *eepromFd = NULL;
const char *EEPROM_FILE = "eeprom.bin";

void FLASH_Unlock(void) {
	uint8_t * const eeprom = &__config_start;

	if(eepromFd != NULL) {
		printf("[FLASH_Unlock] eepromFd != NULL\n");
		return;
	}

	// open or create
	eepromFd = fopen(EEPROM_FILE,"r+");
	if(eepromFd != NULL) {
		long lSize;
		int c;

		// obtain file size:
		fseek(eepromFd , 0 , SEEK_END);
		lSize = ftell(eepromFd);
		rewind(eepromFd);

		printf("[FLASH_Unlock]size = %ld\n", lSize);
		for(unsigned int i=0; i<((uintptr_t)&__FLASH_CONFIG_Size); i++){
			c = fgetc(eepromFd);
			if(c == EOF) break;
			eeprom[i] = (uint8_t)c;
		}
	}else{
		eepromFd = fopen(EEPROM_FILE, "w+");
		fwrite(eeprom, sizeof(uint8_t), (size_t)&__FLASH_CONFIG_Size, eepromFd);
	}
}
void FLASH_Lock(void) {
	// flush & close
	if(eepromFd != NULL) {
		const uint8_t *eeprom = &__config_start;
		fseek(eepromFd, 0, SEEK_SET);
		fwrite(eeprom, sizeof(uint8_t), (size_t)&__FLASH_CONFIG_Size, eepromFd);
		fflush(eepromFd);
		fclose(eepromFd);
		eepromFd = NULL;
	}
}

FLASH_Status FLASH_ErasePage(uint32_t Page_Address) {
	UNUSED(Page_Address);
//	printf("[FLASH_ErasePage]%x\n", Page_Address);
	return FLASH_COMPLETE;
}
FLASH_Status FLASH_ProgramWord(uint32_t addr, uint32_t Data) {
	*((uint32_t*)(uintptr_t)addr) = Data;
//	printf("[FLASH_ProgramWord]0x%x = %x\n", addr, *((uint32_t*)(uintptr_t)addr));
	return FLASH_COMPLETE;
}


