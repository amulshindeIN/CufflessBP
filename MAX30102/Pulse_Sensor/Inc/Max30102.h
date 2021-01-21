/*
 * Max30102.h
 *
 *  Created on: Dec 24, 2020
 *      Author: Amul Shinde
 */
#include "stm32f4xx_hal.h"
#include "stdbool.h"
//#include "stm32f4xx_hal_conf.h"

#ifndef INC_MAX30102_H_
#define INC_MAX30102_H_

#define Max30102_Address 				(0x57 << 1) // 7-bit I2C Address

// Status Registers
#define MAX30102_INTSTAT1 				0x00
#define MAX30102_INTSTAT2 				0x01
#define MAX30102_INTENABLE1 			0x02
#define MAX30102_INTENABLE2 			0x03

// FIFO Registers
#define MAX30102_FIFOWRITEPTR  			0x04
#define MAX30102_FIFOOVERFLOW  			0x05
#define MAX30102_FIFOREADPTR  			0x06
#define MAX30102_FIFODATA 				0x07

// Configuration Registers
#define MAX30102_FIFOCONFIG  			0x08
#define MAX30102_MODECONFIG  			0x09
#define MAX30102_PARTICLECONFIG 		0x0A    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
#define MAX30102_LED1_PULSEAMP  		0x0C    // Red LED ??
#define MAX30102_LED2_PULSEAMP  		0x0D	// IR LED ??
#define MAX30102_LED3_PULSEAMP  		0x0E	// Green LED ??
#define MAX30102_LED_PROX_AMP  			0x10
#define MAX30102_MULTILEDCONFIG1  		0x11
#define MAX30102_MULTILEDCONFIG2  		0x12

// Die Temperature Registers
#define MAX30102_DIETEMPINT  			0x1F
#define MAX30102_DIETEMPFRAC  			0x20
#define MAX30102_DIETEMPCONFIG  		0x21

// Proximity Function Registers
#define MAX30102_PROXINTTHRESH  		0x30

// Part ID Registers
#define MAX30102_REVISIONID  			0xFE
#define MAX30102_PARTID  				0xFF    // Should always be 0x15. Identical to MAX30102.

// MAX30102 Commands
// Interrupt configuration (pg 13, 14)
#define MAX30102_INT_A_FULL_MASK 		0x7F
#define MAX30102_INT_A_FULL_ENABLE  	0x80
#define MAX30102_INT_A_FULL_DISABLE 	0x00

#define MAX30102_INT_DATA_RDY_MASK  	0xBF
#define MAX30102_INT_DATA_RDY_ENABLE 	0x40
#define MAX30102_INT_DATA_RDY_DISABLE  	0x00

#define MAX30102_INT_ALC_OVF_MASK  		0xDF
#define MAX30102_INT_ALC_OVF_ENABLE 	0x20
#define MAX30102_INT_ALC_OVF_DISABLE  	0x00

#define MAX30102_INT_PROX_INT_MASK  	0xxEF
#define MAX30102_INT_PROX_INT_ENABLE  	0x10
#define MAX30102_INT_PROX_INT_DISABLE  	0x00

#define MAX30102_INT_DIE_TEMP_RDY_MASK  	0xF2
#define MAX30102_INT_DIE_TEMP_RDY_ENABLE  	0x02
#define MAX30102_INT_DIE_TEMP_RDY_DISABLE  	0x00

#define MAX30102_SAMPLEAVG_MASK 		0x1F
#define MAX30102_SAMPLEAVG_1  			0x00
#define MAX30102_SAMPLEAVG_2  			0x20
#define MAX30102_SAMPLEAVG_4  			0x40
#define MAX30102_SAMPLEAVG_8  			0x60
#define MAX30102_SAMPLEAVG_16  			0x80
#define MAX30102_SAMPLEAVG_32  			0xA0

#define MAX30102_ROLLOVER_MASK  		0xEF
#define MAX30102_ROLLOVER_ENABLE  		0x10
#define MAX30102_ROLLOVER_DISABLE  		0x00

#define MAX30102_A_FULL_MASK  			0xF0

// Mode configuration commands (page 18)
#define MAX30102_SHUTDOWN_MASK  		0x7F
#define MAX30102_SHUTDOWN  				0x80
#define MAX30102_WAKEUP  				0x00

#define MAX30102_RESET_MASK  			0xBF
#define MAX30102_RESET  				0x40

#define MAX30102_MODE_MASK  			0xF8
#define MAX30102_MODE_REDONLY  			0x02
#define MAX30102_MODE_REDIRONLY  		0x03
#define MAX30102_MODE_MULTILED  		0x07

// Particle sensing configuration commands (pgs 19-20)
#define MAX30102_ADCRANGE_MASK  		0x9F
#define MAX30102_ADCRANGE_2048  		0x00
#define MAX30102_ADCRANGE_4096  		0x20
#define MAX30102_ADCRANGE_8192  		0x40
#define MAX30102_ADCRANGE_16384 		0x60

#define MAX30102_SAMPLERATE_MASK  		0x1F
#define MAX30102_SAMPLERATE_50  		0x00
#define MAX30102_SAMPLERATE_100  		0x04
#define MAX30102_SAMPLERATE_200  		0x08
#define MAX30102_SAMPLERATE_400  		0x0C
#define MAX30102_SAMPLERATE_800  		0x10
#define MAX30102_SAMPLERATE_1000  		0x14
#define MAX30102_SAMPLERATE_1600  		0x18
#define MAX30102_SAMPLERATE_3200  		0x1C

#define MAX30102_PULSEWIDTH_MASK  		0xFC
#define MAX30102_PULSEWIDTH_69  		0x00
#define MAX30102_PULSEWIDTH_118  		0x01
#define MAX30102_PULSEWIDTH_215  		0x02
#define MAX30102_PULSEWIDTH_411  		0x03

//Multi-LED Mode configuration (pg 22)
#define MAX30102_SLOT1_MASK  			0xF8
#define MAX30102_SLOT2_MASK  			0x8F
#define MAX30102_SLOT3_MASK  			0xF8
#define MAX30102_SLOT4_MASK  			0x8F

#define SLOT_NONE  						0x00
#define SLOT_RED_LED  					0x01
#define SLOT_IR_LED  					0x02
#define SLOT_GREEN_LED  				0x03
#define SLOT_NONE_PILOT  				0x04
#define SLOT_RED_PILOT 					0x05
#define SLOT_IR_PILOT  					0x06
#define SLOT_GREEN_PILOT  				0x07

#define MAX_30102_EXPECTEDPARTID  		0x15

// Pulse Amplitude
#define ledCurrent00mA		0x00
#define ledCurrent0_2mA		0x01
#define ledCurrent0_4mA		0x02
#define ledCurrent3_1mA		0x0F
#define ledCurrent5_0mA		0x1A
#define ledCurrent6_4mA		0x1F
#define ledCurrent10_0mA	0x33
#define ledCurrent12_5mA	0x3F
#define ledCurrent15_0mA	0x4C
#define ledCurrent17_4mA	0x58
#define ledCurrent20_0mA	0x66
#define ledCurrent22_4mA	0x72
#define ledCurrent25_0mA	0x7F
#define ledCurrent27_5mA	0x8C
#define ledCurrent30_0mA	0x99
#define ledCurrent32_5mA	0xA5
#define ledCurrent35_0mA	0xB2
#define ledCurrent37_5mA	0xBF
#define ledCurrent40_0mA	0xCC
#define ledCurrent42_5mA	0xD8
#define ledCurrent45_0mA	0xE1
#define ledCurrent47_5mA	0xF2
#define ledCurrent50p0mA	0xFF

// Some variables
#define I2C_BUFFER_LENGTH 	32
uint8_t regAddr[2];

uint32_t markTime;
I2C_HandleTypeDef hi2c1;
uint8_t originalContents;
uint8_t readPointer;
uint8_t writePointer;
uint8_t FIFOWRTPointer;
uint8_t FIFORDPointer;
uint8_t Num_Available_Samples;
uint8_t Num_Samples_To_Read;
uint8_t value;

uint8_t numberOfSamples;
uint8_t bytesLeftToRead;
uint8_t activeLEDs;
uint8_t toGet;

#define STORAGE_SIZE 		4 // Each long is 4 bytes so limit this to fit on your micro
typedef struct Record{
	uint32_t red[STORAGE_SIZE];
	uint32_t IR[STORAGE_SIZE];
	int8_t head;
	int8_t tail;

} sense_struct;
sense_struct sense;

uint8_t temp[4];
uint32_t tempLong;


//void setup(byte powerLevel = 0x1F, byte sampleAverage = 4, byte ledMode = 3, int sampleRate = 400, int pulseWidth = 411, int adcRange = 4096);
void spO2setup(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange);
void setLedMode(uint8_t mode);
void setRedAmplitude(uint8_t powerLevel);
void setIRAmplitude(uint8_t powerLevel);
void setSampleAverage(uint8_t sampleAverage);
void setSampleRate(uint8_t sampleRate);
void setPulseWidth(uint8_t pulseWidth);
void setADCrange(uint8_t adcRange);
bool safeCheck(uint8_t maxTimeToCheck);
void setFIFOAverage(uint8_t numberOFsamples);
void clearFIFO(void);
void enableFIFORollover(void);
void disableFIFORollover(void);
void setFIFOAlmostFull(uint8_t numberOfSamples);
uint8_t getWritePointer(void);
uint8_t getReadPointer(void);
void softReset(void);
void shutdDown(void);
void wakeUp(void);
uint16_t check(void);


void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);



uint8_t writeMCU(uint8_t regAddr[2], uint8_t size);
uint8_t readMCU(uint8_t regAddr[20], uint8_t size);
uint8_t getData(void);
uint8_t getDataBurst(void);
uint8_t buf[40];
uint8_t pBuffer[32];
uint32_t getRedAmp(void);
uint32_t getIRAmp(void);
uint16_t getAnything(void);

uint16_t RedAmp;
uint8_t RedAmp0;
uint8_t RedAmp1;
uint8_t RedAmp2;

uint16_t IRAmp;
uint8_t IRAmp0;
uint8_t IRAmp1;
uint8_t IRAmp2;


// Data Burst Arrays
uint16_t RedAmpArray[32];
uint16_t IRAmpArray[32];

#endif /* INC_MAX30102_H_ */
