/*
 * Max30102.c
 *
 *  Created on: Dec 24, 2020
 *      Author: Amul Shinde
 */
#include "Max30102.h"
#include "stm32f4xx_hal.h"
#include <string.h>

// Check device ID and Revision

void spO2setup(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange){
	setLedMode(MAX30102_MODE_REDIRONLY);
	setRedAmplitude(ledCurrent6_4mA);
	setIRAmplitude(ledCurrent6_4mA);
	setSampleAverage(sampleAverage);
	setSampleRate(sampleRate);
	setPulseWidth(pulseWidth);
	setADCrange(adcRange);
	// set FIFO
	regAddr[0] = MAX30102_FIFOWRITEPTR;
	regAddr[1] = 0x00;
	writeMCU(regAddr, 2);
	regAddr[0] = MAX30102_FIFOOVERFLOW;
	writeMCU(regAddr, 2);
	regAddr[0] = MAX30102_FIFOREADPTR;
	writeMCU(regAddr, 2);
}

void setLedMode(uint8_t mode){
	// set spO2 mode Mode Configuration: 0x09; Modes [B0:B2] 011 Two leds
	regAddr[0] = MAX30102_MODECONFIG;
	regAddr[1] = mode;
	writeMCU(regAddr, 2);
	if(mode == MAX30102_MODE_REDIRONLY){
		activeLEDs = 2;
	} else if(mode == MAX30102_MODE_REDONLY){
		activeLEDs = 1;
	} else {
		activeLEDs = 3; // if there are three LED's on MAX3010x Breakout board
	}

}

void setRedAmplitude(uint8_t powerLevel){
	regAddr[0] = MAX30102_LED1_PULSEAMP;
	regAddr[1] = powerLevel;
	writeMCU(regAddr, 2);
}

void setIRAmplitude(uint8_t powerLevel){
	regAddr[0] = MAX30102_LED2_PULSEAMP;
	regAddr[1] = powerLevel;
	writeMCU(regAddr, 2);
}

void setSampleAverage(uint8_t sampleAverage){
	regAddr[0] = MAX30102_FIFOCONFIG;
	regAddr[1] = sampleAverage;
	writeMCU(regAddr, 2);
}

void setSampleRate(uint8_t sampleRate){
	regAddr[0] = MAX30102_PARTICLECONFIG;
	regAddr[1] = sampleRate;
	writeMCU(regAddr, 2);
}

void setPulseWidth(uint8_t pulseWidth){
	regAddr[0] = MAX30102_PARTICLECONFIG;
	regAddr[1] = pulseWidth;
	writeMCU(regAddr, 2);

}

void setADCrange(uint8_t adcRange){
	regAddr[0] = MAX30102_PARTICLECONFIG;
	regAddr[1] = adcRange;
	writeMCU(regAddr, 2);
}

bool safeCheck(uint8_t maxTimeToCheck){
	uint32_t markTime = HAL_GetTick();  // tick value in milliseconds

	while(1)
	{
		if(HAL_GetTick() - markTime > maxTimeToCheck) return(false);
		if(check() == true) // data found
			return true;
		HAL_Delay(1);
	}
}


// FIFO Configuration

// Set sample average
void setFIFOAverage(uint8_t numberOFsamples){
	bitMask(MAX30102_FIFOCONFIG, MAX30102_SAMPLEAVG_MASK, numberOFsamples);
}

//Resets all points to start in a known state
//Page 14 recommends clearing FIFO before beginning a read
void clearFIFO(void){
	regAddr[0] = MAX30102_FIFOWRITEPTR;
	regAddr[1] = 0x00;
	writeMCU(regAddr, 1);

	regAddr[0] = MAX30102_FIFOOVERFLOW;
	writeMCU(regAddr, 1);

	regAddr[0] = MAX30102_FIFOREADPTR;
	writeMCU(regAddr, 1);
}

// Enable roll over if FIFO overflows
void enableFIFORollover(void){
	bitMask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_ENABLE);
}

// Disable roll-over if FIFO overflows
void disableFIFORollover(void){
	bitMask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_DISABLE);
}

//Set number of samples to trigger the almost full interrupt (Page 17)
//Power on default is 32 samples
//Note it is reverse: 0x00 is 32 samples, 0x0F is 17 samples
void setFIFOAlmostFull(uint8_t numberOfSamples){
	bitMask(MAX30102_FIFOCONFIG, MAX30102_A_FULL_MASK, numberOfSamples);
}

uint8_t getWritePointer(void){
	regAddr[0] = MAX30102_FIFOWRITEPTR ;
	regAddr[1] = 0x00;
	return readMCU(regAddr, 1);
}

uint8_t getReadPointer(void){
	regAddr[0] = MAX30102_FIFOREADPTR ;
	regAddr[1] = 0x00;
	return readMCU(regAddr, 1);
}

void softReset(void){
	bitMask(MAX30102_MODECONFIG, MAX30102_RESET_MASK, MAX30102_RESET);
	// Poll for bit to clear, reset is then complete
	// Timeout after 100ms
	unsigned long startTime = HAL_GetTick();  // tick value in milliseconds
	while (HAL_GetTick() - startTime < 100)
	{
		regAddr[0] = MAX30102_MODECONFIG;
		uint8_t response = readMCU(regAddr, 1);
		if ((response & MAX30102_RESET) == 0) break; //we are done!
		HAL_Delay(1); //Let's not over burden the I2C bus
	 }
}

void shutdDown(void){
	// Put IC into low power mode (datasheet pg. 18)
	// During shutdown the IC will continue to respond to I2C commands but will
	// not update with or take new readings (such as temperature)
	bitMask(MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_SHUTDOWN);
}

void wakeUp(void){
	// pull IC out of low power mode (datasheet pg. 18)
	bitMask(MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_WAKEUP);
}

//Polls the sensor for new data
//Call regularly
//If new data is available, it updates the head and tail in the main struct
//Returns number of new samples obtained

uint16_t check(void){
	return 0;
}

// Read and write data

uint8_t writeMCU(uint8_t regAddr[2], uint8_t size){
	HAL_I2C_Master_Transmit(&hi2c1, Max30102_Address, regAddr, size, 10);
	return 0;
}

uint8_t readMCU(uint8_t regAddr[2], uint8_t size){
	HAL_I2C_Master_Transmit(&hi2c1, Max30102_Address, regAddr, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, Max30102_Address, buf, strlen((char*) buf), 10);
	return buf[0];
}

void bitMask(uint8_t reg, uint8_t mask, uint8_t thing){
	regAddr[0] = reg;

	originalContents = readMCU(regAddr, 1);
	originalContents = originalContents & mask;

	regAddr[1] = (originalContents | thing);

	writeMCU(regAddr, 2);

}

uint8_t getData(void){
	regAddr[0] = MAX30102_FIFODATA;
	regAddr[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, Max30102_Address, regAddr, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, Max30102_Address, pBuffer, 6, 100);

	//RedAmp = pBuffer[0] << 16 | pBuffer[1] << 8 | pBuffer[2];
	/*IRAmp  = pBuffer[3] << 16 | pBuffer[4] << 8 | pBuffer[5];

	RedAmp &= 0x03FFFF;
	IRAmp &= 0x03FFFF;*/

	RedAmp0 = pBuffer[0];
	RedAmp1 = pBuffer[1];
	RedAmp2 = pBuffer[2];
	RedAmp  = RedAmp0 << 16 | RedAmp1 <<8 | RedAmp2;
	RedAmp &= 0x03FFFF;

	IRAmp0	= pBuffer[3];
	IRAmp1	= pBuffer[4];
	IRAmp2	= pBuffer[5];
	IRAmp	= IRAmp0 << 16| IRAmp1 << 8 | IRAmp2 ;
	IRAmp  &= 0x03FFFF;

	return 0;
}

uint8_t getDataBurst(void){
	// First transaction: Get the FIFO_WR_PTR:
	FIFOWRTPointer = getWritePointer();

	// First transaction: Get the FIFO_WR_PTR:
	FIFORDPointer = getReadPointer();

	// The central processor evaluates the number of samples to be read from the FIFO:
	Num_Available_Samples = FIFOWRTPointer - FIFORDPointer;

	if(Num_Available_Samples > 0){
		Num_Samples_To_Read = Num_Available_Samples;
	} else {
		Num_Samples_To_Read = 32;
	}

	// Read Num_SamplesTo_Read samples from the FIFO

	regAddr[0] = MAX30102_FIFODATA;
	regAddr[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, Max30102_Address, regAddr, 1, 100);

	for (int var = 0; var < Num_Samples_To_Read; ++var) {

	HAL_I2C_Master_Receive(&hi2c1, Max30102_Address, pBuffer, 6, 100);
	RedAmp0 = pBuffer[0];
	RedAmp1 = pBuffer[1];
	RedAmp2 = pBuffer[2];
	RedAmp  = RedAmp0 << 16 | RedAmp1 <<8 | RedAmp2;
	RedAmp &= 0x03FFFF;
	RedAmpArray[var] = RedAmp;

	IRAmp0	= pBuffer[3];
	IRAmp1	= pBuffer[4];
	IRAmp2	= pBuffer[5];
	IRAmp	= IRAmp0 << 16| IRAmp1 << 8 | IRAmp2 ;
	IRAmp  &= 0x03FFFF;
	IRAmpArray[var] = IRAmp;
	}

	return 0;
}

uint32_t getRedAmp(void){
	getData();
	HAL_Delay(250);
	return RedAmp;
}

uint32_t getIRAmp(void){
	getData();
	return IRAmp;
}

uint16_t getAnything(void){
	getData();
	return IRAmp;
}
