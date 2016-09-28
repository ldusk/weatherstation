
#ifndef MPL_H_
#define MPL_H_


// Standard 7-bit I2C slave address is 1100000 = 0x60, 8-bit read address is 0xC1, 8-bit write is 0xC0
#define MPL3115A2_ADDRESS 0x60  // SA0 is high, 0x1C if low

#define STATUS     0x00
#define OUT_P_MSB  0x01
#define OUT_P_CSB  0x02
#define OUT_P_LSB  0x03
#define OUT_T_MSB  0x04
#define OUT_T_LSB  0x05
#define DR_STATUS  0x06
#define OUT_P_DELTA_MSB  0x07
#define OUT_P_DELTA_CSB  0x08
#define OUT_P_DELTA_LSB  0x09
#define OUT_T_DELTA_MSB  0x0A
#define OUT_T_DELTA_LSB  0x0B
#define WHO_AM_I   0x0C
#define F_STATUS   0x0D
#define F_DATA     0x0E
#define F_SETUP    0x0F
#define TIME_DLY   0x10
#define SYSMOD     0x11
#define INT_SOURCE 0x12
#define PT_DATA_CFG 0x13
#define BAR_IN_MSB 0x14 // Set at factory to equivalent sea level pressure for measurement location, generally no need to change
#define BAR_IN_LSB 0x15 // Set at factory to equivalent sea level pressure for measurement location, generally no need to change
#define P_TGT_MSB  0x16
#define P_TGT_LSB  0x17
#define T_TGT      0x18
#define P_WND_MSB  0x19
#define P_WND_LSB  0x1A
#define T_WND      0x1B
#define P_MIN_MSB  0x1C
#define P_MIN_CSB  0x1D
#define P_MIN_LSB  0x1E
#define T_MIN_MSB  0x1F
#define T_MIN_LSB  0x20
#define P_MAX_MSB  0x21
#define P_MAX_CSB  0x22
#define P_MAX_LSB  0x23
#define T_MAX_MSB  0x24
#define T_MAX_LSB  0x25
#define CTRL_REG1  0x26
#define CTRL_REG2  0x27
#define CTRL_REG3  0x28
#define CTRL_REG4  0x29
#define CTRL_REG5  0x2A
#define OFF_P      0x2B
#define OFF_T      0x2C
#define OFF_H      0x2D
#define byte uint8_t

void toggleOneShot(void);
void SampleRate(byte samplerate);
void initFIFOMPL3115A2(void);
void initRealTimeMPL3115A2(void);
void TimeStep(byte ST_Value);
void MPL3115A2enableEventflags(void);
void ActiveAltimeterMode(void);
void ActiveBarometerMode(void);
void MPL3115A2Reset(void);
void MPL3115A2Standby(void);
void MPL3115A2Active(void);
void readAltitude(void);
void readRegisters(byte address, int i, byte * dest);
void readPressure(void);
void runMPL(void);
void writeRegister(unsigned char address, unsigned char data);
uint8_t readRegister(uint8_t address);
uint8_t MPL3115A2_init(void);

float altitude = 0.;
float pressure = 0.;
float temperature = 0.;


#endif /* MPL_H_ */
