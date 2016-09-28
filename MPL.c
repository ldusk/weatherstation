#include <asf.h>
#include "i2c.h"
#include <MPL.h>


/*
=====================================================================================================
Define functions according to
"Data Manipulation and Basic Settings of the MPL3115A2 Command Line Interface Drive Code"
by Miguel Salhuana
Freescale Semiconductor Application Note AN4519 Rev 0.1, 08/2012
=====================================================================================================
*/
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Clears then sets OST bit which causes the sensor to immediately take another reading
void toggleOneShot(void)
{
	MPL3115A2Active();  // Set to active to start reading
	byte c = readRegister(CTRL_REG1);
	writeRegister(CTRL_REG1, c & ~(1<<1)); // Clear OST (bit 1)
	c = readRegister(CTRL_REG1);
	writeRegister(CTRL_REG1, c | (1<<1)); // Set OST bit to 1
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Set the Outputting Sample Rate
void SampleRate(byte samplerate)
{
	MPL3115A2Standby();  // Must be in standby to change registers

	byte c = readRegister(CTRL_REG1);
	writeRegister(CTRL_REG1, c & ~(0x38)); // Clear OSR bits 3,4,5
	if(samplerate < 8) { // OSR between  and 7
		writeRegister(CTRL_REG1, c | (samplerate << 3));  // Write OSR to bits 3,4,5
	}
	
	MPL3115A2Active();  // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Initialize the MPL3115A2 registers for FIFO mode
void initFIFOMPL3115A2(void)
{
	// Clear all interrupts by reading the data output registers
	byte temp;
	temp = readRegister(OUT_P_MSB);
	temp = readRegister(OUT_P_CSB);
	temp = readRegister(OUT_P_LSB);
	temp = readRegister(OUT_T_MSB);
	temp = readRegister(OUT_T_LSB);
	temp = readRegister(F_STATUS);
	
	MPL3115A2Standby();  // Must be in standby to change registers
	
	// Set CTRL_REG4 register to configure interrupt enable
	// Enable data ready interrupt (bit 7), enable FIFO (bit 6), enable pressure window (bit 5), temperature window (bit 4),
	// pressure threshold (bit 3), temperature threshold (bit 2), pressure change (bit 1) and temperature change (bit 0)
	writeRegister(CTRL_REG4, 0x40);  // enable FIFO
	
	//  Configure INT 1 for data ready, all other (inc. FIFO) interrupts to INT2
	writeRegister(CTRL_REG5, 0x80);
	
	// Set CTRL_REG3 register to configure interrupt signal type
	// Active HIGH, push-pull interrupts INT1 and INT 2
	writeRegister(CTRL_REG3, 0x22);
	
	// Set FIFO mode
	writeRegister(F_SETUP, 0x00); // Clear FIFO mode
	// In overflow mode, when FIFO fills up, no more data is taken until the FIFO registers are read
	// In watermark mode, the oldest data is overwritten by new data until the FIFO registers are read
	writeRegister(F_SETUP, 0x80); // Set F_MODE to interrupt when overflow = 32 reached
	//  writeRegister(F_SETUP, 0x60); // Set F_MODE to accept 32 data samples and interrupt when watermark = 32 reached

	MPL3115A2Active();  // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Initialize the MPL3115A2 for realtime data collection (unused)
void initRealTimeMPL3115A2(void)
{
	// Clear all interrupts by reading the data output registers
	byte temp;
	temp = readRegister(OUT_P_MSB);
	temp = readRegister(OUT_P_CSB);
	temp = readRegister(OUT_P_LSB);
	temp = readRegister(OUT_T_MSB);
	temp = readRegister(OUT_T_LSB);
	temp = readRegister(F_STATUS);
	
	MPL3115A2Standby();  // Must be in standby to change registers
	
	// Set CTRL_REG4 register to configure interupt enable
	// Enable data ready interrupt (bit 7), enable FIFO (bit 6), enable pressure window (bit 5), temperature window (bit 4),
	// pressure threshold (bit 3), temperature threshold (bit 2), pressure change (bit 1) and temperature change (bit 0)
	writeRegister(CTRL_REG4, 0x80);
	
	//  Configure INT 1 for data ready, all other interrupts to INT2
	writeRegister(CTRL_REG5, 0x80);
	
	// Set CTRL_REG3 register to configure interupt signal type
	// Active HIGH, push-pull interupts INT1 and INT 2
	writeRegister(CTRL_REG3, 0x22);
	
	// Set FIFO mode
	writeRegister(F_SETUP, 0x00); // disable FIFO mode
	
	MPL3115A2Active();  // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Set the Auto Acquisition Time Step (unused)
void TimeStep(byte ST_Value)
{
	MPL3115A2Standby(); // First put device in standby mode to allow write to registers
	
	byte c = readRegister(CTRL_REG2); // Read contents of register CTRL_REG2
	if (ST_Value <= 0xF) {
		writeRegister(CTRL_REG2, (c | ST_Value)); // Set time step n from 0x0 to 0xF (bits 0 - 3) for time intervals from 1 to 32768 (2^n) seconds
	}
	
	MPL3115A2Active(); // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enable the pressure and temperature event flags
// Bit 2 is general data ready event mode on new Pressure/Altitude or temperature data
// Bit 1 is event flag on new Pressure/Altitude data
// Bit 0 is event flag on new Temperature data
void MPL3115A2enableEventflags(void)
{
	MPL3115A2Standby();  // Must be in standby to change registers
	writeRegister(PT_DATA_CFG, 0x07); //Enable all three pressure and temperature event flags
	MPL3115A2Active();  // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enter Active Altimeter mode
void ActiveAltimeterMode(void)
{
	MPL3115A2Standby(); // First put device in standby mode to allow write to registers
	byte c = readRegister(CTRL_REG1); // Read contents of register CTRL_REG1
	writeRegister(CTRL_REG1, c | (0x80)); // Set ALT (bit 7) to 1
	MPL3115A2Active(); // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enter Active Barometer mode
void ActiveBarometerMode(void)
{
	MPL3115A2Standby(); // First put device in standby mode to allow write to registers
	byte c = readRegister(CTRL_REG1); // Read contents of register CTRL_REG1
	writeRegister(CTRL_REG1, c & ~(0x80)); // Set ALT (bit 7) to 0
	MPL3115A2Active(); // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Software resets the MPL3115A2.
// It must be in standby to change most register settings
void MPL3115A2Reset(void)
{
	writeRegister(CTRL_REG1, (0x04)); // Set RST (bit 2) to 1
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sets the MPL3115A2 to standby mode.
// It must be in standby to change most register settings
void MPL3115A2Standby(void)
{
	byte c = readRegister(CTRL_REG1); // Read contents of register CTRL_REG1
	writeRegister(CTRL_REG1, c & ~(0x01)); // Set SBYB (bit 0) to 0
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sets the MPL3115A2 to active mode.
// Needs to be in this mode to output data
void MPL3115A2Active(void)
{
	byte c = readRegister(CTRL_REG1); // Read contents of register CTRL_REG1
	writeRegister(CTRL_REG1, c | 0x01); // Set SBYB (bit 0) to 1
}

//=====================================================================================================//

/*
=====================================================================================================
i2c functions
=====================================================================================================
*/
// Writes a single byte (data) into address
void writeRegister(unsigned char address, unsigned char data)
{
	i2cSendStart();
	i2cWaitForComplete();

	i2cSendByte((MPL3115A2_ADDRESS<<1)); // Write 0xB4
	i2cWaitForComplete();

	i2cSendByte(address);	// Write register address
	i2cWaitForComplete();

	i2cSendByte(data);
	i2cWaitForComplete();

	i2cSendStop();
}



uint8_t readRegister(uint8_t address)
{
	uint8_t data;

	i2cSendStart();
	i2cWaitForComplete();

	i2cSendByte((MPL3115A2_ADDRESS<<1)); // Write 0xB4
	i2cWaitForComplete();

	i2cSendByte(address);	// Write register address
	i2cWaitForComplete();

	i2cSendStart();

	i2cSendByte((MPL3115A2_ADDRESS<<1)|0x01); // Write 0xB5
	i2cWaitForComplete();
	i2cReceiveByte(TRUE);								//this probably won't work
	i2cWaitForComplete();

	data = i2cGetReceivedByte();	// Get MSB result
	i2cWaitForComplete();
	i2cSendStop();

	cbi(TWCR, TWEN);	// Disable TWI
	sbi(TWCR, TWEN);	// Enable TWI

	return data;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Read i registers sequentially, starting at address into the dest byte array
void readRegisters(byte address, int i, byte * dest)
{
	i2cSendStart();
	i2cWaitForComplete();

	i2cSendByte((MPL3115A2_ADDRESS<<1)); // write 0xB4
	i2cWaitForComplete();

	i2cSendByte(address);	// write register address
	i2cWaitForComplete();

	i2cSendStart();
	i2cSendByte((MPL3115A2_ADDRESS<<1)|0x01); // write 0xB5
	i2cWaitForComplete();
	for (int j=0; j<i; j++)
	{
		i2cReceiveByte(TRUE);
		i2cWaitForComplete();
		dest[j] = i2cGetReceivedByte(); // Get MSB result
	}
	i2cWaitForComplete();
	i2cSendStop();

	cbi(TWCR, TWEN); // Disable TWI
	sbi(TWCR, TWEN); // Enable TWI
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
=====================================================================================================
MPL data functions
=====================================================================================================
*/
void readAltitude(void) // Get altitude in meters and temperature in centigrade
{

	byte rawData[5];  // msb/csb/lsb pressure and msb/lsb temperature stored in five contiguous registers


	// use a polling method
	// Check data read status; if PTDR (bit 4) not set, then
	
	// toggle OST bit to cause sensor to immediately take a reading
	// Setting the one shot toggle is the way to get faster than 1 Hz data read rates
	while ((readRegister(STATUS) & 0x08) == 0);  
	toggleOneShot();
	
	readRegisters(OUT_P_MSB, 5, &rawData[0]);  // Read the five raw data registers into data array

	// Altitude bytes-whole altitude contained defined by msb, csb, and first two bits of lsb, fraction by next two bits of lsb
	byte msbA = rawData[0];
	byte csbA = rawData[1];
	byte lsbA = rawData[2];
	// Temperature bytes
	byte msbT = rawData[3];
	byte lsbT = rawData[4];
	
	// Calculate altitude, check for negative sign in altimeter data
	long foo = 0;
	if(msbA > 0x7F) {
		foo = ~((long)msbA << 16 | (long)csbA << 8 | (long)lsbA) + 1; // 2's complement the data
		altitude = (float) (foo >> 8) + (float) ((lsbA >> 4)/16.0); // Whole number plus fraction altitude in meters for negative altitude
		altitude *= -1.;
	}
	else {
		altitude = (float) ( (msbA << 8) | csbA) + (float) ((lsbA >> 4)/16.0);  // Whole number plus fraction altitude in meters
	}

	// Calculate temperature, check for negative sign
	if(msbT > 0x7F) {
		foo = ~(msbT << 8 | lsbT) + 1 ; // 2's complement
		temperature = (float) (foo >> 8) + (float)((lsbT >> 4)/16.0); // add whole and fractional degrees Centigrade
		temperature *= -1.;
	}
	else {
		temperature = (float) (msbT) + (float)((lsbT >> 4)/16.0); // add whole and fractional degrees Centigrade
	}
}



void readPressure(void)
{
  byte rawData[5];  // msb/csb/lsb pressure and msb/lsb temperature stored in five contiguous registers
 
//polling:
// Check data read status; if PTDR (bit 4) not set, then
// toggle OST bit to cause sensor to immediately take a reading
// Setting the one shot toggle is the way to get faster than 1 Hz data read rates
while ((readRegister(STATUS) & 0x08) == 0);  toggleOneShot(); 
 
  readRegisters(OUT_P_MSB, 5, &rawData[0]);  // Read the five raw data registers into data array

// Pressure bytes
  byte msbP = rawData[0];
  byte csbP = rawData[1];
  byte lsbP = rawData[2];
// Temperature bytes
  byte msbT = rawData[3];
  byte lsbT = rawData[4]; 
 
  long pressure_whole =   ((long)msbP << 16 |  (long)csbP << 8 |  (long)lsbP) ; // Construct whole number pressure
  pressure_whole >>= 6; // Only two most significant bits of lsbP contribute to whole pressure; its an 18-bit number
 
  lsbP &= 0x30; // Keep only bits 5 and 6, the fractional pressure
  lsbP >>= 4; // Shift to get the fractional pressure in terms of quarters of a Pascal
  float pressure_frac = (float) lsbP/4.0; // Convert numbers of fractional quarters to fractional pressure n Pasacl

  pressure = (float) (pressure_whole) + pressure_frac; // Combine whole and fractional parts to get entire pressure in Pascal

// Calculate temperature, check for negative sign
long foo = 0;
if(msbT > 0x7F) { // Is the most significant bit a 1? Then its a negative number in two's complement form
 foo = ~(msbT << 8 | lsbT) + 1 ; // 2's complement
 temperature = (float) (foo >> 8) + (float)((lsbT >> 4)/16.0); // add whole and fractional degrees Centigrade
 temperature *= -1.;
 }
 else {
   temperature = (float) (msbT) + (float)((lsbT >> 4)/16.0); // add whole and fractional degrees Centigrade
 }
}

uint8_t MPL3115A2_init(void){
	
	DDRC = 0xFF;		//mask this later
	PORTC = 0xFF;		//pullup resistors

	ActiveAltimeterMode();
	SampleRate(7); //"OSR"
	MPL3115A2enableEventflags();

	uint8_t connect =	readRegister(WHO_AM_I); //proper reading should be 0xC4
}


void runMPL(void){
		ActiveAltimeterMode();
		readAltitude();
		ActiveBarometerMode();
		readPressure();
}
/*
=====================================================================================================
end of program

recommended use:

board_init();
i2cInit();

if (MPL3115A2_init()==0xC4)			//execute mpl init code, correct "who am i" connection should return 0xc4
{	//connection successful
}

while(1){
	runMPL();
}

global variables 'temperature', 'altitude' and 'pressure' are updated in runMPL().
=====================================================================================================
*/
int main (void)
{
	//system clock scaling?
	
	i2cInit();
	
	if (MPL3115A2_init()==0xC4)			//execute mpl init code
	{	//
	}
	
	
	board_init();

	while(1){
								//todo: make output a comma separated string
	
	runMPL();





	}

}
