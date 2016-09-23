/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include <gpio.h>
#include <inttypes.h>

#include <stdio.h>

#define WINDPIN 0b100			    //pin 3 ?
#define RAINPIN 0b010			    //pin 2


									//saving port D for analog reads
uint16_t calcWind(uint16_t windVal);
uint16_t getWind(void);


uint16_t rawWind;
float rawRain = 0;
uint16_t degree;



ISR(PCINT2_vect){					//make vector routine that starts when PCINT10 changes: (switch opens and closes when bucket turns, so it's both high and low)
rawRain += .2794;					//increment the rain amount collected by 0.2794mm. 
}

uint16_t getWind(void){

rawWind = (PORTC&WINDPIN);			//mask out PORTD PIN 3
return calcWind(rawWind);

}

uint16_t calcWind(uint16_t windVal)
{
if ((0<windVal) && (windVal<.365)){			//double ampersand means if statement will skip second case if first one is false
	degree = 112.5;
}
if (.365<windVal && windVal<.43){
	degree = 67.5;
}
if (.43<windVal && windVal<.535){
	degree = 90;
}
if (.535<windVal && windVal<.76){
	degree = 157.5;
}
if (.76<windVal && windVal<1.045){
	degree = 135;
}
if (1.045<windVal && windVal<1.295){
	degree = 202.5;
}
if (1.295<windVal && windVal<1.69){
	degree = 180;
}
if (1.69<windVal && windVal<2.115){
	degree = 22.5;
}
if (2.115<windVal && windVal<2.59){
	degree = 45;
}
if (2.59<windVal && windVal<3.005){
	degree = 45;
}
if (3.005<windVal && windVal<3.255){
	degree = 45;
}
if (3.255<windVal && windVal<3.635){
	degree = 45;
}
if (3.635<windVal && windVal<3.94){
	degree = 45;
}
if (3.94<windVal && windVal<4.33){
	degree = 45;
}
if (4.33<windVal && windVal<4.7){
	degree = 270;
}
if (4.7<windVal && windVal<5){
	degree = 315;
}
//else
	//sprintf("invalid wind direction value: %" PRIu16 "\n", windVal);
	
	return degree;
}

int main(void){

	DDRC &= ~(0b1111);	//data direction masked, last 4 bits output
	ADCSRA |= (1<<7);	//enable adc with 1
	PCMSK1 |= (1<<2);	//PCINT2 is the vector used above. mask out PCMSK1 (pin change mask register) to pcint10 (port c pin 2) to get the rain bucket pin to start the ISR (PCINT2_vect).
	
	while(1){
	uint16_t windDir = getWind();	
	sprintf("wind dir: %u", windDir);
	}
}