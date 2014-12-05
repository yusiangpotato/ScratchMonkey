//SMoHWIF_Uno.h
//Hardware Interface for Arduino UNO


#ifndef _SMoHWIF_Uno_h_ //Include guard
#define _SMoHWIF_Uno_h_
#include <stdint.h>
#include <Arduino.h>
#include "SMoConfig.h"



//Dependencies
#include <SPI.h>

//


namespace SMoHWIF{
	namespace HVSP{
		enum { //Variables/constants e.g. pins go here
    		HVSP_VCC   = SMO_SVCC,
    		HVSP_RESET = SMO_HVRESET,
    		HVSP_SDI   =  8,
    		HVSP_SII   =  9,
    		HVSP_SDO   = 12,
    		HVSP_SCI   = 13,
		};

		inline void    initPins(){
			pinMode(HVSP_VCC, OUTPUT);
			digitalWrite(HVSP_VCC, LOW);
    		digitalWrite(HVSP_RESET, HIGH); // Set BEFORE pinMode, so we don't glitch LOW
    		pinMode(HVSP_RESET, OUTPUT);
    		pinMode(HVSP_SCI, OUTPUT);
    		digitalWrite(HVSP_SCI, LOW);
    		pinMode(HVSP_SDI, OUTPUT);
    		digitalWrite(HVSP_SDI, LOW);
    		pinMode(HVSP_SII, OUTPUT);
    		digitalWrite(HVSP_SII, LOW);
    		pinMode(HVSP_SDO, OUTPUT);       // progEnable[2] on tinyX5
    		digitalWrite(HVSP_SDO, LOW);
    	}

		
		inline void    	writeReset	(uint8_t state){digitalWrite(HVSP_RESET,state);}
		inline void 	writeVCC	(uint8_t state){digitalWrite(HVSP_VCC,state);}
		inline void    	writeSCI  	(uint8_t state){digitalWrite(HVSP_SCI,state);}
		inline void    	writeSDI  	(uint8_t state){digitalWrite(HVSP_SDI,state);}
		inline void    	writeSII  	(uint8_t state){digitalWrite(HVSP_SII,state);}
		inline uint8_t 	readSDO		(){return digitalRead(HVSP_SDO);}
		inline void		trisSDO		(bool state){pinMode(HVSP_SDO,state?INPUT:OUTPUT);}
	}
	namespace HVPP{
		enum {
    		HVPP_RESET  = SMO_HVRESET,
    		HVPP_RDY    = 12,
    		HVPP_VCC    = SMO_SVCC,
    		HVPP_RCLK   = A1,
    		HVPP_XTAL   = A2,

    		PORTD_MASK = 0xFC,
    		PORTB_MASK = 0x03,
    		PORTD_SHIFT = 2,
    		PORTB_SHIFT = 6
		};

		inline void 	initPins(){ //Make CTRL pins output mode.
			pinMode(HVPP_VCC, OUTPUT);
    		digitalWrite(HVPP_VCC, LOW);
    		digitalWrite(HVPP_RESET, HIGH); // Set BEFORE pinMode, so we don't glitch LOW
    		pinMode(HVPP_RESET, OUTPUT);
    		pinMode(HVPP_RDY, INPUT);
    		digitalWrite(HVPP_RDY, LOW);
    		pinMode(HVPP_XTAL, OUTPUT);
    		digitalWrite(HVPP_XTAL, LOW);
    		trisData(false);
			//In this case we're using a 74'595 shift register for control, so set up SPI.
			SPI.begin();
    		SPI.setDataMode(SPI_MODE0);
    		SPI.setBitOrder(MSBFIRST);
    		SPI.setClockDivider(SPI_CLOCK_DIV2);// Pedal to the metal
    		digitalWrite(HVPP_RCLK, LOW);
    		pinMode(HVPP_RCLK, OUTPUT);
		}

		inline void		trisData	(bool state){//True=input, false=output
			if (!state) {
        		DDRD |= PORTD_MASK;
        		DDRB |= PORTB_MASK;
    		} else {
        		DDRD &= ~PORTD_MASK;
        		DDRB &= ~PORTB_MASK;
    		}
		}
		inline void		writeData	(uint8_t data){
			PORTD = (PORTD & ~PORTD_MASK) | ((data << PORTD_SHIFT) & PORTD_MASK);
    		PORTB = (PORTB & ~PORTB_MASK) | ((data >> PORTB_SHIFT) & PORTB_MASK);
		}
		inline uint8_t  readData	(){
			return (PINB << PORTB_SHIFT) | (PIND >> PORTD_SHIFT);
		}
		inline void 	writeControl(uint8_t ctl){
			digitalWrite(HVPP_RCLK, LOW);
    		SPI.transfer(ctl);
    		digitalWrite(HVPP_RCLK, HIGH);
		}
		inline void	   	writeReset	(uint8_t state){digitalWrite(HVPP_RESET,state);}
		inline void    	writeVCC	(uint8_t state){digitalWrite(HVPP_VCC,state);}
		inline void    	writeXTAL	(uint8_t state){digitalWrite(HVPP_XTAL,state);}
		inline uint8_t	readRDY		(){return digitalRead(HVPP_RDY);}

	}
}
#endif //Include guard