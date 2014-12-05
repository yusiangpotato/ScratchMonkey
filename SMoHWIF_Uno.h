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
    		SPI.setClockDivider(SPI_CLOCK_DIV2);// 8MHz - Pedal to the metal
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
	namespace ISP{
		enum {
    		ISP_RESET       = SS,
    		MCU_CLOCK       = 9,    // OC1A    
		};
		static bool usingHwSPI = false;

		inline void		init(){
			pinMode(MISO,INPUT);
			pinMode(MOSI,OUTPUT);
			pinMode(SCK,OUTPUT);
			pinMode(ISP_RESET,OUTPUT);

    		//
    		// Set up clock generator on OC1A
    		//
			pinMode(MCU_CLOCK, OUTPUT);
   			TCCR1A = _BV(COM1A0);              // CTC mode, toggle OC1A on comparison with OCR1A
    		OCR1A  = 0;                        // F(OC1A) = 16MHz / (2*8*(1+0) == 1MHz
    		TIMSK1 = 0;
    		TCCR1B = _BV(WGM12) | _BV(CS11);   // Prescale by 8
    		TCNT1  = 0;

    		//
    		// Set up SPI
    		//
			digitalWrite(MISO,      LOW);
			pinMode(MISO,           INPUT);
			pinMode(ISP_RESET,      OUTPUT);

		}
		inline void		cleanup		(){
			TCCR1B = 0;//Stop Clock Gen
			if(usingHwSPI) SPI.end();

		}
		inline uint8_t  transferSPI (uint8_t data, uint8_t speed){
			//Based on speed given, decide to use HW SPI or bitbang.
			//Speed is start from 0=fastest recommended
			//Speed=0-3 is HW SPI - 1MHz,500KHz,250KHz,125KHz. Corresponding to dividers.
			//Speed=4-10 is SW SPI - ~62KHz, ~31K, ~16K, ~8K, ~4K,    ~2K, ~1KHz. Then we give up.
			//				Period  	16us 32us  64us  128us 256us  512us  1ms
			//	So quarter-period = 1<<(speed-2) microseconds.
			static const int clockDividers[4] = {
				SPI_CLOCK_DIV16, //0 - 16MHz/16 = 1MHz
				SPI_CLOCK_DIV32, //1
				SPI_CLOCK_DIV64, //2
				SPI_CLOCK_DIV128 //3 - 16MHz/128 = 125kHz
			};
			if(speed<4){
				if(!usingHwSPI){
					pinMode(SS,OUTPUT);
					SPI.begin();
					SPI.setDataMode(SPI_MODE0);
					SPI.setBitOrder(MSBFIRST);
					usingHwSPI=true;
				}
				SPI.setClockDivider(clockDividers[speed]);
				return SPI.transfer(data);
			}else{
				//SW SPI.
				if(usingHwSPI){
					SPI.end();
					pinMode(MISO,INPUT);
					pinMode(MOSI,OUTPUT);
					pinMode(SCK,OUTPUT);
					usingHwSPI=false;
				}
				uint16_t kQuarterCycle = 1<<(speed-2);
				uint8_t in = 0;
    			for (int i=0; i<8; ++i) {
        		digitalWrite(MOSI, (data & 0x80) != 0);
        		data <<= 1;
        		delayMicroseconds(kQuarterCycle);
        		digitalWrite(SCK, HIGH);
        		delayMicroseconds(kQuarterCycle);
        		in = (in << 1) | digitalRead(MISO);
        		delayMicroseconds(kQuarterCycle);
        		digitalWrite(SCK, LOW);
        		delayMicroseconds(kQuarterCycle);        
    			}
    			return in;
			}
		}
			
		inline uint8_t  minSpeed(){return 10;}
			//Returns as slow as it can get. Larger is better, but make this too large and it will take forever.
		inline void		writeReset(uint8_t state){digitalWrite(ISP_RESET,state);}
	}
}
#endif //Include guard