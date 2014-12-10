#ifndef _SMoHWIF_Generic_h_ //Include guard
#define _SMoHWIF_Generic_h_
#include <stdint.h>
#include <Arduino.h>
#include "SMoConfig.h"

#ifdef 	_SMoHWIF_defined_
	#error "Pick one, and only one, SMoHWIF file..."
#endif
#define _SMoHWIF_defined_

//Dependencies
#include <SPI.h>

//


namespace SMoHWIF{
	namespace HVSP{
		enum { //Variables/constants e.g. pins go here
			HVSP_VCC   = -1,
			HVSP_RESET = -1,
			HVSP_SDI   = -1,
			HVSP_SII   = -1,
			HVSP_SDO   = -1,
			HVSP_SCI   = -1,
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

    	inline void 	cleanup(){
			;
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
    		HVPP_RESET  = -1,
    		HVPP_RDY    = -1,
    		HVPP_VCC    = -1,
    		HVPP_XTAL   = -1,
    	};
    	//Define pins
    	static const int dataPins[8]={-1,-1,-1,-1,-1,-1,-1,-1};
    	static const int controlPins[8]={-1,-1,-1,-1,-1,-1,-1,-1}; //Set bit 1 as -1.

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
			for(uint8_t i=7;i!=0xFF;i--){
				pinMode(controlPins[i],state?INPUT:OUTPUT);
			}
    	}

    	inline void 	cleanup(){
			;
    	}

		inline void		trisData	(bool state){//True=input, false=output
			for(uint8_t i=7;i!=0xFF;i--){
				pinMode(dataPins[i],state?INPUT:OUTPUT);
			}
		}
		inline void		writeData	(uint8_t data){
			for(uint8_t i=0;i<8;i++){
				digitalWrite(dataPins[i],(data>>i)&1);
			}
		}
		inline uint8_t  readData	(){
			uint8_t data=0;
			for(uint8_t i=0;i<8;i++){
				data<<=1;
				data|=digitalRead(dataPins[i]);
			}
			return data;
		}
		inline void 	writeControl(uint8_t ctl){
			for(uint8_t i=0;i<8;i++){
				digitalWrite(controlPins[i],(data>>i)&1);
			}
		}
		inline void	   	writeReset	(uint8_t state){digitalWrite(HVPP_RESET,state);}
		inline void    	writeVCC	(uint8_t state){digitalWrite(HVPP_VCC,state);}
		inline void    	writeXTAL	(uint8_t state){digitalWrite(HVPP_XTAL,state);}
		inline uint8_t	readRDY		(){return digitalRead(HVPP_RDY);}

	}
	namespace ISP{
		enum {
    		ISP_RESET       = -1,
    		MCU_CLOCK       = -1,    // OC1A    
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