//SMoHWIF_Uno.h
//Hardware Interface for Arduino UNO


#ifndef _SMoHWIF_Uno_h_
#define _SMoHWIF_Uno_h_
#include <stdint.h>
#include <Arduino.h>
#include "SMoConfig.h"

namespace SMoHWIF{
	namespace HVSP{
		enum {
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

	}
}
#endif //Include guard