//Hardware Interface. 
//Modifying SMoHWIF_*.cpp should be able to get this working on any/all arduino variants.
//We need the functions below for proper operation.

#ifndef _SMoHWIF_h_
#define _SMoHWIF_h_
#include <stdint.h>

namespace SMoHWIF{
	namespace HVSP{
		inline void    initPins();
			//Set VCC to Output, Low;
			//Set RST to Output, High;
			//Set SCI,SDI,SII,SDO to Output, Low.
		inline void    writeVCC	 (uint8_t state);
		inline void    writeReset(uint8_t state);
		inline void    writeSCI  (uint8_t state);
		inline void    writeSDI  (uint8_t state);
		inline void    writeSII  (uint8_t state);
		inline uint8_t readSDO();
		inline void	trisSDO	  (bool state);
			//If state==0 set as output. Otherwise input.
	}
	namespace HVPP{
		inline void 	initPins();

		inline void		trisData	(bool state);
		inline void		writeData	(uint8_t data);
		inline uint8_t  readData	();
		inline void 	writeControl(uint8_t ctl);
		inline void	   	writeReset	(uint8_t state);
		inline void    	writeVCC	(uint8_t state);
		inline void    	writeXTAL	(uint8_t state);
		inline uint8_t	readRDY		();

	}
	namespace ISP{
		inline void 	initPins	();
		inline void		cleanup		();
		inline uint8_t  transferSPI (uint8_t data, int8_t speed);
			//Based on speed given, decide to use HW SPI or bitbang.
			//Speed is start from 0=fastest reccomended. Valid values for speed are 0-minSpeed() inclusive.
		inline uint8_t  minSpeed();
			//Returns as slow as it can get. Larger is better, but make this too large and it will take forever.
		inline void		writeReset(uint8_t state);
	}
}

#if SMO_LAYOUT==SMO_LAYOUT_STANDARD
	#include "SMoHWIF_Uno.h"
#else
	#error "Wait what halps too stronks i cannot >.<"
#endif

#endif //Include guard 