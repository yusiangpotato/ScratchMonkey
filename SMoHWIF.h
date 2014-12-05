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
}

#if SMO_LAYOUT==SMO_LAYOUT_STANDARD
	#include "SMoHWIF_Uno.h"
#endif

#endif //Include guard 