//Hardware Interface.
//Modifying SMoHWIF_*.cpp should be able to get this working on any/all arduino variants.
//We need the functions below for proper operation.

#ifndef _SMoHWIF_h_
#define _SMoHWIF_h_
#include <stdint.h>

namespace SMoHWIF {
  inline void releasePins();
  namespace HVSP {
    inline void    initPins();
    inline void	   cleanup();
    inline void    writeVCC	 (uint8_t state);
    inline void    writeReset(uint8_t state);
    inline void    writeSCI  (uint8_t state);
    inline void    writeSDI  (uint8_t state);
    inline void    writeSII  (uint8_t state);
    inline uint8_t readSDO();
    inline void	trisSDO	  (bool state);
    //If state==0 set as output. Otherwise input.
  }
  namespace HVPP {
    inline void 	initPins();
    inline void 	cleanup();
    inline void		trisData	(bool state);
    inline void		writeData	(uint8_t data);
    inline uint8_t  readData	();
    inline void 	writeControl(uint8_t ctl);
    inline void	   	writeReset	(uint8_t state);
    inline void    	writeVCC	(uint8_t state);
    inline void    	writeXTAL	(uint8_t state);
    inline uint8_t	readRDY		();
  
  }
  namespace ISP {
    inline void 	initPins	();
    inline void		cleanup		();
    inline uint8_t  transferSPI (uint8_t data, uint8_t speed);
    //Based on speed given, decide to use HW SPI or bitbang.
    //Speed is start from 0=fastest reccomended. Valid values for speed are from 0 to minSpeed() inclusive.
    inline uint8_t  minSpeed();
    //Returns as slow as it can get. More slower speeds (i.e. larger) is better, but make this too large and it will take forever.
    inline void		writeReset(uint8_t state);
  }
}


/*
Target Specific Interface files.
Uncomment one, and only one, of the following lines to select a hardware interface.
Do ensure that it is suitable for your purposes!
*/

//#include "SMoHWIF_Mabel.h"
//#include "SMoHWIF_ProMini.h"
//#include "SMoHWIF_Uno.h"


#if !defined (_SMoHWIF_defined_)
#error "You need to #include an appropriate interface file in SMoHWIF.h! See line 50."
#endif


#endif //Include guard 
