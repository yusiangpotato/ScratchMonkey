//Hardware Interface. 
//Modifying SMoHWIF.cpp should be able to get this working on any/all arduino variants.

#pragma once

namespace SMoHWIF{
	namespace HVSP{
		void initPins();
		void writeReset(uint8_t state);
		void writeSCI  (uint8_t state);
		void writeSDI  (uint8_t state);
		void writeSII  (uint8_t state);
	}
}