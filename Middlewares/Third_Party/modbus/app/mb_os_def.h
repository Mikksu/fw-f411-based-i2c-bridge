#ifndef __MB_OS_DEF_H
#define	__MB_OS_DEF_H

#include "stdint.h"

typedef struct
{
	uint16_t RegIndex;					/*< the index of the first register  				*/
	uint16_t NRegs;							/*< the number of registers value changed. 	*/
	uint16_t RegBitIndex;

} MB_MSG_TypeDef;

void CreateMbCoilProcTask(void);
void CreateMbHoldingProcTask(void);

#endif
