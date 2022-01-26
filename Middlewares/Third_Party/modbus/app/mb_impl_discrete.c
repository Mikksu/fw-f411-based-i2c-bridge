#include "port.h"
#include "mb.h"
#include "mbutils.h"
#include "mb_app.h"

static uint16_t usNDiscreteStart = DISCRETE_START;
#if DISCRETE_NDISCRETES % 8

uint8_t ucSDiscInBuf[DISCRETE_NDISCRETES / 8 + 1];
#else
uint8_t ucSDiscInBuf[DISCRETE_NDISCRETES / 8];
#endif

#ifdef MB_OS_USED
#include "cmsis_os.h"
#include "mb_os_def.h"

osMessageQDef(msgQueueDisc, 5, uint32_t);
osMessageQId msgQueueDiscHandle;

osPoolDef(poolDiscMsg, 5, MB_MSG_TypeDef);
osPoolId poolDiscMsgHandle;

void StartTaskRegDisc(void const * argument)
{
  // create the message queue.
  msgQueueDiscHandle = osMessageCreate(osMessageQ(msgQueueDisc), NULL);

  // create the message pool.
  poolDiscMsgHandle = osPoolCreate(osPool(poolDiscMsg));

  for(;;)
  {
    osEvent evt = osMessageGet(msgQueueDiscHandle, osWaitForever);
    if(evt.status == osEventMessage)
    {
        // get the pointer of the modbus message struct.
        MB_MSG_TypeDef* msg = evt.value.p;
        if(msg != NULL)
        {
            // some of the coil regs are changed.
        }
    }
  }
}

osThreadDef(regDiscTask, StartTaskRegDisc, osPriorityNormal, 0, 128);
osThreadId regDiscTaskHandle;

#endif

/**
 * Modbus slave discrete callback function.
 *
 * @param pucRegBuffer discrete buffer
 * @param usAddress discrete address
 * @param usNDiscrete discrete number
 *
 * @return result
 */
eMBErrorCode eMBRegDiscreteCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
    eMBErrorCode eStatus = MB_ENOERR;
    USHORT iRegIndex, iRegBitIndex, iNReg;
    iNReg = usNDiscrete / 8 + 1;

    usAddress--;

    if ((usAddress >= usNDiscreteStart) && (usAddress + usNDiscrete <= usNDiscreteStart + DISCRETE_NDISCRETES))
    {
        iRegIndex = (USHORT)(usAddress - usNDiscreteStart) / 8;
        iRegBitIndex = (USHORT)(usAddress - usNDiscreteStart) % 8;

        while (iNReg > 0)
        {
            *pucRegBuffer++ = xMBUtilGetBits(&ucSDiscInBuf[iRegIndex++],
                                             iRegBitIndex, 8);
            iNReg--;
        }
        pucRegBuffer--;
        /* last discrete */
        usNDiscrete = usNDiscrete % 8;
        /* filling zero to high bit */
        *pucRegBuffer = *pucRegBuffer << (8 - usNDiscrete);
        *pucRegBuffer = *pucRegBuffer >> (8 - usNDiscrete);
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

