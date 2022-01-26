#include "port.h"
#include "mb.h"
#include "mbutils.h"
#include "mb_app.h"

static uint16_t usCoilStart = COIL_START;
#if COIL_NCOILS % 8
uint8_t usCoilBuf[COIL_NCOILS / 8 + 1];
#else
uint8_t usCoilBuf[COIL_NCOILS / 8];
#endif

#ifdef MB_OS_USED

#include "cmsis_os.h"
#include "mb_os_def.h"

/*
 * The message queue to pass the message indicates where and how many registers are modified.
 */
osMessageQDef(msgQueueCoil, 5, uint32_t);
osMessageQId msgQueueCoilHandle;

/*
 * The pool of the message.
 */
osPoolDef(poolCoilMsg, 5, MB_MSG_TypeDef);
osPoolId poolCoilMsgHandle;

/*
 * The handle of the processing task.
 */
osThreadId regCoilTaskHandle;

static void StartTaskRegCoil(void const * argument)
{
	/*
  // create the message queue.
  msgQueueCoilHandle = osMessageCreate(osMessageQ(msgQueueCoil), NULL);

  // create the message pool.
  poolCoilMsgHandle = osPoolCreate(osPool(poolCoilMsg));

  for(;;)
  {
    // wait the message.
    osEvent evt = osMessageGet(msgQueueCoilHandle, osWaitForever);
    if(evt.status == osEventMessage)
    {
        // get the pointer of the modbus message struct.
        MB_MSG_TypeDef* msg = evt.value.p;
        if(msg != NULL)
        {
          int nbits = msg->NRegs;
          int regIndex = msg->RegIndex;
          int regBitIndex = msg->RegBitIndex;
          uint8_t bitValue;

          while(nbits > 0)
          {
            switch(regIndex)
            {
              case 0: // ucCoilBuf[0]

                bitValue = BIT_ADDR((uint32_t)usCoilBuf, regBitIndex);

                if(regBitIndex == 0)  // VCC1 ON/OFF
                {
                  if(bitValue == 0) Top_TurnOffVcc1();
                  else Top_TurnOnVcc1();
                }
                else if(regBitIndex == 1) // VCC2 ON/OFF
                {
                  if(bitValue == 0) Top_TurnOffVcc2();
                  else Top_TurnOnVcc2();
                }
                else if(regBitIndex == 2) // VCC3 ON/OFF
                {
                  if(bitValue == 0) Top_TurnOffVcc3();
                  else Top_TurnOnVcc3();
                }
                else if(regBitIndex == 3) // TEC ON/OFF
                {
                  if(bitValue == 0) Top_TurnOffTec();
                  else Top_TurnOnTec();
                }
                else if(regBitIndex == 4) // TOSA POWER SUPPLY ON/OFF
                {
                  if(bitValue == 0) // turn off the power supply
                  {
                    Top_TurnOffVcc3();
                    Top_TurnOffVcc2();
                    HAL_Delay(50);
                    Top_TurnOffVcc1();

                    Top_DutReset();
                  }
                  else
                  {
                    Top_TurnOnVcc1();
                    HAL_Delay(50);
                    Top_TurnOnVcc2();
                    Top_TurnOnVcc3();
                    HAL_Delay(10);
                    Top_DutUnreset();
                  }
                }
                else if(regBitIndex == 5) // TOSA Tx Disable
                {
                  if(bitValue == 0) Top_DutTxDisable();
                  else Top_DutTxEnable();
                }
                else if(regBitIndex == 6) // TOSA Reset
                {
                  if(bitValue == 0) Top_DutReset();
                  else Top_DutUnreset();
                }
                break; // end of ucCoilBuf[0]
            }

            // move to the next bit.
            nbits--;
            regBitIndex++;
            if(regBitIndex >= 8)
            {
              regBitIndex = 0;
              regIndex++;
            }
          }
        }

        osPoolFree(poolCoilMsgHandle, (void*)msg);
    }
  }
	*/
}

/*
 * Create the task to process the coil registers.
 */
void CreateMbCoilProcTask(void)
{
  osThreadDef(regCoilTask, StartTaskRegCoil, osPriorityNormal, 0, 128);
  regCoilTaskHandle = osThreadCreate(osThread(regCoilTask), NULL);
}

#endif

/**
 * Modbus slave coils callback function.
 *
 * @param pucRegBuffer coils buffer
 * @param usAddress coils address
 * @param usNCoils coils number
 * @param eMode read or write
 *
 * @return result
 */
eMBErrorCode
eMBRegCoilsCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNCoils,
              eMBRegisterMode eMode)
{
    eMBErrorCode eStatus = MB_ENOERR;
    USHORT iRegIndex, iRegBitIndex, iNReg;
    iNReg = usNCoils / 8 + 1;

    /*
#ifdef MB_OS_USED
    MB_MSG_TypeDef* msg;
#endif
*/

    usAddress--;

    if ((usAddress >= usCoilStart) && (usAddress + usNCoils <= usCoilStart + COIL_NCOILS))
    {
        iRegIndex = (USHORT)(usAddress - usCoilStart) / 8;
        iRegBitIndex = (USHORT)(usAddress - usCoilStart) % 8;
        switch (eMode)
        {
        /* read current coil values from the protocol stack. */
        case MB_REG_READ:
            while (iNReg > 0)
            {
                *pucRegBuffer++ = xMBUtilGetBits(&usCoilBuf[iRegIndex++], iRegBitIndex, 8);
                iNReg--;
            }
            pucRegBuffer--;
            /* last coils */
            usNCoils = usNCoils % 8;
            /* filling zero to high bit */
            *pucRegBuffer = *pucRegBuffer << (8 - usNCoils);
            *pucRegBuffer = *pucRegBuffer >> (8 - usNCoils);
            break;

            /* write current coil values with new values from the protocol stack. */
        case MB_REG_WRITE:

          /*
#ifdef MB_OS_USED
            // send a message to tell the task there are some registers are set.

            msg = osPoolCAlloc(poolCoilMsgHandle);
            if(msg != NULL)
            {
               msg->RegIndex = iRegIndex;
               msg->NRegs = usNCoils;
               msg->RegBitIndex = iRegBitIndex;
            }
#endif
*/

            while (iNReg > 1)
            {
                xMBUtilSetBits(&usCoilBuf[iRegIndex++], iRegBitIndex, 8, *pucRegBuffer++);
                iNReg--;
            }
            /* last coils */
            usNCoils = usNCoils % 8;
            /* xMBUtilSetBits has bug when ucNBits is zero */
            if (usNCoils != 0)
            {
                xMBUtilSetBits(&usCoilBuf[iRegIndex++], iRegBitIndex, usNCoils, *pucRegBuffer++);
            }

            /*
#ifdef MB_OS_USED
            if(msg != NULL)
              osMessagePut(msgQueueCoilHandle, (uint32_t)msg, 100);

#endif
*/

            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

