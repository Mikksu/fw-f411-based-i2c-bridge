#include "port.h"
#include "mb.h"
#include "mbutils.h"
#include "mb_app.h"

static uint16_t usRegInputStart = REG_INPUT_START;
uint16_t usRegInputBuf[REG_INPUT_NREGS] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

#ifdef MB_OS_USED
#include "cmsis_os.h"
#include "mb_os_def.h"

osMessageQDef(msgQueueInput, 5, uint32_t);
osMessageQId msgQueueInputHandle;

osPoolDef(poolInputMsg, 5, MB_MSG_TypeDef);
osPoolId poolInputMsgHandle;

osThreadId regInputTaskHandle;

void StartTaskRegInput(void const * argument)
{
  // create the message queue.
  msgQueueInputHandle = osMessageCreate(osMessageQ(msgQueueInput), NULL);

  // create the message pool.
  poolInputMsgHandle = osPoolCreate(osPool(poolInputMsg));

  for(;;)
  {
    osEvent evt = osMessageGet(msgQueueInputHandle, osWaitForever);
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

/*
 * Create the task to process the coil registers.
 */
void CreateMbInputProcTask(void)
{
  osThreadDef(regInputTask, StartTaskRegInput, osPriorityNormal, 0, 128);
  regInputTaskHandle = osThreadCreate(osThread(regInputTask), NULL);
}

#endif



eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
    eMBErrorCode eStatus = MB_ENOERR;
    int iRegIndex;

    usAddress--;

    if ((usAddress >= usRegInputStart) && (usAddress + usNRegs <= usRegInputStart + REG_INPUT_NREGS)) //请求地址大于起始地址 && 地址长度小于设定长度
    {
        iRegIndex = (int)(usAddress - usRegInputStart);
        while (usNRegs > 0)
        {
            *pucRegBuffer++ =
                (unsigned char)(usRegInputBuf[iRegIndex] >> 8);
            *pucRegBuffer++ =
                (unsigned char)(usRegInputBuf[iRegIndex] & 0xFF);
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

