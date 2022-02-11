#ifndef __MB_APP_H
#define __MB_APP_H

#include "stdint.h"


/**
 * @brief         The maximum size of the buffer for the DUT i2c operation
 */
#define MAX_SIZE_DUT_I2C_BUF            (10)

/**
 * @brief         The starting position of the IIC receive buffer.
 * 
 */
#define REG_INPUT_POS_DUT_IIC_RX_BUFF   (70)

/**
 * @brief         The starting position of the IIC transimit buffer.
 * 
 */
#define REG_HOLDING_POS_DUT_IIC_TX_BUFF (80)

/**
 * @brief         The position to input IIC operation code in the Holding Registers.
 * 
 */
#define REG_HOLDING_POS_EXECUTE         (99)


/**
 * @brief         The definition of the Error Code.
 */
#define ERR_NO                          (0)
#define ERR_PID_PARAM                   (-10)         /*!< Invalid PID parameters             */
#define ERR_PID_INVALID_RTTEMP          (-11)         /*!< Invalid real-time temperature read while PID tuning    */
#define ERR_PID_INVALID_TARGET_TEMP     (-12)         /*!< Invalid target temperature    */
#define ERR_PID_RTTEMP_TOO_LOW          (-13)         /*!< The real time temp. is too low       */
#define ERR_PID_RTTEMP_TOO_HIGH         (-14)         /*!< The real time temp. is too high       */
#define ERR_DUT_I2C_NO_ACK              (-20)         /*!< No ack detected on the I2C bus to communicate with the DUT    */
#define ERR_DUT_I2C_BUS_BUSY            (-21)         /*!< The SDA line can not be pull to high while starting the DUT i2c bus        */

#define ERR_UNDEFINED                   (-999)        /*!< Undefined error    */

/**
 * @brief       I2C Operation parameters
 * 
 */
typedef struct 
{
  uint16_t                      SlaveAddress;
  uint16_t                      TxLength;
	uint16_t                      RxLength;
  uint16_t                      TxBuff[MAX_SIZE_DUT_I2C_BUF];
  
} DutI2cOper_TypeDef;




#define COIL_START 0
#define COIL_NCOILS 8
#if COIL_NCOILS % 8
extern uint8_t usCoilBuf[COIL_NCOILS / 8 + 1];
#else
extern uint8_t usCoilBuf[COIL_NCOILS / 8];
#endif



#define DISCRETE_START 0
#define DISCRETE_NDISCRETES 8
#if DISCRETE_NDISCRETES % 8
extern uint8_t ucSDiscInBuf[DISCRETE_NDISCRETES / 8 + 1];
#else
extern uint8_t ucSDiscInBuf[DISCRETE_NDISCRETES / 8];
#endif


#define REG_INPUT_START 0
#define REG_INPUT_NREGS 100
extern uint16_t usRegInputBuf[REG_INPUT_NREGS];


#define REG_HOLDING_START 0
#define REG_HOLDING_NREGS 100
extern uint16_t usRegHoldingBuf[REG_HOLDING_NREGS];


extern void CreateMbHoldingProcTask(void);

#endif
