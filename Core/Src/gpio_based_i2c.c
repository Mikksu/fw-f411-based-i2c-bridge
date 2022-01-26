#include "gpio_based_i2c.h"

#define PIN_SCL(x)        (PASet(9) = x)
#define PIN_SDA(x)        (PASet(10) = x)
#define PIN_SDA_IN        (PAGet(10))

static void delay()
{
  // delay about 5us
  volatile uint8_t i;
  //volatile uint8_t j;
  for (i = 0; i < 20; i++);
    //for(j = 0; j < 1; j++);

}

static int start(void)
{
  int busBusy = 0;

	PIN_SCL (1);
	PIN_SDA (1);
	delay();

	if (PIN_SDA_IN == 0)
	  busBusy = -1;

	PIN_SDA (0);
	delay();
	PIN_SCL (0);
	delay();	

	return busBusy;
}

static void stop(void)
{
	PIN_SCL (0);
	PIN_SDA (0);
	delay();
	PIN_SCL (1);
	delay();
	PIN_SDA (1);
	delay();
}

//产生应答信号
static void send_ack(void)
{
	PIN_SCL (0);
	delay();
	PIN_SDA (0);
	delay();
	PIN_SCL (1);
	delay();
	PIN_SCL (0);
	delay();
}

//产生非应答信号
static void send_nack(void)
{
	PIN_SCL (0);
	delay();
	PIN_SDA (1);
	delay();
	PIN_SCL (1);
	delay();
	PIN_SCL (0);
	delay();
}


static void send_byte(uint8_t send_byte)
{
	uint8_t i;
	
	for(i = 0 ; i < 8 ; i++)
	{
		PIN_SDA ((send_byte & 0x80) >> 7);
		delay();
		PIN_SCL (1);
		delay();
		PIN_SCL (0);
		if(i==7)
		{
			PIN_SDA (1);	//发送最后一位之后，释放SDA总线，方便接收来自接收端的应答信号
		}
		//delay();
		send_byte <<= 1;
	}
}


static uint8_t read_byte()
{
	uint8_t i;
	uint8_t receive = 0;

	for(i = 0; i < 8; i++)
	{
		receive <<= 1;
		PIN_SDA (1);
		PIN_SCL (1);
		delay();
		if(PIN_SDA_IN == 1)
		{
			receive++;
		}
		PIN_SCL (0);
		delay();
	}
	return receive;
}

static uint8_t wait_ack(void)
{
	uint8_t ack_flag = 0;
	
	PIN_SDA (1);	//释放SDA总线
	delay();
	PIN_SCL (1);
	delay();
	if(PIN_SDA_IN == 1)
	{	
		ack_flag = 1;	//非应答
	}
	else
	{
		ack_flag = 0;	//应答
	}
	PIN_SCL (0);
	delay();
	return ack_flag;
}

int I2C_Master_MemWrite(uint8_t slaveAddress, uint8_t regStart, uint8_t length, uint8_t *data)
{
  if(start() == -1) goto _bus_busy;
  send_byte(slaveAddress & 0xFE);
  if(wait_ack() == 1) goto _no_ack;
  send_byte(regStart);
  if(wait_ack() == 1) goto _no_ack;
  for(int i = 0; i < length; i++)
  {
    send_byte(*(data + i));
    if(wait_ack() == 1) goto _no_ack;
  }
  stop();
  return 0;

_no_ack:
  stop();
  return -1;

_bus_busy:
  stop();
  return -2;
}

int I2C_Master_MemRead(uint8_t slaveAddress, uint8_t regStart, uint8_t length, uint8_t *data)
{
  if(start() == -1) goto _bus_busy;
  send_byte(slaveAddress & 0xFE); // slave address with nW.
  if(wait_ack() == 1) goto _no_ack;
  send_byte(regStart); // send the register index to read
  if(wait_ack() == 1) goto _no_ack;
  if(start() == -1) goto _bus_busy; // restart
  send_byte(slaveAddress | 0x1); // slave address with R.
  if(wait_ack() == 1) goto _no_ack;
  for(int i = 0; i < length; i++)
  {
    *(data + i) = read_byte();
    if(i == (length - 1))
      send_nack();
    else
      send_ack();

  }
  stop();
  return 0;

_no_ack:
  stop();
  return -1;

_bus_busy:
  stop();
  return -2;
}
