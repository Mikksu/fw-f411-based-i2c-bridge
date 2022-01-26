#include "main.h"
#include "tftlcd.h"
#include "spi.h"

//LCD缓存大小设置，修改此值时请注意！！！！修改这两个值时可能会影响以下函数  LCD_Clear/LCD_Fill/LCD_DrawLine
#define LCD_TOTAL_BUF_SIZE  (240*135*2)
#define LCD_Buf_Size 1152
static uint8_t lcd_buf[LCD_Buf_Size];

uint16_t POINT_COLOR = CYAN; //画笔颜色  默认为黑色
uint16_t BACK_COLOR  = BLACK;  //背景颜色  默认为白色

/**
 * @brief LCD控制接口初始化
 *
 * @param   void
 *
 * @return  void
 */
//static void LCD_Gpio_Init(void)
//{
//  GPIO_InitTypeDef  GPIO_InitStructure;
//
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC, ENABLE);   //使能PA,PC端口时钟

//    /*
//      LCD_PWR:  PA4
//      LCD_RST:  PA1
//      LCD_DC:   PA6
//      LCD_CS:   PC4
//    */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 |GPIO_Pin_4|GPIO_Pin_6;//PA1.4.6 端口配置
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //推挽输出
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    //IO口速度为50MHz
//  GPIO_Init(GPIOA, &GPIO_InitStructure);           //根据设定参数初始化
//  GPIO_SetBits(GPIOA,GPIO_Pin_1 |GPIO_Pin_4|GPIO_Pin_6);   //PA1.4.6 输出高

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; //PC4 端口配置
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //推挽输出
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    //IO口速度为50MHz
//  GPIO_Init(GPIOC, &GPIO_InitStructure);           //根据设定参数初始化
//  GPIO_SetBits(GPIOC,GPIO_Pin_4);  //PC4输出高

//    LCD_CS = 0;
//    LCD_PWR = 0;

//    LCD_RST = 0;
//    delay_ms(120);
//    LCD_RST = 1;
//
//  SPI1_Init();  //初始化SPI2接口
//}


/**
 * @brief     交换RGB565数据的高低字节。
 * @param     *fb   FrameBuffer起始地址
 * @param     len   FrameBuffer中的像素总数
 *
 */
static void SwapRGB565Bytes(uint8_t *fb, int32_t len)
{
  uint8_t v;
  while (len > 0)
  {
    v = *(fb + 1);
    *(fb + 1) = *fb;
    *fb = v;
    fb += 2;
    len -= 2;
  }
}

/**
 * @brief LCD底层SPI发送数据函数
 *
 * @param   data  数据的起始地址
 * @param   size  发送数据大小
 *
 * @return  void
 */
static void LCD_SPI_Send(uint8_t *data, uint16_t size)
{
  HAL_SPI_Transmit(&hspi1, data, size, 100);
}


/**
 * @brief 写命令到LCD
 *
 * @param   cmd   需要发送的命令
 *
 * @return  void
 */
static void LCD_Write_Cmd(uint8_t cmd)
{
    LCD_DC(0);

    LCD_SPI_Send(&cmd, 1);
}

/**
 * @brief 写数据到LCD
 *
 * @param   cmd   需要发送的数据
 *
 * @return  void
 */
static void LCD_Write_Data(uint8_t data)
{
    LCD_DC(1);

    LCD_SPI_Send(&data, 1);
}

/**
 * @brief 写半个字的数据到LCD
 *
 * @param   cmd   需要发送的数据
 *
 * @return  void
 */
void LCD_Write_HalfWord(const uint16_t da)
{
    uint8_t data[2] = {0};

    data[0] = da >> 8;
    data[1] = da;

    LCD_DC(1);
    LCD_SPI_Send(data, 2);
}


/**
 * 设置数据写入LCD缓存区域
 *
 * @param   x1,y1 起点坐标
 * @param   x2,y2 终点坐标
 *
 * @return  void
 */
void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  if(USE_HORIZONTAL==0)
  {
    LCD_Write_Cmd(0x2a);//列地址设置
    LCD_Write_HalfWord(x1+52);
    LCD_Write_HalfWord(x2+52);
    LCD_Write_Cmd(0x2b);//行地址设置
    LCD_Write_HalfWord(y1+40);
    LCD_Write_HalfWord(y2+40);
    LCD_Write_Cmd(0x2c);//储存器写
  }
  else if(USE_HORIZONTAL==1)
  {
    LCD_Write_Cmd(0x2a);//列地址设置
    LCD_Write_HalfWord(x1+53);
    LCD_Write_HalfWord(x2+53);
    LCD_Write_Cmd(0x2b);//行地址设置
    LCD_Write_HalfWord(y1+40);
    LCD_Write_HalfWord(y2+40);
    LCD_Write_Cmd(0x2c);//储存器写
  }
  else if(USE_HORIZONTAL==2)
  {
    LCD_Write_Cmd(0x2a);//列地址设置
    LCD_Write_HalfWord(x1+40);
    LCD_Write_HalfWord(x2+40);
    LCD_Write_Cmd(0x2b);//行地址设置
    LCD_Write_HalfWord(y1+53);
    LCD_Write_HalfWord(y2+53);
    LCD_Write_Cmd(0x2c);//储存器写
  }
  else
  {
    LCD_Write_Cmd(0x2a);//列地址设置
    LCD_Write_HalfWord(x1+40);
    LCD_Write_HalfWord(x2+40);
    LCD_Write_Cmd(0x2b);//行地址设置
    LCD_Write_HalfWord(y1+52);
    LCD_Write_HalfWord(y2+52);
    LCD_Write_Cmd(0x2c);//储存器写
  }
}

/**
 * 打开LCD显示
 *
 * @param   void
 *
 * @return  void
 */
void LCD_DisplayOn(void)
{
    LCD_PWR(0);
}
/**
 * 关闭LCD显示
 *
 * @param   void
 *
 * @return  void
 */
void LCD_DisplayOff(void)
{
    LCD_PWR(1);
}

/**
 * 以一种颜色清空LCD屏
 *
 * @param   color 清屏颜色
 *
 * @return  void
 */
void LCD_Clear(uint16_t color)
{
//    uint16_t i, j;
//    uint8_t data[2] = {0};

//    data[0] = color >> 8;
//    data[1] = color;

//    LCD_Address_Set(0, 0, LCD_Width - 1, LCD_Height - 1);

//    for(j = 0; j < LCD_Buf_Size / 2; j++)
//    {
//        lcd_buf[j * 2] =  data[0];
//        lcd_buf[j * 2 + 1] =  data[1];
//    }

//    LCD_DC(1);

//    for(i = 0; i < (LCD_TOTAL_BUF_SIZE / LCD_Buf_Size); i++)
//    {
//        LCD_SPI_Send(lcd_buf, LCD_Buf_Size);
//    }
  LCD_Fill(0,0,LCD_Width,LCD_Height,color);
}


void LCD_DrawBitmap(uint16_t w, uint16_t h, uint8_t *s)
{
  uint32_t bufSize = 0;
  HAL_StatusTypeDef stat;

  bufSize = w * h * 2;

  LCD_DC(1);

  SwapRGB565Bytes(s, bufSize);

  //HAL_SPI_Transmit(&hspi1, s, (uint16_t)bufSize, 100);
  stat = HAL_SPI_Transmit_IT(&hspi1, s, (uint16_t)bufSize);
  stat = stat;

}

void LCD_EndOfDrawBitmap(void)
{

}


/**
 * 用一个颜色填充整个区域
 *
 * @param   x_start,y_start     起点坐标
 * @param   x_end,y_end     终点坐标
 * @param   color           填充颜色
 *
 * @return  void
 */
void LCD_Fill(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t color)
{
    uint16_t i = 0;
    uint32_t size = 0, size_remain = 0;

    size = (x_end - x_start + 1) * (y_end - y_start + 1) * 2;

    if(size > LCD_Buf_Size)
    {
        size_remain = size - LCD_Buf_Size;
        size = LCD_Buf_Size;
    }

    LCD_Address_Set(x_start, y_start, x_end, y_end);

    while(1)
    {
        for(i = 0; i < size / 2; i++)
        {
            lcd_buf[2 * i] = color >> 8;
            lcd_buf[2 * i + 1] = color;
        }

        LCD_DC(1);
        LCD_SPI_Send(lcd_buf, size);

        if(size_remain == 0)
            break;

        if(size_remain > LCD_Buf_Size)
        {
            size_remain = size_remain - LCD_Buf_Size;
        }

        else
        {
            size = size_remain;
            size_remain = 0;
        }
    }
}

/**
 * 画点函数
 *
 * @param   x,y   画点坐标
 *
 * @return  void
 */
void LCD_Draw_Point(uint16_t x, uint16_t y)
{
    LCD_Address_Set(x, y, x, y);
    LCD_Write_HalfWord(POINT_COLOR);
}
void LCD_Draw_Point1(uint16_t x, uint16_t y,uint8_t t)
{
    LCD_Address_Set(x, y, x, y);
  if(t==1)
    LCD_Write_HalfWord(POINT_COLOR);
  if(t==0)
    LCD_Write_HalfWord(BACK_COLOR);
}
/**
 * 画点带颜色函数
 *
 * @param   x,y   画点坐标
 *
 * @return  void
 */
void LCD_Draw_ColorPoint(uint16_t x, uint16_t y,uint16_t color)
{
    LCD_Address_Set(x, y, x, y);
    LCD_Write_HalfWord(color);
}

/**
 * @brief 画线函数(直线、斜线)
 *
 * @param   x1,y1 起点坐标
 * @param   x2,y2 终点坐标
 *
 * @return  void
 */
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, row, col;
    uint32_t i = 0;

    if(y1 == y2)
    {
        /*快速画水平线*/
        LCD_Address_Set(x1, y1, x2, y2);

        for(i = 0; i < x2 - x1; i++)
        {
            lcd_buf[2 * i] = POINT_COLOR >> 8;
            lcd_buf[2 * i + 1] = POINT_COLOR;
        }

        LCD_DC(1);
        LCD_SPI_Send(lcd_buf, (x2 - x1) * 2);
        return;
    }

    delta_x = x2 - x1;
    delta_y = y2 - y1;
    row = x1;
    col = y1;

    if(delta_x > 0)incx = 1;

    else if(delta_x == 0)incx = 0;

    else
    {
        incx = -1;
        delta_x = -delta_x;
    }

    if(delta_y > 0)incy = 1;

    else if(delta_y == 0)incy = 0;

    else
    {
        incy = -1;
        delta_y = -delta_y;
    }

    if(delta_x > delta_y)distance = delta_x;

    else distance = delta_y;

    for(t = 0; t <= distance + 1; t++)
    {
        LCD_Draw_Point(row, col);
        xerr += delta_x ;
        yerr += delta_y ;

        if(xerr > distance)
        {
            xerr -= distance;
            row += incx;
        }

        if(yerr > distance)
        {
            yerr -= distance;
            col += incy;
        }
    }
}

/**
 * @brief 画一个矩形
 *
 * @param   x1,y1 起点坐标
 * @param   x2,y2 终点坐标
 *
 * @return  void
 */
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    LCD_DrawLine(x1, y1, x2, y1);
    LCD_DrawLine(x1, y1, x1, y2);
    LCD_DrawLine(x1, y2, x2, y2);
    LCD_DrawLine(x2, y1, x2, y2);
}

/**
 * @brief 画一个圆
 *
 * @param   x0,y0 圆心坐标
 * @param   r       圆半径
 *
 * @return  void
 */
void LCD_Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r)
{
    int a, b;
    int di;
    a = 0;
    b = r;
    di = 3 - (r << 1);

    while(a <= b)
    {
        LCD_Draw_Point(x0 - b, y0 - a);
        LCD_Draw_Point(x0 + b, y0 - a);
        LCD_Draw_Point(x0 - a, y0 + b);
        LCD_Draw_Point(x0 - b, y0 - a);
        LCD_Draw_Point(x0 - a, y0 - b);
        LCD_Draw_Point(x0 + b, y0 + a);
        LCD_Draw_Point(x0 + a, y0 - b);
        LCD_Draw_Point(x0 + a, y0 + b);
        LCD_Draw_Point(x0 - b, y0 + a);
        a++;

        if(di < 0)di += 4 * a + 6;
        else
        {
            di += 10 + 4 * (a - b);
            b--;
        }

        LCD_Draw_Point(x0 + a, y0 + b);
    }
}


/**
 * @brief m^n函数
 *
 * @param   m,n   输入参数
 *
 * @return  m^n次方
 */
static uint32_t LCD_Pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;

    while(n--)result *= m;

    return result;
}

/**
 * @brief 显示数字,高位为0不显示
 *
 * @param   x,y   起点坐标
 * @param   num   需要显示的数字,数字范围(0~4294967295)
 * @param   len   需要显示的位数
 * @param   size  字体大小
 *
 * @return  void
 */
void LCD_ShowNum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    for(t = 0; t < len; t++)
    {
        temp = (num / LCD_Pow(10, len - t - 1)) % 10;

        if(enshow == 0 && t < (len - 1))
        {
            if(temp == 0)
            {
                LCD_ShowChar(x + (size / 2)*t, y, ' ', size);
                continue;
            }

            else enshow = 1;
        }

        LCD_ShowChar(x + (size / 2)*t, y, temp + '0', size);
    }
}



/**
 * @brief 显示数字,高位为0,可以控制显示为0还是不显示
 *
 * @param   x,y   起点坐标
 * @param   num   需要显示的数字,数字范围(0~999999999)
 * @param   len   需要显示的位数
 * @param   size  字体大小
 * @param   mode  1:高位显示0   0:高位不显示
 *
 * @return  void
 */
void LCD_ShowxNum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint8_t mode)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    for(t = 0; t < len; t++)
    {
        temp = (num / LCD_Pow(10, len - t - 1)) % 10;

        if(enshow == 0 && t < (len - 1))
        {
            if(temp == 0)
            {
                if(mode)LCD_ShowChar(x + (size / 2)*t, y, '0', size);
                else
                    LCD_ShowChar(x + (size / 2)*t, y, ' ', size);

                continue;
            }

            else enshow = 1;
        }

        LCD_ShowChar(x + (size / 2)*t, y, temp + '0', size);
    }
}

void LCD_ShowChar(uint16_t x, uint16_t y, char chr, uint8_t size)
{
	
}


/**
 * @brief 显示字符串
 *
 * @param   x,y   起点坐标
 * @param   width 字符显示区域宽度
 * @param   height  字符显示区域高度
 * @param   size  字体大小
 * @param   p   字符串起始地址
 *
 * @return  void
 */
void LCD_ShowString(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, char *p)
{
    uint8_t x0 = x;
    width += x;
    height += y;

    while((*p <= '~') && (*p >= ' ')) //判断是不是非法字符!
    {
        if(x >= width)
        {
            x = x0;
            y += size;
        }

        if(y >= height)break; //退出

        LCD_ShowChar(x, y, *p, size);
        x += size / 2;
        p++;
    }
}


/**
 * @brief 显示图片
 *
 * @remark  Image2Lcd取模方式：  C语言数据/水平扫描/16位真彩色(RGB565)/高位在前    其他的不要选
 *
 * @param   x,y   起点坐标
 * @param   width 图片宽度
 * @param   height  图片高度
 * @param   p   图片缓存数据起始地址
 *
 * @return  void
 */
void LCD_Show_Image(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *p)
{
    if(x + width > LCD_Width || y + height > LCD_Height)
    {
        return;
    }

    LCD_Address_Set(x, y, x + width - 1, y + height - 1);

    LCD_DC(1);

    LCD_SPI_Send((uint8_t *)p, width * height * 2);
}

/**
 * @brief LCD初始化
 *
 * @param   void
 *
 * @return  void
 */
 void LCD_Init(void)
{
    LCD_PWR(0);
    HAL_Delay(120);
    LCD_RST(0);
    HAL_Delay(120);
    LCD_RST(1);

    HAL_Delay(120);
    /* Sleep Out */
    LCD_Write_Cmd(0x11);
    /* wait for power stability */
    HAL_Delay(120);

    /* Memory Data Access Control */
    LCD_Write_Cmd(0x36);
    if(USE_HORIZONTAL==0)
      LCD_Write_Data(0x00);
    else if(USE_HORIZONTAL==1)
      LCD_Write_Data(0xC0);
    else if(USE_HORIZONTAL==2)
      LCD_Write_Data(0x70);
    else LCD_Write_Data(0xA0);

    /* RGB 5-6-5-bit  */
    LCD_Write_Cmd(0x3A);
    LCD_Write_Data(0x05);

    /* Porch Setting */
    LCD_Write_Cmd(0xB2);
    LCD_Write_Data(0x0C);
    LCD_Write_Data(0x0C);
    LCD_Write_Data(0x00);
    LCD_Write_Data(0x33);
    LCD_Write_Data(0x33);

    /*  Gate Control */
    LCD_Write_Cmd(0xB7);
    LCD_Write_Data(0x35);

    /* VCOM Setting */
    LCD_Write_Cmd(0xBB);
    LCD_Write_Data(0x19);   //Vcom=1.625V

    /* LCM Control */
    LCD_Write_Cmd(0xC0);
    LCD_Write_Data(0x2C);

    /* VDV and VRH Command Enable */
    LCD_Write_Cmd(0xC2);
    LCD_Write_Data(0x01);

    /* VRH Set */
    LCD_Write_Cmd(0xC3);
    LCD_Write_Data(0x12);

    /* VDV Set */
    LCD_Write_Cmd(0xC4);
    LCD_Write_Data(0x20);

    /* Frame Rate Control in Normal Mode */
    LCD_Write_Cmd(0xC6);
    LCD_Write_Data(0x0F); //60MHZ

    /* Power Control 1 */
    LCD_Write_Cmd(0xD0);
    LCD_Write_Data(0xA4);
    LCD_Write_Data(0xA1);

    /* Positive Voltage Gamma Control */
    LCD_Write_Cmd(0xE0);
    LCD_Write_Data(0xD0);
    LCD_Write_Data(0x04);
    LCD_Write_Data(0x0D);
    LCD_Write_Data(0x11);
    LCD_Write_Data(0x13);
    LCD_Write_Data(0x2B);
    LCD_Write_Data(0x3F);
    LCD_Write_Data(0x54);
    LCD_Write_Data(0x4C);
    LCD_Write_Data(0x18);
    LCD_Write_Data(0x0D);
    LCD_Write_Data(0x0B);
    LCD_Write_Data(0x1F);
    LCD_Write_Data(0x23);

    /* Negative Voltage Gamma Control */
    LCD_Write_Cmd(0xE1);
    LCD_Write_Data(0xD0);
    LCD_Write_Data(0x04);
    LCD_Write_Data(0x0C);
    LCD_Write_Data(0x11);
    LCD_Write_Data(0x13);
    LCD_Write_Data(0x2C);
    LCD_Write_Data(0x3F);
    LCD_Write_Data(0x44);
    LCD_Write_Data(0x51);
    LCD_Write_Data(0x2F);
    LCD_Write_Data(0x1F);
    LCD_Write_Data(0x1F);
    LCD_Write_Data(0x20);
    LCD_Write_Data(0x23);

    /* Display Inversion On */
    LCD_Write_Cmd(0x21);

    LCD_Write_Cmd(0x29);

    LCD_Address_Set(0, 0, LCD_Width - 1, LCD_Height - 1);

    /*打开显示*/
    LCD_PWR(1);

}



