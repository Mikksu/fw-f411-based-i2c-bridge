/* Functions called by the TouchGFX HAL to invoke the actual data transfer to ILI9341.
 * Pero, 2021
 */

#include "tftlcd.h"
#include "TouchGFX_DataTransfer.h"

extern void DisplayDriver_TransferCompleteCallback();

static uint8_t isTransmittingData = 0;

uint32_t touchgfxDisplayDriverTransmitActive(void)
{
	return isTransmittingData;
}

void touchgfxDisplayDriverTransmitBlock(uint8_t* pixels, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
	isTransmittingData = 1;
	LCD_Address_Set(x, y, x + w - 1, y + h - 1);
	LCD_DrawBitmap(w, h, pixels);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI1)
	{
		LCD_EndOfDrawBitmap();
		isTransmittingData = 0;
		DisplayDriver_TransferCompleteCallback();
	}
}