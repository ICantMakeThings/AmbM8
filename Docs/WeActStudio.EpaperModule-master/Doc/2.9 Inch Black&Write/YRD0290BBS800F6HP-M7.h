#include "sys.h"
#include "delay.h"  

#define SPI_OUT 1
#define SPI_IN  0

#define Epaper_EN PAout(12)                                  // Epaper_EN
#define Epaper_BUSY GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9) //
#define Epaper_RESET PCout(8)                                // RESET
#define Epaper_BS PAout(8)                                   // BS
#define Epaper_DC PCout(7)                                   // D/C#
#define Epaper_CS PCout(6)                                   //
#define SPI_CLK PBout(13)                                    //
#define SPI_MOSI PBout(15)                                   // SDO
 
#define	 MAX_LINE_BYTES   16
#define  MAX_COLUMN_BYTES 296	
#define  ALLSCREEN_BYTES  4736

#define	 X_Addr_Start    0x01
#define  X_Addr_End      0x10	
#define  Y_Addr_Start_H  0x01  
#define  Y_Addr_Start_L  0x27
#define  Y_Addr_End_H    0x00
#define  Y_Addr_End_L    0x00

