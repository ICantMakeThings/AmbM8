#include "DEPG0290RWS800F6HP.h"
#include "Image.h"
 
void EpaperIO_Init(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;

   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE); // 使能PB端口时钟

   // EPAPER_BS N
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; // 端口配置, 推挽输出
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   // EPAPER_EN
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; // 端口配置, 推挽输出
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   // SPI_CLK-->PB13    SPI_MOSI--->PB15
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15; // 端口配置, 推挽输出
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   //  CS-->PC6    D/C--->PC7   RES-->PC8
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_12; // 端口配置, 推挽输出
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOC, &GPIO_InitStructure);

   // BUSY--->PC9
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 下拉输入
   GPIO_Init(GPIOC, &GPIO_InitStructure);                // 初始化GPIO

   Epaper_BS = 0;
}
void SPI_DATA_IO_Init(uint8_t mode)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // ??PB????

    if (mode == SPI_OUT)
    {
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; // ????, ????
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
    }

    if (mode == SPI_IN)
    {
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // ????
        GPIO_Init(GPIOB, &GPIO_InitStructure);        // ???GPIO
    }
}

void Epaper_Spi_WriteByte(u8 TxData)
{
   u8 TempData;
   u8 scnt;
   TempData = TxData;
   SPI_DATA_IO_Init(SPI_OUT);
   SPI_CLK = 0;
   for (scnt = 0; scnt < 8; scnt++)
   {
      if (TempData & 0x80)
         SPI_MOSI = 1;
      else
         SPI_MOSI = 0;
      SPI_CLK = 1;
      delay_us(5);
      SPI_CLK = 0;
      TempData = TempData << 1;
      delay_us(5);
   }
}

void Epaper_Write_Command(u8 cmd)
{
   Epaper_CS = 1;
   Epaper_CS = 0;
   Epaper_DC = 0; // D/C#   0:command  1:data

   delay_us(5);
   Epaper_Spi_WriteByte(cmd);
   delay_us(5);
   Epaper_CS = 1;
}

void Epaper_Write_Data(u8 data)
{
   Epaper_CS = 1;
   Epaper_CS = 0;
   Epaper_DC = 1; // D/C#   0:command  1:data

   delay_us(5);
   Epaper_Spi_WriteByte(data);
   delay_us(5);
   Epaper_CS = 1;
}

void Epaper_READBUSY(void)
{
   while (1)
   {
      if (Epaper_BUSY == 0)
         break;
      delay_us(5);
   }
   delay_us(100);
}


/*
 * 函数名：Epaper_Init
 * 描述  ：电子纸初始化程序
 * 输入  ：无
 * 输出  ：无
 */

void Epaper_Initial(void)
{
   Epaper_EN=1;
   delay_ms(100);
   Epaper_RESET = 0; // EPD 硬件复位    Hardware  reset
   delay_ms(10);
   Epaper_RESET = 1; // EPD 硬件复位释放    Hard  reset  release
   delay_ms(10);
   Epaper_READBUSY(); 	// 读busy信号

   Epaper_Write_Command(0x12); // 软件复位    soft  reset
   Epaper_READBUSY();
	
	Epaper_Write_Command(0x01);  			 //?????? drive output control    
	Epaper_Write_Data(Y_Addr_Start_L); //Y ???? 
	Epaper_Write_Data(Y_Addr_Start_H); //Y ???? 
	Epaper_Write_Data(0x00);
	

	Epaper_Write_Command(0x11);        //Data entry mode setting
	Epaper_Write_Data(0x01);           //00 Y decrement, X decrement,
																		 //01 Y decrement, X increment,
																		 //10 Y  increment, X decrement,
																		 //11 Y increment, X increment    

 
	Epaper_Write_Command(0x44);        // Set RAM X - address Start / End position
	Epaper_Write_Data(X_Addr_Start);
	Epaper_Write_Data(X_Addr_End);
 
	Epaper_Write_Command(0x45);        // Set RAM Y - address Start / End position   
	Epaper_Write_Data(Y_Addr_Start_L);
	Epaper_Write_Data(Y_Addr_Start_H);
	Epaper_Write_Data(Y_Addr_End_L);
	Epaper_Write_Data(Y_Addr_End_H); 
	
	Epaper_Write_Command(0x4E);        //Set RAM X address counter
	Epaper_Write_Data(X_Addr_Start);
 
 
	Epaper_Write_Command(0x4F);        //Set RAM Y address counter  
	Epaper_Write_Data(Y_Addr_Start_L);
	Epaper_Write_Data(Y_Addr_Start_H);	 

	Epaper_Write_Command(0x3C);        //Border   Border setting 
	Epaper_Write_Data(0x05);
	
	Epaper_Write_Command(0x21);          
	Epaper_Write_Data(0x00);
	Epaper_Write_Data(0x00);
}

 void Epaper_Update_and_Deepsleep(void)
{
   Epaper_Write_Command(0x20);
   Epaper_READBUSY();

   Epaper_Write_Command(0x10);
   Epaper_Write_Data(0x01);
   delay_ms(100);
}

void Epaper_Load_Image(u8 *BWdatas,u8 *RWdatas,u32 all_bytes)
{
  u32 i; 
  u8 tempOriginal;      
  u32 tempcol=0;
	u32 tempcol1=0;
  u32 templine=0;
  u32 templine1=0;


	  Epaper_Write_Command(0x24); //DTM1
		for(i=0;i<all_bytes;i++)
		{          
			tempOriginal=*(BWdatas+templine*MAX_COLUMN_BYTES+tempcol);
			templine++;
			if(templine>=MAX_LINE_BYTES)
			{
				tempcol++;
				templine=0;
			}     
			Epaper_Write_Data(tempOriginal);
		} 

	

		Epaper_Write_Command(0x26); //DTM2
		for(i=0;i<all_bytes;i++)
		{          
			tempOriginal=*(RWdatas+templine1*MAX_COLUMN_BYTES+tempcol1);
			templine1++;
			if(templine1>=MAX_LINE_BYTES)
			{
				tempcol1++;
				templine1=0;
			}     
			Epaper_Write_Data(tempOriginal);
		} 

}
 
void Epaper_Load_White(void)
{
   u32 i, j;

     Epaper_Write_Command(0x24);
   for (i = 0; i < MAX_COLUMN_BYTES; i++)
   {
      for (j = 0; j < MAX_LINE_BYTES; j++)
      {
         Epaper_Write_Data(0xFF);
      }
   }

   Epaper_Write_Command(0x26);
   for (i = 0; i < MAX_COLUMN_BYTES; i++)
   {
      for (j = 0; j < MAX_LINE_BYTES; j++)
      {
         Epaper_Write_Data(0x00);
      }
   }
}

/*
 * 函数名：Display_All_Black
 * 描述  ：刷新显示全黑
 * 输入  ：无
 * 输出  ：无
 */
void Epaper_Load_Black(void)
{
   u32 i, j;
   Epaper_Write_Command(0x24);
   for (i = 0; i < MAX_COLUMN_BYTES; i++)
   {
      for (j = 0; j < MAX_LINE_BYTES; j++)
      {
         Epaper_Write_Data(0x00);
      }
   }
  
   Epaper_Write_Command(0x26);
   for (i = 0; i < MAX_COLUMN_BYTES; i++)
   {
      for (j = 0; j < MAX_LINE_BYTES; j++)
      {
         Epaper_Write_Data(0x00);
      }
   }
}

/*
 * 函数名：Display_All_Red
 * 描述  ：刷新显示全红
 * 输入  ：无
 * 输出  ：无
 */
void Epaper_Load_Red(void)
{
   u32 i, j;
   Epaper_Write_Command(0x24);
   for (i = 0; i < MAX_COLUMN_BYTES; i++)
   {
      for (j = 0; j < MAX_LINE_BYTES; j++)
      {
         Epaper_Write_Data(0x00);
      }
   }

   Epaper_Write_Command(0x26);
   for (i = 0; i < MAX_COLUMN_BYTES; i++)
   {
      for (j = 0; j < MAX_LINE_BYTES; j++)
      {
         Epaper_Write_Data(0xFF);
      }
   }
}

void Display_image_new(u8 *BW_DTM1,u8 *RW_DTM1)
{
	Epaper_Initial();
  Epaper_Load_Image((u8 *)BW_DTM1, (u8 *)RW_DTM1, ALLSCREEN_BYTES);
	Epaper_Update_and_Deepsleep();
}

void Display_Black( )
{
	Epaper_Initial();
	Epaper_Load_Black();
	Epaper_Update_and_Deepsleep();
}

void Display_White( )
{
	Epaper_Initial();
	Epaper_Load_White();
	Epaper_Update_and_Deepsleep();
}

void Display_Red( )
{
	Epaper_Initial();
	Epaper_Load_Red();
	Epaper_Update_and_Deepsleep();
}

 
int main()
{
	
EpaperIO_Init( );//dke test use
delay_init();	 //dke test use
Display_image_new((u8*)ESL_BW,(u8*)ESL_RED);
Display_Red();
Display_Black();
Display_White();

}
 
