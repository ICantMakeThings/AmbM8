//备注:用Source Insight软件浏览程序效果最佳	  

//xx Includes xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
#include "msp430x22x4.h"
#include "420_1.h"


//xx Private macro xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

#define SDA_H     	(P1OUT |=BIT7)						// P1.7
#define SDA_L     	(P1OUT &=~BIT7)
#define SCLK_H    	(P1OUT |=BIT6)  					// P1.6
#define SCLK_L   	(P1OUT &=~BIT6) 
#define nCS_H     	(P1OUT |=BIT5)						// P1.5
#define nCS_L     	(P1OUT &=~BIT5)
#define nDC_H     	(P1OUT |=BIT4)						// P1.4
#define nDC_L     	(P1OUT &=~BIT4)
#define nRST_H     	(P1OUT |=BIT3)						// P1.3 
#define nRST_L     	(P1OUT &=~BIT3)

#define R_SDA         0x80 	//P1.7

#define DELAY_TIME	1									// 图片显示完停留时间(单位:秒)

#define MODE1  											// panel scan direction

unsigned char tempvalue;
unsigned char temp1,temp2; 


#define PIC_BLACK		252
#define PIC_WHITE		255
#define PIC_A			1
#define PIC_B   	    2
#define PIC_HLINE		3
#define PIC_VLINE	    4
#define PIC_C			5
#define PIC_D   	    6
#define PIC_E		    7
#define PIC_R	        8
//xx Private functions xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//xx   延时函数    xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
void DELAY_100nS(int delaytime)   						// 30us 
{
	int i,j;
	
	for(i=0;i<delaytime;i++)
		for(j=0;j<1;j++);
}

void DELAY_mS(int delaytime)    						// 1ms
{
	int i,j;
	
	for(i=0;i<delaytime;i++)
		for(j=0;j<200;j++);
}

void DELAY_S(int delaytime)     						// 1s
{
	int i,j,k;
	
	for(i=0;i<delaytime;i++)
		for(j=0;j<1000;j++)
			for(k=0;k<200;k++);
}


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//xx   电子纸驱动操作函数    xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

// 复位 xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
void RESET()
{
	nRST_L;
	DELAY_mS(1);								
 	nRST_H;
  	DELAY_mS(1);
}

// 读忙 xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
void READBUSY()
{
  	while(1)
  	{
   		_NOP();
   	 	if((P1IN & 0x04)==0)
    		break;
  	}      
}

// 写命令 xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
void SPI4W_WRITECOM(unsigned char INIT_COM)
{
	unsigned char TEMPCOM;
	unsigned char scnt;
   P1DIR |= R_SDA;
	TEMPCOM=INIT_COM;
	nCS_H;
	nCS_L;
	SCLK_L;
	nDC_L;
	for(scnt=0;scnt<8;scnt++)
	{
		if(TEMPCOM&0x80)
			SDA_H;
		else
			SDA_L;
		SCLK_H;  
		SCLK_L;  
		TEMPCOM=TEMPCOM<<1;
	}
	//nCS_H;	
}

// 写数据 xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
void SPI4W_WRITEDATA(unsigned char INIT_DATA)
{
	unsigned char TEMPCOM;
	unsigned char scnt;
	P1DIR |= R_SDA;

	TEMPCOM=INIT_DATA;
	//nCS_H;
	//nCS_L;
	SCLK_L;
	nDC_H;
	for(scnt=0;scnt<8;scnt++)
	{
		if(TEMPCOM&0x80)
		SDA_H;
		else
		SDA_L;
		SCLK_H;  
		SCLK_L;  
		TEMPCOM=TEMPCOM<<1;
	}
	//nCS_H;	
}


// 读数据xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
 unsigned char SPI4W_READDATA1()
 {
	 P1DIR &=~ R_SDA;

	 unsigned char scnt,temp;
         temp=0;
         
	// nCS_H;
	// nCS_L;
	// SCLK_H;
	 nDC_H;
	 for(scnt=0;scnt<8;scnt++)
	 {
		 	  
		 SCLK_L;  
		 if(P1IN&R_SDA)
		 temp=(temp<<1)|0x01;
		 else
		 temp=temp<<1;		
		 SCLK_H;	  
		 SCLK_L;  
	 }
	// nCS_H;  
         tempvalue= temp;
	 return temp;
 }

void read_temperture(void)
 {
   //调用此函数前busy线必须为低
   unsigned char temp1,temp2;	
   SPI4W_WRITECOM(0x18);
   SPI4W_WRITEDATA(0X80);
     SPI4W_WRITECOM(0x22);
   SPI4W_WRITEDATA(0XB1); 
    SPI4W_WRITECOM(0x20);
      READBUSY();
   
   SPI4W_WRITECOM(0x1B);
    temp1=SPI4W_READDATA1(); 
 
   tempvalue=temp1;
	 nCS_H;  
 }


// 入深度睡眠 xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
void enterdeepsleep()
{

        SPI4W_WRITECOM(0x10);
  	SPI4W_WRITEDATA(0x01);
        nCS_H; 

}
// 电子纸驱动初始化 xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

void INIT_SSD1683()
{    
  
    SPI4W_WRITECOM(0x01);	 // Set Driver Output Control for Gate setting = Gate Channel - 1                                       
    SPI4W_WRITEDATA(0x2B);    // For example, for 300 gate channel => 300 - 1 = 299 (0x12B)	
    SPI4W_WRITEDATA(0x01);
    SPI4W_WRITEDATA(0x00);

    SPI4W_WRITECOM(0x11);		// Set Data Entry Mode setting
    SPI4W_WRITEDATA(0x01);          //Y decrement, X increment, address counter in X-direction
    
    SPI4W_WRITECOM(0x44);		// Set RAM X address
    SPI4W_WRITEDATA(0x00);		// start position = 0
    SPI4W_WRITEDATA(0x31);          //  end position = Source Channel/8 - 1 => 49(0x31)
    
    SPI4W_WRITECOM(0x45);		//Set RAM Y address 
    SPI4W_WRITEDATA(0x2B);		 //start position = 300 - 1 = 299 (0x12B) and
    SPI4W_WRITEDATA(0x01);		
    SPI4W_WRITEDATA(0x00);		//  end position = 0
    SPI4W_WRITEDATA(0x00);	
    
    SPI4W_WRITECOM(0x3C);		// board
    SPI4W_WRITEDATA(0x01);		// White: 0x01
    
    SPI4W_WRITECOM(0x21);		
    SPI4W_WRITEDATA(0x40);

    
}


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//xx   图片显示函数    xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
void dis_img(unsigned char num)
{
	unsigned int row, col;
	unsigned int pcnt;
/***************************************************bw image****************************************************/
        SPI4W_WRITECOM(0x4E);	 //Set RAM X address counter = 0
        SPI4W_WRITEDATA(0x00);   
        SPI4W_WRITECOM(0x4F);	//Set Driver Output Control for Gate setting = Gate Channel - 1	
        SPI4W_WRITEDATA(0x2B);  //For example, for 300 gate channel => 300 - 1 = 299 (0x12B)
        SPI4W_WRITEDATA(0x01);
	SPI4W_WRITECOM(0x24);   //Write B/W image data into to Register 0x24 RAM
	
        pcnt = 0;											// 复位或保存提示字节序号
	for(col=0; col<300; col++)							// 总共172列		// send 128x252bits ram 2D13
	{
		for(row=0; row<50; row++)						// 总共72行，每个像素2bit,即 72/4 字节
		{
				switch (num)
				{

				case PIC_WHITE:
					SPI4W_WRITEDATA(0xff);
					break;	

				case PIC_BLACK:
					SPI4W_WRITEDATA(0x00);
					break;	
			   	default:
					break;
				}
			pcnt++;
		}
	}
             

        
        SPI4W_WRITECOM(0x22);
	SPI4W_WRITEDATA(0xF7);		//Load LUT from MCU(0x32), Display update
	SPI4W_WRITECOM(0x20);
	DELAY_mS(1);
	READBUSY();
	

} 



//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//xx   主函数    xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
void main( void )
{
    int i;

	WDTCTL = WDTPW + WDTHOLD;							// Stop watchdog timer to prevent time out reset
	BCSCTL1 = CALBC1_8MHZ; 								// set DCO frequency 1MHZ
	DCOCTL = CALDCO_8MHZ;

	P1DIR |=0x78;  										// set P1.3~7 output
	P3OUT = 0XFF;
        P3DIR = 0xff;										// p3.4,p3.5 设为输入
        P4DIR |= 0X01;

		                RESET();								
				SPI4W_WRITECOM(0x12);			
				READBUSY(); 
				INIT_SSD1683();
				dis_img(PIC_WHITE);
				enterdeepsleep();
				DELAY_S(480);
                                _NOP();

}
