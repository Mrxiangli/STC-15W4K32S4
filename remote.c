#include "STC15Wxx_STC15Fxx.h"
#include "intrins.h"
#include "math.h"
#include "LT89XX_DRV.h"
#define cycle 1390L
#define FOSC 11059200L
#define BAUD 115200
#define NONE_PARITY	0
#define ODD_PARITY 1
#define EVEN_PARITY 2
#define MARK_PARITY 3
#define SPACE_PARITY 4
#define PARITYBIT NONE_PARITY
#define S2RI 0x01
#define S2TI 0x02
#define FALSE 0
#define TRUE 1
#define TX_DAT1 0x01	 		
#define TX_DAT2 0x02     
#define TX_DAT3 0x04
#define TX_DAT4	0X05	
#define INT_RX  0x03     
#define RX_MOD  0x00     
#define FALSE 0
#define TRUE 1

//function declaration
void Delay_ms(unsigned short T);
void receive_data();

sbit test = P2^6;
unsigned char idata comp_buffer[8];
unsigned char flag = 0;
unsigned int data_count = 0;
unsigned char process_data = 0;
unsigned char receive_buffer[3];
unsigned char TX_buffer[8];
unsigned char top_buffer = 0;
unsigned char bom_buffer = 0;
unsigned char TX_status = 0;
unsigned char TX_index = 0;
unsigned char TX_number = 0;
unsigned char index_state;           
unsigned char idata RBUF[32];
unsigned char time_5ms_flag;
unsigned char RegH;
unsigned char RegL;              //Used to store the Registor Value which is read.

//Funtion Definitions
void SPI_WriteReg(unsigned char addr, unsigned char H, unsigned char L)
{
  int i;
  SPI_SS = 0;
  for(i = 0; i < 8; ++ i)
	{
	  MOSI = addr & 0x80;
	  SPI_CLK = 1;                       //capturing at the down side.
	  SPI_CLK = 0;
	  addr = addr << 1;                    //There is no Delay here. determines the rate of SPI.
	}

  for(i = 0; i < 8; ++i)                 //Write H
	{
	  MOSI = H & 0x80;
	  SPI_CLK = 1;
	  SPI_CLK = 0;
	  H = H << 1;
	}

  for(i = 0; i < 8; ++i)                 //Write L
	{  
	  MOSI = L & 0x80;
	  SPI_CLK = 1;
	  SPI_CLK = 0;
	  L = L << 1;
	}

   SPI_SS = 1;
}

void SPI_ReadReg(unsigned char addr)
{
  int i;
  SPI_SS = 0;
  addr += 0x80;                    //when reading a Register,the Address should be added with 0x80.
  for(i = 0; i < 8; ++ i)
	{
	  MOSI = addr & 0x80;
	  SPI_CLK = 1;
	  SPI_CLK = 0;
	  addr = addr << 1;                      //Move one bit to the left.
	}

  for(i = 0; i < 8; ++ i)
	{
	  SPI_CLK = 1;                         //transmit at the up side.
	  SPI_CLK = 0;
	  RegH = RegH << 1;  
	  RegH |= MISO;
	}
   	
  for(i = 0; i < 8; ++ i)
	{
	  SPI_CLK = 1;                         //transmit at the up side.
	  SPI_CLK = 0;
	  RegL = RegL << 1; 
	  RegL |= MISO;
	}
   SPI_SS = 1;
}

void LT8900_Init(void)
{

  RST  = 0;
	Delay_ms(2);
	RST  = 1;
	Delay_ms(5);
	PKT = 1;

	SPI_WriteReg( 0, 0x6f, 0xef );        ///6fe0
	SPI_WriteReg( 1, 0x56, 0x81 );
	SPI_WriteReg( 2, 0x66, 0x17 );
	SPI_WriteReg( 4, 0x9c, 0xc9 );
	SPI_WriteReg( 5, 0x66, 0x37 );
	SPI_WriteReg( 7, 0x00, 0x00 );				///0030			  //channel is 2402Mhz
	SPI_WriteReg( 8, 0x6c, 0x90 );
	SPI_WriteReg( 9, 0x48, 0x00 );			  //4800				  //PA -12.2dbm
	SPI_WriteReg(10, 0x7f, 0xfd );
	SPI_WriteReg(11, 0x00, 0x08 );
	SPI_WriteReg(12, 0x00, 0x00 );
	SPI_WriteReg(13, 0x48, 0xbd );
	SPI_WriteReg(22, 0x00, 0xff );
	SPI_WriteReg(23, 0x80, 0x05 );
	SPI_WriteReg(24, 0x00, 0x67 );
	SPI_WriteReg(25, 0x16, 0x59 );
	SPI_WriteReg(26, 0x19, 0xe0 );
	SPI_WriteReg(27, 0x13, 0x00 );
	SPI_WriteReg(28, 0x18, 0x00 );
	SPI_WriteReg(32, 0x58, 0x00 );//4800
	SPI_WriteReg(33, 0x3f, 0xc7 );
	SPI_WriteReg(34, 0x20, 0x00 );
	SPI_WriteReg(35, 0x0a, 0x00 );			//0300	    /* ?????9? ????10?*/
	SPI_WriteReg(36, 0x03, 0x80 );
	SPI_WriteReg(37, 0x03, 0x80 );
	SPI_WriteReg(38, 0x5a, 0x5a );
	SPI_WriteReg(39, 0x03, 0x80 );
	SPI_WriteReg(40, 0x44, 0x02 );//2102
	//SPI_WriteReg(41, 0xb4, 0x00 );	                /*CRC is ON; scramble is OFF; AUTO_ACK is OFF*/
	SPI_WriteReg(41, 0xb8, 0x00 );	    //b000                /*CRC is ON; scramble is OFF; AUTO_ACK is ON*/
    #ifdef LT8910
	SPI_WriteReg(42, 0xfd, 0xff );		.//fdb0			
    #else
	SPI_WriteReg(42, 0xfd, 0xb0 );				
	#endif
	SPI_WriteReg(43, 0x00, 0x0f );
}
	   
void Init_Timer(void)
{
  time_5ms_flag=FALSE; 
}

void timer2_isr(void)  interrupt 12
{
  time_5ms_flag=TRUE;  //??5MS????
}

void Delay_10us(unsigned char T )
{
  for(;T>0;T--)
  {
    TF0=0;
    TH0=(65536-18)/256; 
    TL0=(65536-18)%256; //6M  12T  10US
    TR0=1;
    while(TF0==0);
    TR0=0;
  }
}

void Delay_ms(unsigned short T)
{
  for(;T>0;T--)
  {
    Delay_10us(100);
  }
}

//uart initialization
void uart_ini()
{
  P_SW1 &= ~S1_S0;
	#if(PARITYBIT == NONE_PARITY)
  	 SCON = 0x50;
  #elif(PARITYBIT == ODD_PARITY)||(PARITYBIT == EVEN_PARITY)||(PARITYBIT == MARK_PARITY)
  	 SCON = 0xda;
  #elif(PARITYBIT == SPACE_PARITY)
  	 SCON = 0xd2;
  #endif
  T2L = (65536-(FOSC/4/BAUD));
  T2H = (65536-(FOSC/4/BAUD))>>8;
  AUXR = 0x14;
  AUXR |= 0x01;
  ES = 1;
  EA = 1;
}

void Uart() interrupt 4 using 1
{
//	unsigned char unTemp;
	if(process_data == 0)
	{
    if(SCON & SRI)
		{
			 SCON &= ~SRI;
			 if((SBUF == 0x55)||(flag==1))
			 {	
        receive_buffer[bom_buffer++] = SBUF;
			  if(bom_buffer==3)
			  {
				  bom_buffer = 0;
			  } 
			  data_count++;
			  flag = 1;
			  if(data_count==3)
			  {
			    process_data = 1;
				  flag = 0;
			  }
		  }
    }
	}		
  
  if(SCON & STI)
	  { 
      SCON &= ~STI;
			TX_index++;
			if(TX_index < TX_number)
				SBUF = TX_buffer[TX_index];
			else TX_status = 0;
    }
}

void send_data(unsigned char *p, unsigned char num)//frame
{
	unsigned char index;
	TX_index=0;
  for(index=0;index<num;index++)
	    {
		    TX_buffer[index] = *p;
		    p++;
	    }
	TX_number = num;
	SBUF = TX_buffer[0];
}	

void receive_data()
{
	if(receive_buffer[0] == 0x55)
	{
    if(receive_buffer[1] == 0x01)
		{
			if(receive_buffer[2] == 0x54)
			{
				index_state = TX_DAT1;
				process_data = 0;
				data_count = 0;
				receive_buffer[0]=0;
				receive_buffer[1]=0;
				receive_buffer[2]=0;
			}
		}		
   if(receive_buffer[1] == 0x02)
		{
			if(receive_buffer[2] == 0x54)
			{
				index_state = TX_DAT2;
				process_data = 0;
				data_count = 0;
				receive_buffer[0]=0;
				receive_buffer[1]=0;
				receive_buffer[2]=0;
			}
		}		
			if(receive_buffer[1] == 0x05)
		{
			if(receive_buffer[2] == 0x54)
			{
				index_state = TX_DAT4;
				process_data = 0;
				data_count = 0;
				receive_buffer[0]=0;
				receive_buffer[1]=0;
				receive_buffer[2]=0;
			}
		}		
	}
}

void main()
{
   unsigned char i=1,j;
   unsigned short count;
	 unsigned char data_count=0;
	 unsigned char data_flag=0;

   uart_ini();
   Init_Timer(); //initialize timer
   Delay_ms(10);
	 LT8900_Init();
	 #ifdef LT8910
	 SPI_WriteReg(44, 0x10, 0x00);
	 SPI_WriteReg(45, 0x05, 0x52);	
	 #endif
	 SPI_ReadReg(40);
	 if (RegH==0x44 && RegL==0x02)
	 {}
   count=0;
   index_state = INT_RX;
  while (1)
  {
  //index_state = 0;	
 //	receive_data();//from computer
	
	 switch(index_state)
   {
	  /////////////////////////////////////////////////////////
	  case INT_RX:
  	  SPI_WriteReg(52, 0x80, 0x80);
      SPI_WriteReg(7, 0x00, 0x80 + 0x20);	
	    index_state=RX_MOD;
	  break;

	  /////////////////////////////////////////////////////////	
    case RX_MOD:
	  //checking if receive the data
	  if(PKT == 1)
	  {
		  SPI_ReadReg(50);
		   //read data
		  SPI_ReadReg(50);
		  RBUF[0]=RegH;
		  RBUF[1]=RegL;
			SPI_ReadReg(50);
		  RBUF[2]=RegH;
		  RBUF[3]=RegL;
			SPI_ReadReg(50);
		  RBUF[4]=RegH;
		  RBUF[5]=RegL;
			SPI_ReadReg(50);
		  RBUF[6]=RegH;
		  RBUF[7]=RegL;
	    //Test CRC	
		  SPI_ReadReg(48);
		  if((RegH&0x80)==0)  
			//initialize to receive mode
		  // index_state = 12;
			 comp_buffer[0]=RBUF[0];
			 comp_buffer[1]=RBUF[1];
			 comp_buffer[2]=RBUF[2];
			 comp_buffer[3]=RBUF[3];
			 comp_buffer[4]=RBUF[4];
			 comp_buffer[5]=RBUF[5];
			 comp_buffer[6]=RBUF[6];
			 comp_buffer[7]=RBUF[7];
       send_data(comp_buffer, 8);	
       Delay_ms(20);	 	 
			}
		index_state = INT_RX;
			break;

		  /////////////////////////////////////////////////////////////////
			case TX_DAT1:
 		  //send the data
		   SPI_WriteReg(52, 0x80, 0x80);
		   SPI_WriteReg(50, 2,0);
		   SPI_WriteReg(50, 1,0);
		   SPI_WriteReg(7, 0x01, 0x20);
		   while (PKT == 0); //
       SPI_ReadReg(52);
		   if((RegH& 0x3F)==0)//
		   { 
		     count=0;
		   }  
			index_state = INT_RX;
			break;

			case TX_DAT2:
		    SPI_WriteReg(52, 0x80, 0x80);
		    i=0;
		   SPI_WriteReg(50, 2,0);
       SPI_WriteReg(50, 2,0);
		   SPI_WriteReg(7, 0x01, 0x20);
		   while (PKT== 0);      
           SPI_ReadReg(52);
		   if((RegH& 0x3F)==0)
		   	{
			   count=0;
		   	}
		   index_state=INT_RX;
			break;
				
				case TX_DAT4:
		    SPI_WriteReg(52, 0x80, 0x80);
		    i=0;
		   SPI_WriteReg(50, 2,0);
       SPI_WriteReg(50, 3,0);
		   SPI_WriteReg(7, 0x01, 0x20);
		   while (PKT== 0);      
           SPI_ReadReg(52);
		   if((RegH& 0x3F)==0)
		   	{
			   count=0;
		   	}
		   index_state=INT_RX;
			break;

		  default:break;
		}
	
	}
}
