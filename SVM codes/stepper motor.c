#include<REGX52.h>
#include<intrins.h>
sbit CLK  = P1^0;
sbit ENA  = P1^1; 
sbit DIR  = P1^2;          

//unsigned int Tspeed[3]={20,4000,10000};        //?????
//unsigned int StepAngle[4]={200,200,200,200};   //?????--90?/180?/360?/720?
bit CY_status=0;
unsigned int CYCLE=20;//????
unsigned int cy_count=0;
unsigned PWM_count=0;
unsigned time1=0;
 
void Init_time();
void Key_scan();
void Step(unsigned int step_count);
void UartInit();

void main()
{
    ENA=0;         //??????
    CLK=0;         //??????????
    DIR=0;         //??????
    Init_time();   //?????? 
	  UartInit();	   //?????
	Step(20);
	while(1)
	{
  while(RI==0);   //????????
    P2_0=0;
   	RI=0;      //????????
	 Step(185);  //??????
	  //Delay1000ms();
    P2_0=~P2_0;
	}
}
void Init_time()                 
{
    TMOD |= 0x01;
    TH0=(65536-CYCLE)/256;       //?????65536-cycle??????
    TL0=(65536-CYCLE)%256;
                IE = 0x82;
                TR0=1;                       //?????????
}
void Enter_Timer0() interrupt 1             //???0??
{
    TH0=(65536-CYCLE)/256; 
    TL0=(65536-CYCLE)%256;
    CLK=~CLK;
    PWM_count++;
    if(PWM_count==(2*cy_count)&&CY_status)
    {
        PWM_count=0;
        TR0=0;
        ENA=0;
    }   
}



void Step(unsigned int step_count)//????????
{
    PWM_count=0;
    CY_status=1;
    cy_count=step_count;
    ENA=1;
    TR0=1;
}

void UartInit()		//?????
{
	PCON &= 0x7F;		//??????
	SCON = 0x50;		//8?
	TMOD &= 0x0F;		//?????1
	TMOD |= 0x20;		//?????1????
	TL1 = 0xFD;			//??
	TH1 = 0xFD;			//??		
	TR1 = 1;			//?????1
	ET1 = 0;        	//????
	EA=1;				//???
	ES=0;				//????
}
