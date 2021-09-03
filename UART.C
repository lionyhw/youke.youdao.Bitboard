#include <main.h>
//*****************************************************************************************//
//**********************************VER 1.0 data:20181227**********************************//
//**********************************    author : fred    **********************************//
//*********************************** UART  send&receive***********************************//
//***************** receive 10 sample data and then send the data**************************//
//****************************reveive data and then send data *****************************//
//*****************************************************************************************//

//************************UART**************************//
bit	F_transmit = 1;
//****************************ADC************************//
bit ADC_Transformation_End=0;
//***************************GPIO_FUNCTION****************//
#define __bit(x)	(1<<(x))
#define SET_IO_DIR_OUT(a,p) P##a##TRIS |= __bit(p)
#define SET_IO_DIR_IN(a,p) P##a##TRIS &= ~__bit(p)
#define SET_IO_OD(a,p) P##a##OD |= __bit(p)
#define SET_IO_PP(a,p) P##a##OD &= ~__bit(p)
#define EN_IO_UP(a,p) P##a##UP |= __bit(p)
#define DE_IO_UP(a,p) P##a##UP &= ~__bit(p)
#define   pin0  0x01;
#define   pin1  0x02;
#define   pin2  0x04;
#define   pin3  0x08;
#define   pin4  0x10;
#define   pin5  0x20;
#define   pin6  0x40;
#define   pin7  0x80;
//*******************GPIO_Pin*********************//
#define UP P3_2
#define UP_LED P3_6
#define LEFT P3_1
#define LEFT_LED P3_0
#define DOWN P2_4
#define DOWN_LED P2_1
#define RIGHT P2_6
#define RIGHT_LED P2_5
#define SPACE P2_3
#define SPACE_LED P1_7
#define CLICK	P2_2
#define CLICK_LED	P1_6
#define A P1_3
#define A_LED P0_4
#define B P1_4
#define B_LED P1_5

/////////////////////////////////
///////KeyCodes//////////////////
/////////////////////////////////
u8 CMDSendFlag = 0;
u8 MousePressd = 0;
u8 KeyPadADCChannel[8]={0xBE,0xBA,0xBD,0xBC,0xB9,0xB7,0xB6,0xB8};
u8 KeyPadCMD[14]={0x57,0xAB,0x00,0x02,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
u8 KeyPadCode[7]={0x52,0x51,0x50,0x4F,0x2C,0x05,0x04};   //up down left right space B  A
u8 KeyPadCMDNum[6]={7,8,9,10,11,12};
u8 MousePress[13]		={0x57,0xab,0x00,0x04,0x07,0x02,0x01,0x00,0x00,0x00,0x00,0x00,0x10};
u8 MouseRelease[13]	={0x57,0xab,0x00,0x04,0x07,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x0F};

///////////////////////////////Suanfa//////////////////////////////
#define SWITCH_THRESHOLD_OFFSET_PERC 15
#define SWITCH_THRESHOLD_CENTER_BIAS 50      
#define BUFFER_LENGTH    3     // 3 bytes gives us 24 samples
u8 pressThreshold = 0;
u8 releaseThreshold = 0;
u8 bufferIndex = 0;
u8 byteCounter = 0;
u8 bitCounter = 0;
typedef struct {
	u8 measurementBuffer[3]; 
	bool oldestMeasurement;
	bool pressed;
	u8 bufferSum;
} MakeyMakeyInput;

MakeyMakeyInput inputs[8];
void delay50us(void)   //?? 0us
{
    unsigned char a,b;
    for(b=239;b>0;b--)
        for(a=1;a>0;a--);
}


//*******************MAIN*********************//
void main()
{
	GPIO_Init();						//initinal GPIO
	DanceLED();
	UART1_INIT();						//initinal UART
	INT_init();	
	ADC_init();
	initializeInputs();
	while(1)
	{
		clrwdt()								//clear WDT
		updateMeasurementBuffers();
		updateBufferSums();
		updateBufferIndex();
		updateInputStates();
		if(CMDSendFlag){
			sendKeyPadStates();
			CMDSendFlag=0;
		}
delay50us();
	}
}
///////////////////////////
// INITIALIZE INPUTS
///////////////////////////
void initializeInputs() {
	u8 i =0,j = 0;
  float thresholdPerc = SWITCH_THRESHOLD_OFFSET_PERC;
  float thresholdCenterBias = SWITCH_THRESHOLD_CENTER_BIAS/50.0;
  float pressThresholdAmount = (BUFFER_LENGTH * 8) * (thresholdPerc / 100.0);
  float thresholdCenter = ( (BUFFER_LENGTH * 8) / 2.0 ) * (thresholdCenterBias);
  pressThreshold = thresholdCenter + pressThresholdAmount;
  releaseThreshold = thresholdCenter - pressThresholdAmount;


  for (i=0; i<8; i++) {
    for (j=0; j<3; j++) {
      inputs[i].measurementBuffer[j] = 0;
    }
    inputs[i].oldestMeasurement = 0;
    inputs[i].bufferSum = 0;
    inputs[i].pressed = 0;
  }
}

//////////////////////////////
// UPDATE MEASUREMENT BUFFERS
//////////////////////////////
void updateMeasurementBuffers() {
u8 i=0;
	char newMeasurement = 0;
  for (i=0; i<8; i++) {

    // store the oldest measurement, which is the one at the current index,
    // before we update it to the new one 
    // we use oldest measurement in updateBufferSums
    u8 currentByte = inputs[i].measurementBuffer[byteCounter];
    inputs[i].oldestMeasurement = (currentByte >> bitCounter) & 0x01; 

    // make the new measurement
		newMeasurement = ((Get_Channl_ADC_Val(KeyPadADCChannel[i]) < 1200) ? 0 : 1);


    // invert so that true means the switch is closed
    newMeasurement = !newMeasurement; 

    // store it    
    if (newMeasurement) {
      currentByte |= (1<<bitCounter);
    } 
    else {
      currentByte &= ~(1<<bitCounter);
    }
    inputs[i].measurementBuffer[byteCounter] = currentByte;
  }
}
///////////////////////////
// UPDATE BUFFER SUMS
///////////////////////////
void updateBufferSums() {
u8 i = 0;
  // the bufferSum is a running tally of the entire measurementBuffer
  // add the new measurement and subtract the old one

  for (i=0; i<8; i++) {
    u8 currentByte = inputs[i].measurementBuffer[byteCounter];
    bool currentMeasurement = (currentByte >> bitCounter) & 0x01; 
    if (currentMeasurement) {
      inputs[i].bufferSum++;
    }
    if (inputs[i].oldestMeasurement) {
      inputs[i].bufferSum--;
    }
  }  
}

///////////////////////////
// UPDATE BUFFER INDEX
///////////////////////////
void updateBufferIndex() {
  bitCounter++;
  if (bitCounter == 8) {
    bitCounter = 0;
    byteCounter++;
    if (byteCounter == 3) {
      byteCounter = 0;
    }
  }
}
///////////////////////////
// UPDATE INPUT STATES
///////////////////////////
void updateInputStates() {
	u8 i = 0;

  for (i=0; i<8; i++) {
    if (inputs[i].pressed) {
      if (inputs[i].bufferSum < releaseThreshold) {  
        inputs[i].pressed = 0;
        KeyPadCMD[KeyPadCMDNum[i]] = 0;
				CMDSendFlag = 1;
				if(i == 0){
					UP_LED = 1;
				}
				else if(i == 1){
					DOWN_LED = 1;
				}
				else if(i == 2){
					LEFT_LED = 1;
				}
				else if(i == 3){
					RIGHT_LED = 1;
				}
				else if(i == 4){
					SPACE_LED = 1;
				}
				else if(i == 5){
					B_LED = 1;
				}
				else if(i == 6){
					A_LED = 1;
					KeyPadCMD[KeyPadCMDNum[i-1]] = 0;
				}
				else if(i == 7){
					CLICK_LED = 1;
					MousePressd = 0;
				}
      }
    } 
    else if (!inputs[i].pressed) {
      //if(inputs[i].bufferSum > pressThreshold) {  // input becomes pressed
			if(inputs[i].bufferSum > 18) {  // input becomes pressed
        inputs[i].pressed = 1; 
        KeyPadCMD[KeyPadCMDNum[i]] = KeyPadCode[i];
				CMDSendFlag = 1;
				if(i == 0){
					UP_LED = 0;
				}
				else if(i == 1){
					DOWN_LED = 0;
				}
				else if(i == 2){
					LEFT_LED = 0;
				}
				else if(i == 3){
					RIGHT_LED = 0;
				}
				else if(i == 4){
					SPACE_LED = 0;
				}
				else if(i == 5){
					B_LED = 0;
				}
				else if(i == 6){
					A_LED = 0;
					KeyPadCMD[KeyPadCMDNum[i-1]] = KeyPadCode[i];
				}
				else if(i == 7){
					CLICK_LED = 0;
					MousePressd = 1;
				}
      }
    }
  }
	
}
void sendKeyPadStates(){
	u8 i = 0;
	KeyPadCMD[13] = 0x0C+KeyPadCMD[5]+KeyPadCMD[6]+KeyPadCMD[7]+KeyPadCMD[8]+KeyPadCMD[9]+KeyPadCMD[10]+KeyPadCMD[11]+KeyPadCMD[12];
	for(i=0;i<14;i++){
		while(F_transmit == 0);
		if(F_transmit == 1)						//UART transmit data
		{
			F_transmit	= 0;
			SBUF1	= KeyPadCMD[i];
		}
	}
	

	for(i=0;i<13;i++){
		while(F_transmit == 0);
		if(F_transmit == 1)						//UART transmit data
		{
			F_transmit	= 0;
			if(MousePressd){
				SBUF1	= MousePress[i];
			}
			else{
				SBUF1	= MouseRelease[i];
			}
		}
	}
 }
		
	
void DanceLED(void)
{
	u8 i = 0;
	for(i= 0 ;i<3;i++){
		UP_LED = 0;
		delay30ms();
		UP_LED = 1;

		LEFT_LED = 0;
		delay30ms();
		LEFT_LED = 1;

		DOWN_LED = 0;
		delay30ms();
 		DOWN_LED = 1;     

		RIGHT_LED = 0;
		delay30ms();
		RIGHT_LED = 1;
	}
	for(i = 0;i<3;i++){
		SPACE_LED = 0;
		delay30ms();
		SPACE_LED = 1;

		CLICK_LED = 0;
		delay30ms();
		CLICK_LED     = 1;
	}
	for(i = 0;i<3;i++){ 
		A_LED = 0;
		B_LED = 0;
		delay30ms();
		delay30ms();

		A_LED = 1;
		B_LED = 1;
		delay30ms();
		delay30ms();
 	}

}
void delay30ms(void)   //
{
    u16 a,b;
    for(b=500;b>0;b--)
        for(a=300;a>0;a--){
					_nop_();  //if Keil,require use intrins.h
					_nop_();  //if Keil,require use intrins.h
					_nop_();  //if Keil,require use intrins.h
					_nop_();  //if Keil,require use intrins.h
				}
}

void GPIO_Init(void)
{
	SET_IO_DIR_OUT(3,6);    //UP_LED
	SET_IO_PP(3,6);
	P3_6 = 1;
	
	SET_IO_DIR_OUT(3,0);    //LEFT_LED
	SET_IO_PP(3,0);
	P3_0 = 1;
	
	SET_IO_DIR_OUT(2,1);    //DOWN_LED
	SET_IO_PP(2,1);
	P2_1 = 1;
	
	SET_IO_DIR_OUT(2,5);    //RIGHT_LED
	SET_IO_PP(2,5);
	P2_5 = 1;
	
	SET_IO_DIR_OUT(1,7);    //SPACE_LED
	SET_IO_PP(1,7);
	P1_7 = 1;
	
	SET_IO_DIR_OUT(1,6);    //CLICK_LED
	SET_IO_PP(1,6);
	P1_6 = 1;
	
	SET_IO_DIR_OUT(0,4);    //A_LED
	SET_IO_PP(0,4);
	P0_4 = 1;
	
	SET_IO_DIR_OUT(1,5);    //B_LED
	SET_IO_PP(1,5);
	P1_5 = 1;
	
	SET_IO_DIR_IN(3,2);    //UP
	SET_IO_DIR_IN(2,4);    //DOWN
	SET_IO_DIR_IN(3,1);    //LEFT
	SET_IO_DIR_IN(2,6);    //RIGHT
	SET_IO_DIR_IN(2,3);    //SPACE
	SET_IO_DIR_IN(2,2);    //CLICK
	SET_IO_DIR_IN(1,3);    //A
	SET_IO_DIR_IN(1,4);    //B
}
/*************************************************
  function name: ADC_INT()
  function work: ADC INT sub function
  input data   : null
  output data  : null
  note     : interrupt function and set the CMDSendFlag
*************************************************/
void ADC_INT()    interrupt ADC_VECTOR
{
  EIF2    &=  0xEF;
  ADC_Transformation_End  = 1;
}
/////////////////////////////////////ADC////////////////////////////////////////
unsigned long Get_Channl_ADC_Val(long ADC_Channl) {
  //******volt_AN is the voltage of the input channel*****//
  //******the data divide 1000 is the real voltage *******//
  int i = 0;

  unsigned long result_ADC_CH;

  ADCON1  = ADC_Channl;         //AN channel switch
	ADCON0      |= 0x02;        //start AD converter
	while (ADC_Transformation_End == 0);
	if (ADC_Transformation_End == 1)
	{
      ADC_Transformation_End  = 0;
			result_ADC_CH = (ADRESH << 8) + ADRESL; //read the convert result
	}
	return result_ADC_CH;
}
/*************************************************
  function name: ADC_init()
  function work: initial the ADC function
  input data   : null
  output data  : null
  note     : setting the ADC function
*************************************************/
void ADC_init()
{
  //=======e.g. if use AN0+AN3,in the sample port THE value is pin0+pin3========//

  P32CFG    = 0x01;
	P31CFG    = 0x01;
	P24CFG    = 0x01;
	P26CFG    = 0x01;
	
	P23CFG    = 0x01;
	P22CFG    = 0x01;
	P13CFG    = 0x01;
	P14CFG    = 0x01;

  //==========================other register configure=========================//

  ADCON2    = 0x00;

  //bit7 0:disable HW trigger 1:enable HW trigger
  //bit6 0:external trigger pin selected ADCET0 1:external trigger pin selected ADCET1
  //bit5~4 ADC HW trigger source select
  //     00:PG0(PWM0) 01:PG2 (PWM2) 10:PG4 (PWM4) 11:ADCET0/ADCET1
  //bit3~2 ADC HW trigger edge select
  //     00:falling edge 01:rising edge 10:PWM period middle 11:PWM period end

  ADCON1    = 0x00;

  //bit7  0:disable ADC IP  1:enable ADC IP
  //bit6~4:ADCKS<2:0> ADC clock select
  //    000:f/2 001:f/4 010:f/8 011:f/16  100:f/32 101:f/64 110:f/128 111:f/256
  //bit3~0:ADCHS<3:0>: AN channel select
  //0000~1110:AN0~AN14    1111:AN15 reference for ADCON0.AN15SEL define

  ADCON0    = 0x40;

  //bit7 1: AN channel>=16 0: AN channel<16
  //bit6 0:left align 1:right align
  //bit5~4 AN15 select source bit   00:VBG 01:OP0_O 10:OP1_O
  //bit1 ADGO   0:can not write 1:start AD converter

  ADCMPC    = 0x00;

  //bit7 0:disable ADC CMP pwm fb function 1:enable
  //bit6 0:ADRES>=ADCMP,ADCMPO=1;1:ADRES<ADCMP,ADCMPO=1
  //bit4 ADC CMP output bit 0:CMP output 0 1:CMP output 1
  //bit1~0 ADC HW trigger delay time bit9~8


  //the ADC result
  //ADRESH/ADRESL
  //ADFM=0 ADRESH:ADRES11~ADRES4 ADRESL7~4:ADRES3~ADRES0
  //ADFM=1 ADRESH3~0:ADRES11~ADRES8 ADRESL:ADRES7~ADRES0

  //the CMP register
  //ADCMPH:D11~D4 ADCMPL BIT3~0:D3~D0

  //  ADCON0    |=  0x02;     //start AD converter

  //*****************************************************************//

}
/*************************************************
function name: UART1_INIT()
function work: initial the UART function
input data   : null
output data  : null
note		 : setting the UART1 function
*************************************************/
void UART1_INIT()								//BPS
{
	
	P05CFG	= 0x02; 		   				//enable TX pin
	
	SCON1 	= 0x40;							//UART 8bit asyn transmit enable
	
//Bit7~6	UnSM0~UnSM1 00:master synchronize 01:8 bit async baud rate veriable timer1/4
//						10:9bit async baud rate fsys/32 or fsys/64 11:9 bit async baud rate veriable
//Bit5	UnSM2	Multi-computer Communication control bit 0:disable 1:enable
//Bit4	UnREN	receive enable bit 0:disable 1:enable
//Bit3	UnTB8	trandmit 9-th data bit 0:bit=0 1:bit=1
//Bit2	UnRB8	receive 9-th data bit 0:bit=0 1:bit=1
//Bit1	TIn	transmit interrupt (SW clear)	1:transmit buffer is null ,can transmit the next data
//Bit0	RIn	receive interrupt (SW clear)	1:receive buffer is full ,after read,can receive the next data

	PCON	&= 0x7f;						//baud is the define rate
	PCON	|= 0x40;
	
	FUNCCR 	|= 0x2A;						//clock from BRT
	
	BRTCON	=  0x80;
	
#define value_set	65380

	BRTDL	=  value_set;
	BRTDH	=  value_set/256;
	
}
/*************************************************
function name: UART1_INT()
function work: UART INT function
input data   : null
output data  : null
note		 : set the send and receive the data CMDSendFlag
*************************************************/
void UART1_INT()	interrupt	UART1_VECTOR
{

	if(SCON1&0x02)						//transmit over
	{
		SCON1		&=	0x0FD;
		F_transmit	=	1;
	}
}

void INT_init()									//intterrupt initinal
{
	IE		=	0x40;
	
//Bit7	EA	Global interrupt enable bit 0:disable 1:enable
//Bit6	ES1	UART1 interrupt enable bit  0:disable 1:enable
//Bit5	ET2	Timer2 global interrupt enable bit  0:disable 1:enable
//Bit4	ES0	UART1 interrupt enable bit  0:disable 1:enable
//Bit3	ET1	Timer1 interrupt enable bit  0:disable 1:enable
//Bit2	EX1	INT1 interrupt enable bit  0:disable 1:enable
//Bit1	ET0	Timer0 interrupt enable bit  0:disable 1:enable
//Bit0	EX0	INT0 interrupt enable bit  0:disable 1:enable
	
	IP		=	0x10;
	
	//Bit6	PS1		UART1 interrupt priority control bit 0:high priority 1:low priority
	//Bit5	PT2		Timer2 interrupt priority control bit 0:high priority 1:low priority
	//Bit4	PS0		UART1 interrupt priority control bit 0:high priority 1:low priority
	//Bit3	PT1		Timer1 interrupt priority control bit 0:high priority 1:low priority
	//Bit2	PX1		INT1 interrupt priority control bit 0:high priority 1:low priority
	//Bit1	PT0		Timer0 interrupt priority control bit 0:high priority 1:low priority
	//Bit0	PX0		INT0 interrupt priority control bit 0:high priority 1:low priority
	
	EIE2 = 0x10;	

  //Bit7  SPIIE SPI enable interrupt bit 0:disable 1:enable
  //Bit6  I2CIE I2C enable interrupt bit 0:disable 1:enable
  //Bit5  WDTIE WDT enable interrupt bit 0:disable 1:enable
  //Bit4  ADCIE ADC enable interrupt bit 0:disable 1:enable
  //Bit3  PWMIE PWM enable interrupt bit 0:disable 1:enable(EIP2.PPWM must be the same as EIP1.PP3PWM)
  //Bit1  ET4  Timer4 enable interrupt bit 0:disable 1:enable
  //Bit0  ET3  Timer3 enable interrupt bit 0:disable 1:enable

	EA		=	1;		//Bit7	EA	Global interrupt enable bit 0:disable 1:enable
	
}