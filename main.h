#include <CMS/CMS8S5880.H>
//*******************INIT_FUNCTION*********************//
void delay30ms();
void GPIO_Init();
void DanceLED();
void UART1_INIT();								//the BAUD relate the system clock
void INT_init();								//interrupt initial
void updateInputStates();
void sendKeyPadStates();
void initializeInputs();
void updateMeasurementBuffers();
void updateBufferSums();
void updateBufferIndex();
void ADC_init();
unsigned long Get_Channl_ADC_Val(long ADC_Channl);