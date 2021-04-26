#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

int adc_value = 0;
double referans_sicaklik = 0.0;
double main_adc = 0.0;
double sicaklik = 0.0;
double overshoot = 0.0;
int button_counter = 0;

void MS_Delay(uint32_t time)
{
	for(uint32_t i = 0; i < time *6250; i++);
}

void EXTI_Config()
{
  GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	//Clock settings
	//**************
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);          //GPIOA clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);        //SYSCFG clock enable
	//**************
	//Interrupt Settings -->Button (Gas Button)
	//**************
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;               					//GPIOA Pin2 Select
	GPIO_InitStruct.GPIO_Mode =  GPIO_Mode_IN;					 					//GPIOA Pin2 Input
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;          					//GPIOA Pin2 Push-Pull
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;        					//GPIOA Pin2 Nopull
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;      					//GPIOA Pin2 Very high speed
	GPIO_Init(GPIOA,&GPIO_InitStruct);									 					//Load the configuration

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,GPIO_PinSource0); 	//EXTI_Line Config Pin 2
  EXTI_InitStruct.EXTI_Line = EXTI_Line0;                  			//Line 2 selected. In STM32F4 Pin 2 connected to line 2
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;                    		//Line enable selected. (When interrupt comes)
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;							//Mode interrupt selected
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;       		//Trigger rising mode selected
  EXTI_Init(&EXTI_InitStruct);                               		//Load the configuration

	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;              		//NVIC EXTI 2 channel selected
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;              		//Channel enable (When interrupt comes)
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;        //Priority set 0
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStruct);                                 //Load the configuration
	//**************
}

void EXTI0_IRQHandler()                      //EXTI Interrupt Handler function
{
		if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
    if(button_counter == 0)
     {
			button_counter = 1;
			adc_value = 1500;
     }
     else if(button_counter == 1)
     {
     button_counter = 2;
     adc_value = 4000;
	}


     else
     {
    	 button_counter = 0;
     }

	}
		EXTI_ClearITPendingBit(EXTI_Line0);
}






void LEDS_Init()
{
   GPIO_InitTypeDef GPIO_InitStruct;

	/* Enable clock for GPIOD */
	//***********
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	//***********

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;				   //GPIOD Pin 0 select
	GPIO_InitStruct.GPIO_Mode =  GPIO_Mode_OUT;            //GPIOD Pin 0 output
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			   //GPIOD Pin 0 Push-Pull
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;		   //GPIOD Pin 0 Nopull
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;        //GPIOD Pin 0 very high speed

	GPIO_Init(GPIOD,&GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;				   //GPIOD Pin 1 select
	GPIO_InitStruct.GPIO_Mode =  GPIO_Mode_OUT;            //GPIOD Pin 1 output
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			   //GPIOD Pin 1 Push-Pull
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;		   //GPIOD Pin 1 Nopull
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;        //GPIOD Pin 1 very high speed

	GPIO_Init(GPIOD,&GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;				   //GPIOD Pin 2 select
	GPIO_InitStruct.GPIO_Mode =  GPIO_Mode_OUT;            //GPIOD Pin 2 output
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			   //GPIOD Pin 2 Push-Pull
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;		   //GPIOD Pin 2 Nopull
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;        //GPIOD Pin 2 very high speed

	GPIO_Init(GPIOD,&GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;				   //GPIOD Pin 3 select
	GPIO_InitStruct.GPIO_Mode =  GPIO_Mode_OUT;            //GPIOD Pin 3 output
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			   //GPIOD Pin 3 Push-Pull
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;		   //GPIOD Pin 3 Nopull
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;        //GPIOD Pin 3 very high speed

	GPIO_Init(GPIOD,&GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;				   //GPIOD Pin 4 select
	GPIO_InitStruct.GPIO_Mode =  GPIO_Mode_OUT;            //GPIOD Pin 4 output
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			   //GPIOD Pin 4 Push-Pull
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;		   //GPIOD Pin 4 Nopull
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;        //GPIOD Pin 4 very high speed

	GPIO_Init(GPIOD,&GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;				   //GPIOD Pin 5 select
	GPIO_InitStruct.GPIO_Mode =  GPIO_Mode_OUT;            //GPIOD Pin 5 output
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			   //GPIOD Pin 5 Push-Pull
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;		   //GPIOD Pin 5 Nopull
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;        //GPIOD Pin 5 very high speed

	GPIO_Init(GPIOD,&GPIO_InitStruct);

}

double K = 1, zeta = 1.261, wn = 0.674;
double stepSize = 0.2;

double eq1(double y1, double y2, double u)
{
 return y2;
}

double eq2(double y, double y1, double u)
{
 return (K*wn*wn*u)-(2*zeta*wn*y)-(wn*wn*y1);
}


double ExplicitEulerEq1(double (*f)(double,double,double), double y2, double ts,
double uk)
{
 static double yk = 0;
 double yk1 = yk + ts * f(yk, y2, uk);
 yk = yk1;
 return yk1;
}

double ExplicitEulerEq2(double (*f)(double,double,double), double y1, double ts,
double uk)
{
 static double yk = 0; // Previous value of the output
 double yk1 = yk + ts * f(yk, y1, uk);
 yk = yk1;
 return yk1;
}

// PID parameters
volatile double Kp = 2.8, Ki = 0.8, Kd = 0.0;    //For PI controller scenerio Kp = 2.8, Ki = 0.8, Kd = 0.0;
                                                 //For P controller scenerio Kp = 2.5, Ki = 0.0, Kd =0.0;
// PID Controller

volatile double stepsize = 0.2;

double derivative_pid(double ek, double ek_1)
{
 return (ek - ek_1) / stepsize;
}
// Trapezoidal integration
double integral_pid(double ek, double ek_1)
{
 return (ek + ek_1) * (stepsize / 2);
}

double PID(double r, double y)
{
 static double ek_1 = 0, integralSum = 0;
 double outputPID;
 double ek = r - y;
 integralSum += integral_pid(ek, ek_1);
 outputPID = Kp * ek + Ki * integralSum + Kd * derivative_pid(ek, ek_1);
 ek_1 = ek;
 return outputPID;
}

void ADC_Config()
{
	GPIO_InitTypeDef GPIO_InitStruct;                    //GPIO init struct defined for pin.
	ADC_InitTypeDef ADC_InitStruct;                      //ADC init struct defined.
	ADC_CommonInitTypeDef ADC_CommonInitStruct;          //ADC common init struct defined.

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //Port A clock opened, I used Port A pin 0.
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);  //ADC clock opened.


	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;            //For adc usage, analog mode chosen.
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;               //Pin 0 chosen.
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;          //Output type set as push pull.
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;        // No pull configuration set.
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;      // Full speed
  GPIO_Init(GPIOA, &GPIO_InitStruct);                  //Load the configuation

	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;   //Independent mode chosen, (we are going to read only 1 channel).
	ADC_CommonInitStruct.ADC_Prescaler =  ADC_Prescaler_Div4; //Div 4 chosen for clock, APB2 line divided to 4.

  ADC_CommonInit(&ADC_CommonInitStruct);               //Load the configuration

	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;    //Resolution set 12bit
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;        //For contiunuous analog read it's set to enable.
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;   //We don't need a trigger so none selected
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Left;    //Data aligned to left.


	ADC_Init(ADC1,&ADC_InitStruct);     //Configuration loaded

	ADC_Cmd(ADC1,ENABLE);               //ADC start command.

}

uint32_t read_ADC()
{
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,1,ADC_SampleTime_56Cycles);

	ADC_SoftwareStartConv(ADC1);

	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC == RESET));

	return ADC_GetConversionValue(ADC1);

}


void CONFIG_USART3()
{
    GPIO_InitTypeDef GPIO_InitStructure;                      					//GPIO_InitTypeDef for USART pins.
	USART_InitTypeDef USART_InitStructure;                    					//USART_InitTypeDef definition.

		// USART IO Settings
	  //**************
	   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);                //USART3 clock enable.

	   GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                         //Alternate function mode chosen.
	   GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;              //I have used port D, pins 8 and 9 for USART.
	   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                           //Output type chosen as push-pull.
	   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                       // Pull up chosen for USART.
	   GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;                 //50Mhz speed chosen.
	   GPIO_Init(GPIOD, &GPIO_InitStructure);                              //Load the configuration

	   GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);           //Alternate function config.
	   GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);           //Alternate function config.
	  //**************

       // USART Settings
   	   //**************
	   USART_InitStructure.USART_BaudRate = 19200;                          //Baudrate chosen as 19200.
	   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //We are not using any interrupts or hardware flow.
	   USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;      //Receive and transmit mode chosen.
	   USART_InitStructure.USART_Parity = USART_Parity_No;                  //No parity bits.
	   USART_InitStructure.USART_StopBits = USART_StopBits_1;               //1 stop bit selected.
	   USART_InitStructure.USART_WordLength = USART_WordLength_8b;          //Word lenght 8 bits.
	   USART_Init(USART3, &USART_InitStructure);                            //Load the configuration

		USART_Cmd(USART3, ENABLE);                                           //USART Start.
}


void Config_I2C()
{
 GPIO_InitTypeDef  GPIO_InitStructure;                              //GPIO_InitTypeDef for I2C pins.
 I2C_InitTypeDef I2C_InitStructure;                                 //I2CInitTypeDef definition.

 RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);               //I2C1 clock enable.
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);              //GPIOB clock enable

 //I2C IO Settings
 //**************
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;            //I have used port B pins 6,7 for I2C
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                      //Alternate function mode chosen.
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                 //50Mhz speed chosen.
 GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;                    //For I2C open drain selected.
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                      //Pull Up mode selected.
 GPIO_Init(GPIOB, &GPIO_InitStructure);                            //Load the configuration.

 GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);           //Alternate function config.
 GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);           //Alternate function config.
 //**************

 //I2C Settings
 //**************
 I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;                        //I2C mode selected.
 I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;                //Duty cycle chosen as 2.
 I2C_InitStructure.I2C_OwnAddress1 = 0x00;                         //Device own address.
 I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;                       //Acknowledge enable.
 I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;  //7-bit address acknowledged.
 I2C_InitStructure.I2C_ClockSpeed = 100000;                        //Clock speed set.

 I2C_Init(I2C1, &I2C_InitStructure);                               //I2C init.
 //**************

 I2C_Cmd(I2C1, ENABLE);                                            //I2C1 enabled.
}

uint8_t LM75_ReadConfiguration() {
	uint8_t data;

	I2C_AcknowledgeConfig(I2C1,ENABLE);    																		//Enable I2C acknowledgment.
	I2C_GenerateSTART(I2C1,ENABLE);																						//Generate I2C1 communication START condition.
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); 							//Wait for EV5.
	I2C_Send7bitAddress(I2C1,0x90,I2C_Direction_Transmitter);                 //Send slave address.
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); //Wait for EV6.
	I2C_SendData(I2C1,0x01);                                                  //Send register address.
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));          //Wait for EV8.
	I2C_GenerateSTART(I2C1,ENABLE);                                           //Send repeated START condition.
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));               //Wait for EV5.
	I2C_Send7bitAddress(I2C1,0x90,I2C_Direction_Receiver);                    //Send slave address for READ.
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));    //Wait for EV6.
	I2C_AcknowledgeConfig(I2C1,DISABLE);                                      //Disable I2C acknowledgment.
	I2C_GenerateSTOP(I2C1,ENABLE);                                            //Send STOP condition.
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));             //Wait for EV7 (Byte received from slave).
	data = I2C_ReceiveData(I2C1);                                             //Receive the data.

	return data;                                                              //Return data.
}

void LM75_WriteConfiguration(uint8_t value) {
	I2C_AcknowledgeConfig(I2C1,ENABLE);                                       //Enable I2C acknowledgment
	I2C_GenerateSTART(I2C1,ENABLE);                                           //Generate I2C1 communication START condition.
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));               //Wait for EV5.
	I2C_Send7bitAddress(I2C1,0x90,I2C_Direction_Transmitter);                 //Send slave address.
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); //Wait for EV6.
	I2C_SendData(I2C1,0x01);                                                  //Send register address.
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));          //Wait for EV8.
	I2C_SendData(I2C1,value);                                                 //Write value to register.
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));          // Wait for EV8.
	I2C_GenerateSTOP(I2C1,ENABLE);                                            //Send STOP condition.
}

void LM75_Shutdown(FunctionalState newstate) {
	uint8_t value;

	value = LM75_ReadConfiguration();
	LM75_WriteConfiguration(newstate == ENABLE ? value | 0x01 : value & 0xFE);
}

uint16_t Read_I2C_register(uint8_t reg){

	uint16_t data;

	I2C_AcknowledgeConfig(I2C1, ENABLE);                                          //Enable I2C acknowledgment
	I2C_GenerateSTART(I2C1, ENABLE);                                              //Generate I2C1 communication START condition.
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));                   //Wait for EV5.
	I2C_Send7bitAddress(I2C1, 0x90, I2C_Direction_Transmitter);                   //Send slave address.
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));     //Wait for EV6.
	I2C_SendData(I2C1,reg);                                                       //Send register address.
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));              //Wait for EV8.
   I2C_GenerateSTART(I2C1,ENABLE);                                              //Send repeated START condition.
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));                   //Wait for EV5.
	I2C_Send7bitAddress(I2C1,0x90,I2C_Direction_Receiver);                        //Send slave address for READ.
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));        //Wait for EV6.
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));                 //Wait for EV7 (Byte received from slave).
	data = (I2C_ReceiveData(I2C1) << 8);                                          //Receive high byte.
	I2C_AcknowledgeConfig(I2C1,DISABLE);                                          //Disable I2C acknowledgment.
	I2C_GenerateSTOP(I2C1,ENABLE);                                                //Send STOP condition.
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));                 //Wait for EV7 (Byte received from slave).
	data |= I2C_ReceiveData(I2C1);                                                //Receive low byte.

  return data;                                                                 //Return data.

}

int16_t LM75_Temperature(void) {
	uint16_t raw;
	int16_t temp;

	raw = Read_I2C_register(0x00) >> 7;      //Sensors data register is 9-bit register, because of that, it slided 7 bits.
	if (raw & 0x0100) {                      //If MSB = 1, temperature is negative, if not temperature is positive or zero.
		// Negative temperature
		temp = -10 * (((~(uint8_t)(raw & 0xFE) + 1) & 0x7F) >> 1) - (raw & 0x01) * 5;
	} else {
		// Positive temperature
		temp = ((raw & 0xFE) >> 1) * 10 + (raw & 0x01) * 5;
	}

	return temp;
}


void PWM_TIMER_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

    /* Enable clock for GPIOD */
    //***********
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    //***********

	/* Alternating functions for pins */
    //***********
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	//
	/* Set pins */
	//***********
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;           //I have used Pin 12
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;       //All chosen push pull
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;     //All chosen as nopull
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;         //Alternating function to use PWM
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;   //Full speed
	GPIO_Init(GPIOD, &GPIO_InitStruct);               //Load the configuration
    //***********

	TIM_TimeBaseInitTypeDef TIM_BaseStruct;               //TIMER BaseStruct definiton
	/* Enable clock for TIM4 */
	//***********
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	//***********

	/* TIM4 Base Settings */
	//***********
   TIM_BaseStruct.TIM_Prescaler = 0;                      //Prescaler is set to 0.
   TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;   //Counter mode up selected.
   TIM_BaseStruct.TIM_Period = 33000;                     //We have used 36000 in ADC experiment,
																						             //I have just changed it to 33000 for easy divison to 3.3V
   TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;       //Div1 selected
   TIM_BaseStruct.TIM_RepetitionCounter = 0;              //We don't want any repetition
	//***********

	/* Initialize TIM4 */
	//***********
    TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
	//***********

	/* Start count on TIM4 */
    TIM_Cmd(TIM4, ENABLE);

}


int main(void)
{

	LEDS_Init();             //Leds Init
	CONFIG_USART3();         //USART3 Config
	          //ADC_Config
	PWM_TIMER_Init();        //PWM_Init

	Config_I2C();            /*Init I2C */

	LM75_Shutdown(DISABLE);  /*Wake up LM75*/

	TIM_OCInitTypeDef TIM_OCStruct;                         // Timer struct defined for pwm.
	TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;              //PWM2 mode selected
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;  //Output state enable
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;       //Polarity chosen as low

   EXTI_Config();

  while (1)
  {
	  ADC_Config();

	  sicaklik = LM75_Temperature()/10.0;
	  //adc_value = read_ADC();

	  if(adc_value <= 1365)
	 	 {
	 		 referans_sicaklik = 30.0;
	 		 GPIO_ResetBits(GPIOD,GPIO_Pin_0);    //Red LED off
	 		 GPIO_ResetBits(GPIOD,GPIO_Pin_1);    //Yellow LED off
	 		 GPIO_SetBits(GPIOD,GPIO_Pin_2);      //Green LED on

	 		overshoot = (sicaklik - 30.0) /33.0*100;

	 	 }

	 	 else if(adc_value > 1365 && adc_value <= 2730)
	 	 {
	 		referans_sicaklik = 33.0;
	 		GPIO_ResetBits(GPIOD,GPIO_Pin_0);    //Red LED off
	 		GPIO_SetBits(GPIOD,GPIO_Pin_1);      //Yellow LED on
	 		GPIO_ResetBits(GPIOD,GPIO_Pin_2);    //Green LED off

	 		overshoot = (sicaklik - 33.0) /33.0*100;

	 	 }

	 	 else if(adc_value > 2730)
	 	 {
	 	 	referans_sicaklik = 36.0;
	 	 	GPIO_SetBits(GPIOD,GPIO_Pin_0);      //Red LED on
	 	 	GPIO_ResetBits(GPIOD,GPIO_Pin_1);    //Yellow LED off
	 	 	GPIO_ResetBits(GPIOD,GPIO_Pin_2);    //Green LED off

	 	 	overshoot = (sicaklik - 36.0) /36.0*100;

	 	 }



	  if(sicaklik > referans_sicaklik)
	  {
            TIM_OCStruct.TIM_Pulse =33000;
	  		TIM_OC1Init(TIM4, &TIM_OCStruct);
	  		TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	  		if(overshoot >= 10.0)
	  		{
	  			GPIO_SetBits(GPIOD,GPIO_Pin_3);      //Red LED on
	  			GPIO_ResetBits(GPIOD,GPIO_Pin_4);    //Yellow LED off
	  		    GPIO_ResetBits(GPIOD,GPIO_Pin_5);    //Green LED off
	  		}

	  		else if( overshoot < 10.0 && overshoot >= 2.0)
	  		{
	  			GPIO_ResetBits(GPIOD,GPIO_Pin_3);     //Red LED off
	  			GPIO_SetBits(GPIOD,GPIO_Pin_4);      //Yellow LED on
	  			GPIO_ResetBits(GPIOD,GPIO_Pin_5);    //Green LED off
	  		}

	  		else if(overshoot < 2.0)
	  		{
	  			GPIO_ResetBits(GPIOD,GPIO_Pin_3);      //Red LED off
	  			GPIO_ResetBits(GPIOD,GPIO_Pin_4);    //Yellow LED off
	  		    GPIO_SetBits(GPIOD,GPIO_Pin_5);    //Green LED on
	  		}

	  }

	  else if(sicaklik < referans_sicaklik)
	  {
		  TIM_OCStruct.TIM_Pulse = 0;
		  TIM_OC1Init(TIM4, &TIM_OCStruct);
		  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

			GPIO_ResetBits(GPIOD,GPIO_Pin_3);      //Red LED off
			GPIO_ResetBits(GPIOD,GPIO_Pin_4);    //Yellow LED off
		    GPIO_ResetBits(GPIOD,GPIO_Pin_5);    //Green LED on

	  }


	  USART_SendData(USART3,sicaklik);
	  MS_Delay(1000);


  }
}


/*
 * Callback used by stm32f4_discovery_audio_codec.c.
 * Refer to stm32f4_discovery_audio_codec.h for more info.
 */
void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  /* TODO, implement your code here */
  return;
}

/*
 * Callback used by stm324xg_eval_audio_codec.c.
 * Refer to stm324xg_eval_audio_codec.h for more info.
 */
uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  /* TODO, implement your code here */
  return -1;
}
