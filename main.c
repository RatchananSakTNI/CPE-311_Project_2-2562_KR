/*Base register adddress header file*/
#include "stm32l1xx.h"
/*Library related header files*/
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_tim.h"
#include "dwt_delay.h"
#include "math.h"

void SystemClock_Config(void);
void GPIO_Config(void);
void TIMx_IC_Config(void);
void TIM_TRIG_Config(void);
void TIM_Motor_Config(void);
void TIM4_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void SevenSeg(uint8_t*);

float distance;
uint16_t to_display;
uint32_t micro_time = 0;
uint16_t r = 0;
uint16_t indx = 0;
uint32_t count = 0;
uint8_t state = 0;
uint8_t flag = 0;
float ori[60];
float first;
float later;

uint8_t seg[4];
uint32_t digit[4] = { LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3 };

int main()
{
	SystemClock_Config();
	DWT_Init();
	GPIO_Config();
	TIMx_IC_Config();
	TIM_TRIG_Config();
	TIM_Motor_Config();
	
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);

	while (1)
	{
		// saparate digit example
		seg[0] = to_display / 1000;
		seg[1] = (to_display / 100) % 10;
		seg[2] = (to_display % 100) / 10;
		seg[3] = to_display % 10;

		SevenSeg(seg);
	}
}

void GPIO_Config()
{
	LL_GPIO_InitTypeDef gpio_conf;

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	gpio_conf.Mode = LL_GPIO_MODE_OUTPUT;
	gpio_conf.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio_conf.Pull = LL_GPIO_PULL_NO;
	gpio_conf.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	gpio_conf.Pin = LL_GPIO_PIN_2;
	LL_GPIO_Init(GPIOA, &gpio_conf);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

	gpio_conf.Pin = LL_GPIO_PIN_2;
	LL_GPIO_Init(GPIOB, &gpio_conf);
	gpio_conf.Pin = LL_GPIO_PIN_10;
	LL_GPIO_Init(GPIOB, &gpio_conf);
	gpio_conf.Pin = LL_GPIO_PIN_11;
	LL_GPIO_Init(GPIOB, &gpio_conf);
	gpio_conf.Pin = LL_GPIO_PIN_12;
	LL_GPIO_Init(GPIOB, &gpio_conf);
	gpio_conf.Pin = LL_GPIO_PIN_13;
	LL_GPIO_Init(GPIOB, &gpio_conf);
	gpio_conf.Pin = LL_GPIO_PIN_14;
	LL_GPIO_Init(GPIOB, &gpio_conf);
	gpio_conf.Pin = LL_GPIO_PIN_15;
	LL_GPIO_Init(GPIOB, &gpio_conf);
	gpio_conf.Pin = LL_GPIO_PIN_3;
	LL_GPIO_Init(GPIOB, &gpio_conf);
	gpio_conf.Pin = LL_GPIO_PIN_7;
	LL_GPIO_Init(GPIOB, &gpio_conf);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

	gpio_conf.Pin = LL_GPIO_PIN_0;
	LL_GPIO_Init(GPIOC, &gpio_conf);
	gpio_conf.Pin = LL_GPIO_PIN_1;
	LL_GPIO_Init(GPIOC, &gpio_conf);
	gpio_conf.Pin = LL_GPIO_PIN_2;
	LL_GPIO_Init(GPIOC, &gpio_conf);
	gpio_conf.Pin = LL_GPIO_PIN_3;
	LL_GPIO_Init(GPIOC, &gpio_conf);

	gpio_conf.Mode = LL_GPIO_MODE_ALTERNATE;

	gpio_conf.Alternate = LL_GPIO_AF_1;
	gpio_conf.Pin = LL_GPIO_PIN_15;
	LL_GPIO_Init(GPIOA, &gpio_conf);

	gpio_conf.Alternate = LL_GPIO_AF_2;
	gpio_conf.Pin = LL_GPIO_PIN_8;
	LL_GPIO_Init(GPIOB, &gpio_conf);
	gpio_conf.Pin = LL_GPIO_PIN_9;
	LL_GPIO_Init(GPIOB, &gpio_conf);

	gpio_conf.Pin = LL_GPIO_PIN_5;
	LL_GPIO_Init(GPIOB, &gpio_conf);
	gpio_conf.Pin = LL_GPIO_PIN_4;
	LL_GPIO_Init(GPIOB, &gpio_conf);
}

void TIMx_IC_Config(void)
{
	LL_TIM_IC_InitTypeDef timic;

	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

	//TIM_IC Configure CH3 and CH4
	timic.ICActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI;
	timic.ICFilter = LL_TIM_IC_FILTER_FDIV1_N2;
	timic.ICPrescaler = LL_TIM_ICPSC_DIV1;
	timic.ICPolarity = LL_TIM_IC_POLARITY_RISING;
	LL_TIM_IC_Init(TIM4, LL_TIM_CHANNEL_CH3, &timic);

	timic.ICPolarity = LL_TIM_IC_POLARITY_FALLING;
	LL_TIM_IC_Init(TIM4, LL_TIM_CHANNEL_CH4, &timic);

	NVIC_SetPriority(TIM4_IRQn, 0);

	NVIC_EnableIRQ(TIM4_IRQn);
	LL_TIM_EnableIT_CC3(TIM4);
	LL_TIM_EnableIT_CC4(TIM4);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH4);

	LL_TIM_EnableCounter(TIM4);
}

// for trigger (30ms base)
void TIM_TRIG_Config()
{
	LL_TIM_InitTypeDef tim_init;
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

	tim_init.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	tim_init.CounterMode = LL_TIM_COUNTERMODE_UP;
	tim_init.Autoreload = 3000 - 1;
	tim_init.Prescaler = 320 - 1;

	LL_TIM_Init(TIM2, &tim_init);

	// TIM_OC
	LL_TIM_OC_InitTypeDef tim_oc_initstructure;

	tim_oc_initstructure.OCState = LL_TIM_OCSTATE_DISABLE;
	tim_oc_initstructure.OCMode = LL_TIM_OCMODE_PWM1;
	tim_oc_initstructure.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	tim_oc_initstructure.CompareValue = 2; // 20us pulse
	LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH1, &tim_oc_initstructure);
	/*Interrupt Configure*/
	NVIC_SetPriority(TIM2_IRQn, 0);
	NVIC_EnableIRQ(TIM2_IRQn);
	LL_TIM_EnableIT_CC1(TIM2);

	/*Start Output Compare in PWM Mode*/
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM2);
}

void TIM_Motor_Config()
{
	LL_TIM_InitTypeDef timbase_initstructure;

	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

	timbase_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV2;
	timbase_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP;
	timbase_initstructure.Autoreload = 20 - 1;
	timbase_initstructure.Prescaler = 16000 - 1;

	LL_TIM_Init(TIM3, &timbase_initstructure);

	LL_TIM_OC_InitTypeDef tim_oc_initstructure;

	tim_oc_initstructure.OCState = LL_TIM_OCSTATE_DISABLE;
	tim_oc_initstructure.OCMode = LL_TIM_OCMODE_PWM1;
	tim_oc_initstructure.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	tim_oc_initstructure.CompareValue = LL_TIM_GetAutoReload(TIM3)*0.7; //100% duty
	LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH1, &tim_oc_initstructure);
	LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH2, &tim_oc_initstructure);
	/*Interrupt Configure*/
	NVIC_SetPriority(TIM3_IRQn, 0);
	NVIC_EnableIRQ(TIM3_IRQn);
	LL_TIM_EnableIT_CC1(TIM3);
	LL_TIM_EnableIT_CC2(TIM3);

	/*Start Output Compare in PWM Mode*/
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
	LL_TIM_EnableCounter(TIM3);
}

void TIM4_IRQHandler()
{
	if (LL_TIM_IsActiveFlag_CC3(TIM4) == SET)
	{
		LL_TIM_ClearFlag_CC3(TIM4);
		micro_time = 0;
		while (LL_TIM_IsActiveFlag_CC4(TIM4) != SET)
		{
			micro_time++;
			DWT_Delay(1);
		}
		LL_TIM_ClearFlag_CC4(TIM4);
		micro_time = micro_time * 2;
		distance = micro_time * 340.0 / 2.0 / 1000000.0 * 1.40;
		to_display = distance * 100;
	}
//	if (LL_TIM_IsActiveFlag_CC4(TIM4) == SET)
//	{
//		LL_TIM_ClearFlag_CC4(TIM4);
//	}
}

// trigger 30ms
void TIM2_IRQHandler()
{
	if (LL_TIM_IsActiveFlag_UPDATE(TIM2) == SET)
	{
		LL_TIM_ClearFlag_UPDATE(TIM2);
		
		if(flag < 1)
		{
		}
		else
		{	
			ori[indx] = distance;
		}
		indx++;
	}
	
	if (LL_TIM_IsActiveFlag_CC1(TIM2) == SET)
	{
		LL_TIM_ClearFlag_CC1(TIM2);
	}
}

void TIM3_IRQHandler(void)
{
	uint16_t i;
	float temp = 0;
	
	if (LL_TIM_IsActiveFlag_UPDATE(TIM3) == SET)
	{
		LL_TIM_ClearFlag_UPDATE(TIM3);
		count++;

		if (count >= 150 + 1) // 50 = 1s
		{
			if(flag < 1)
			{
				flag = 1;
			}
			else if (flag == 1)
			{
				for(i=0;i<indx;i++)
				{
					temp += ori[i];
				}
				first = temp/indx;
				temp = 0;
				flag = 2;
			}
			else if (flag == 2)
			{
				temp = 0;
				flag = 3;
			}
			else if (flag == 3)
			{
				for(i=0;i<indx;i++)
				{
					temp += ori[i];
				}
				later = temp/indx;
				temp = 0;
				flag = 2;
				
				if (fabs(first - later) >= (first*0.2))
				{
					LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_2);
				}
			}
			indx = 0;
			
			if (state <= 0)
			{
				LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
				LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);
				LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_ALTERNATE);
				state = 1;
			}
			else
			{
				LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
				LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);
				LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
				state = 0;
			}
			count = 0;
		}
	}
	if (LL_TIM_IsActiveFlag_CC1(TIM3) == SET)
	{
		LL_TIM_ClearFlag_CC1(TIM3);
	}
	if (LL_TIM_IsActiveFlag_CC2(TIM3) == SET)
	{
		LL_TIM_ClearFlag_CC2(TIM3);
	}
}

void SevenSeg(uint8_t* num)
{
	uint8_t i;
	uint8_t position = 3; // most right digit

	for (i = 4; i >= 1; --i)
	{
		LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3);
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15 | LL_GPIO_PIN_2);

		if (position == 1)
		{
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_3); // dp
		}
		else
		{
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_3); // dp
		}

		switch (*(num + (i - 1)))
		{
		case 0:
			if (i == 1)
			{
				LL_GPIO_ResetOutputPin(GPIOC, digit[position]);
			}
			else
			{
				LL_GPIO_SetOutputPin(GPIOC, digit[position]);
				LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2); // a
				LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10); // b
				LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11); // c
				LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12); // d
				LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_13); // e
				LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14); // f
				LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_15); // g
			}
			--position;
			break;
		case 1:
			LL_GPIO_SetOutputPin(GPIOC, digit[position]);
			--position;
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2); // a
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10); // b
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11); // c
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12); // d
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13); // e
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14); // f
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_15); // g
			break;
		case 2:
			LL_GPIO_SetOutputPin(GPIOC, digit[position]);
			--position;
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2); // a
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10); // b
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_11); // c
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12); // d
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_13); // e
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14); // f
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_15); // g
			break;
		case 3:
			LL_GPIO_SetOutputPin(GPIOC, digit[position]);
			--position;
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2); // a
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10); // b
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11); // c
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12); // d
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13); // e
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14); // f
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_15); // g
			break;
		case 4:
			LL_GPIO_SetOutputPin(GPIOC, digit[position]);
			--position;
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2); // a
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10); // b
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11); // c
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12); // d
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13); // e
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14); // f
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_15); // g
			break;
		case 5:
			LL_GPIO_SetOutputPin(GPIOC, digit[position]);
			--position;
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2); // a
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10); // b
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11); // c
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12); // d
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13); // e
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14); // f
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_15); // g
			break;
		case 6:
			LL_GPIO_SetOutputPin(GPIOC, digit[position]);
			--position;
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2); // a
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10); // b
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11); // c
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12); // d
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_13); // e
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14); // f
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_15); // g
			break;
		case 7:
			LL_GPIO_SetOutputPin(GPIOC, digit[position]);
			--position;
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2); // a
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10); // b
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11); // c
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12); // d
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13); // e
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14); // f
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_15); // g
			break;
		case 8:
			LL_GPIO_SetOutputPin(GPIOC, digit[position]);
			--position;
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2); // a
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10); // b
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11); // c
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12); // d
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_13); // e
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14); // f
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_15); // g
			break;
		case 9:
			LL_GPIO_SetOutputPin(GPIOC, digit[position]);
			--position;
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2); // a
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10); // b
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11); // c
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12); // d
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13); // e
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14); // f
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_15); // g
			break;
		default:
			break;
		}
		LL_mDelay(1);
	}
}

void SystemClock_Config(void)
{
	/* Enable ACC64 access and set FLASH latency */
	LL_FLASH_Enable64bitAccess();;
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

	/* Set Voltage scale1 as MCU will run at 32MHz */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

	/* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
	while (LL_PWR_IsActiveFlag_VOSF() != 0)
	{
	};

	/* Enable HSI if not already activated*/
	if (LL_RCC_HSI_IsReady() == 0)
	{
		/* HSI configuration and activation */
		LL_RCC_HSI_Enable();
		while (LL_RCC_HSI_IsReady() != 1)
		{
		};
	}


	/* Main PLL configuration and activation */
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3);

	LL_RCC_PLL_Enable();
	while (LL_RCC_PLL_IsReady() != 1)
	{
	};

	/* Sysclk activation on the main PLL */
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
	{
	};

	/* Set APB1 & APB2 prescaler*/
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

	/* Set systick to 1ms in using frequency set to 32MHz                             */
	/* This frequency can be calculated through LL RCC macro                          */
	/* ex: __LL_RCC_CALC_PLLCLK_FREQ (HSI_VALUE, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3); */
	LL_Init1msTick(32000000);

	/* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
	LL_SetSystemCoreClock(32000000);
}
