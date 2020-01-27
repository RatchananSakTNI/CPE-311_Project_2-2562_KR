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

//#include "dwt_delay.h"

/*already implemented */
void SystemClock_Config(void);
void GPIO_Config(void);
void SevenSeg(uint8_t*);

uint8_t seg[4];
uint32_t digit[4] = { LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3 };

int main()
{
	SystemClock_Config();
	//DWT_Init();
	GPIO_Config();

	while (1)
	{	
		// saparate digit example
		/*
		seg[0] = dis_temp/1000;
		seg[1] = (dis_temp/100)%10;
		seg[2] = (dis_temp%100)/10;
		seg[3] = dis_temp%10;
		*/

		//SevenSeg(seg);
	}
}

void GPIO_Config()
{
	LL_GPIO_InitTypeDef gpio_conf;

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

	gpio_conf.Mode = LL_GPIO_MODE_OUTPUT;
	gpio_conf.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio_conf.Pull = LL_GPIO_PULL_NO;
	gpio_conf.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;

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

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

	gpio_conf.Pin = LL_GPIO_PIN_0;
	LL_GPIO_Init(GPIOC, &gpio_conf);
	gpio_conf.Pin = LL_GPIO_PIN_1;
	LL_GPIO_Init(GPIOC, &gpio_conf);
	gpio_conf.Pin = LL_GPIO_PIN_2;
	LL_GPIO_Init(GPIOC, &gpio_conf);
	gpio_conf.Pin = LL_GPIO_PIN_3;
	LL_GPIO_Init(GPIOC, &gpio_conf);
}

void SevenSeg(uint8_t* num)
{
	uint8_t i;
	uint8_t position = 3; // most right digit

	for (i = 4; i >= 1; --i)
	{
		LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3);
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15 | LL_GPIO_PIN_2);
		
		if(position == 1)
		{
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_3); // dp
		}
		else
		{
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_3); // dp
		}
		
		switch (*(num + (i-1)))
		{
		case 0:
			if(i == 1)
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
