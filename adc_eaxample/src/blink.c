/***************************************************************************//**
 * @file
 * @brief Simple LCD blink demo for EFM32_Gxxx_STK
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_cmu.h"
#include "em_chip.h"
#include "em_timer.h"
#include "em_prs.h"
#include "em_gpio.h"
#include "em_adc.h"

volatile uint32_t msTicks; /* counts 1ms timeTicks */
volatile uint16_t adcReading0 = 0;
volatile uint16_t adcReading1 = 0;
volatile bool adc_set = false;

void ADC0_IRQHandler(void)
{
	uint32_t status = 0;
	uint32_t valid = 0;
	ADC0->IFC = ADC_IFC_SCAN;
	status = (ADC0->STATUS)>>24;
	valid = (ADC0->STATUS) & ADC_STATUS_SCANDV;
	if(ADC0->STATUS & ADC_STATUS_SCANDV)
	{
		switch(status)
		{
			case 0:
				adcReading0 = ADC_DataScanGet(ADC0);
				adc_set = true;
				break;
			case 1:
				adcReading1 = ADC_DataScanGet(ADC0);
				adc_set = true;
				break;
		}
	}

//	ADC0 -> IFC = ADC_IFC_SINGLE;
//	adcReading0 = ADC_DataSingleGet(ADC0);
//	adc_set = true;
}


/***************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 ******************************************************************************/
void SysTick_Handler(void)
{
  msTicks++;       /* increment counter necessary in Delay()*/
}

/***************************************************************************//**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 ******************************************************************************/
void Delay(uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) ;
}

void timer_config(void);
void adc_config(void);

/***************************************************************************//**
 * @brief  Main function
 ******************************************************************************/
int main(void)
{
	/* Ensure core frequency has been updated */
	CMU->CTRL = (CMU->CTRL & ~_CMU_CTRL_HFXOMODE_MASK) | _CMU_CTRL_HFXOMODE_XTAL;
	CMU_OscillatorEnable( cmuOsc_HFXO, true, true);
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO );
	SystemCoreClockUpdate();
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_ADC0, true);
	CMU_ClockEnable(cmuClock_PRS, true);
	CMU_ClockEnable(cmuClock_TIMER0, true);

	GPIO_PinModeSet(gpioPortC, 1, gpioModePushPull, 1);

	PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_TIMER0, PRS_CH_CTRL_SIGSEL_TIMER0OF, prsEdgePos);

	int count = 0;

  /* Chip errata */
	CHIP_Init();

  /* Ensure core frequency has been updated */
 	SystemCoreClockUpdate();


  /* Setup SysTick Timer for 1 msec interrupts  */
 	  if (SysTick_Config(SystemCoreClock / 1000)) {
 	    while (1) ;
 	  }

 	adc_config();
 	timer_config();



  /* Infinite loop */
 	while (1)
 	{
// 		Delay(1000);
// 		GPIO_PinOutToggle(gpioPortC, 1);
 		if(adc_set)
 		{
 			adc_set = false;
 			GPIO_PinOutToggle(gpioPortC, 1);
 		}
 	}
}



void timer_config(void)
{
	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
//	timerInit.prescale = timerPrescale64;	//HFPERCLK undivided
	TIMER_TopSet(TIMER0, 1400); //this will make the LOG_SIG ADC run at 25MHz/25000 = ~1kHz
	TIMER_Init(TIMER0, &timerInit);
}

void adc_config(void)
{

	// Base the ADC configuration on the default setup.
	ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
	ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;
	ADC_InitScan_TypeDef  init_scan = ADC_INITSCAN_DEFAULT;

	// Initialize timebases
	init.timebase = ADC_TimebaseCalc(0);
	init.prescale = ADC_PrescaleCalc(7000000,0);
	ADC_Init(ADC0, &init);


	// Set up scan initialization channel 0 1 2, enable PRS on PRS_ch 0. Timer0 will trigger the scan
//	init_scan.reference = adcRef2V5;
//	init_scan.input = ADC_SCANCTRL_INPUTMASK_CH0 | ADC_SCANCTRL_INPUTMASK_CH1;
//	init_scan.prsEnable  = true; // Enable PRS for ADC
//	ADC_InitScan(ADC0, &init_scan);

	init_scan.reference = adcRef2V5;
	init_scan.input = ADC_SCANCTRL_INPUTMASK_CH0 | ADC_SCANCTRL_INPUTMASK_CH1;
	init_scan.resolution = adcRes12Bit;
	init_scan.diff = false;
	init_scan.prsEnable = true;
	ADC_InitScan(ADC0, &init_scan);
	// Enable ADC Interrupt when Single and SCAN conversion Complete
	ADC0->IEN = ADC_IEN_SCAN;
//	ADC0->IEN = ADC_IEN_SINGLE;

	// Enable ADC interrupt vector in NVIC
	NVIC_EnableIRQ(ADC0_IRQn);
	NVIC_SetPriority(ADC0_IRQn, 1);  // highest pre-empt priority
}
