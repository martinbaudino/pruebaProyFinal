/* Copyright 2016, 6ta Escuela RUSE
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */

/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal example source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20160602 v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/
#include "proyFin.h"       /* <= own header */
#include "uart.h"
#include "led.h"
#include "adc.h"
#include "pwm.h"

/*==================[internal data declaration]==============================*/

#define LPC_UART LPC_USART2
#define UARTx_IRQn  USART2_IRQn
#define UARTx_IRQHandler UART2_IRQHandler
#define _GPDMA_CONN_UART_Tx GPDMA_CONN_UART2_Tx
#define _GPDMA_CONN_UART_Rx GPDMA_CONN_UART2_Rx

/* Transmit and receive ring buffers */
STATIC RINGBUFF_T txring, rxring;

/* Ring buffer size */
#define UART_RB_SIZE 256

/* Transmit and receive buffers */
static uint8_t rxbuff[UART_RB_SIZE], txbuff[UART_RB_SIZE];
/*==================[internal functions declaration]=========================*/
/* Uart Interrupt variables and functions declaration */
static uint8_t uart_interrupt_menu[] =
	"UART Interrupt mode demo ! \r\nPress '1' to '4' to display 4 menus \r\nPress 'x'to exit UART interrupt mode \r\n";
static uint8_t uart_interrupt_menu1[] = "UART interrupt menu 1 \r\n";
static uint8_t uart_interrupt_menu2[] = "UART interrupt menu 2 \r\n";
static uint8_t uart_interrupt_menu3[] = "UART interrupt menu 3 \r\n";
static uint8_t uart_interrupt_menu4[] = "UART interrupt menu 4 \r\n";
/* static uint8_t rxUartIntBuf[1]; */

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
/* Print Welcome Screen Menu subroutine by Interrupt mode */
static void Print_Menu_Interrupt(LPC_USART_T *UARTx)
{
	uint32_t tmp;
	uint8_t *pDat;

	tmp = sizeof(uart_interrupt_menu);
	pDat = (uint8_t *) &uart_interrupt_menu[0];
	Chip_UART_SendRB(UARTx, &txring, pDat, tmp);
}

/* Initialize Interrupt for UART */
static void App_Interrupt_Init(void)
{
	/* Enable UART Rx & line status interrupts */
	/*
	 * Do not enable transmit interrupt here, since it is handled by
	 * UART_Send() function, just to reset Tx Interrupt state for the
	 * first time
	 */
	Chip_UART_IntEnable(LPC_UART, (UART_IER_RBRINT | UART_IER_RLSINT));

	/* Before using the ring buffers, initialize them using the ring
	   buffer init function */
	RingBuffer_Init(&rxring, rxbuff, 1, UART_RB_SIZE);
	RingBuffer_Init(&txring, txbuff, 1, UART_RB_SIZE);

	/* Enable Interrupt for UART channel */
	/* Priority = 1 */
	NVIC_SetPriority(UARTx_IRQn, 1);
	/* Enable Interrupt for UART channel */
	NVIC_EnableIRQ(UARTx_IRQn);
}

/* DeInitialize Interrupt for UART */
static void App_Interrupt_DeInit(void)
{
	/* Disable UART Rx & line status interrupts */
	Chip_UART_IntDisable(LPC_UART, (UART_IER_RBRINT | UART_IER_RLSINT));

	/* Disable Interrupt for UART channel */
	NVIC_DisableIRQ(UARTx_IRQn);
}

/* Interrupt routine for example_uart */
static void App_Interrupt_Test(void)
{
	uint8_t isExit = 0, userInput;
	int len;
	App_Interrupt_Init();

	/* Print out uart interrupt menu */
	Print_Menu_Interrupt(LPC_UART);

	while (!isExit) {
		len = 0;
		while (len == 0) {
			len = Chip_UART_ReadRB(LPC_UART, &rxring, &userInput, 1);
		}
		if (userInput == '1') {
			Chip_UART_SendRB(LPC_UART, &txring, (uint8_t *) &uart_interrupt_menu1[0], sizeof(uart_interrupt_menu1));
		}
		else if (userInput == '2') {
			Chip_UART_SendRB(LPC_UART, &txring, (uint8_t *) &uart_interrupt_menu2[0], sizeof(uart_interrupt_menu2));
		}
		else if (userInput == '3') {
			Chip_UART_SendRB(LPC_UART, &txring, (uint8_t *) &uart_interrupt_menu3[0], sizeof(uart_interrupt_menu3));
		}
		else if (userInput == '4') {
			Chip_UART_SendRB(LPC_UART, &txring, (uint8_t *) &uart_interrupt_menu4[0], sizeof(uart_interrupt_menu4));
		}
		else if (( userInput == 'x') || ( userInput == 'X') ) {
			isExit = 1;
		}
	}

	App_Interrupt_DeInit();
}



/*==================[external functions definition]==========================*/
/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */



int main(void)
{

	uint32_t cnt1 = 0, cnt2 = 0;
	int led_dp = 0, led_step = 1, out_dp = 0;

	/* Generic Initialization */
	SystemCoreClockUpdate();

	leds_init();
	/* Initialize the SCT as PWM and set frequency */
	Chip_SCTPWM_Init(SCT_PWM);
	Chip_SCTPWM_SetRate(SCT_PWM, SCT_PWM_RATE);

	/* Setup Board specific output pin */
	/* Setup board specific pin muxing */

	/* SCT_OUT2 on P2.10 mapped to FUNC1: LED2 */
	Chip_SCU_PinMuxSet(0x2, 10, (SCU_MODE_INACT | SCU_MODE_FUNC1));
	/* SCT_OUT4 on P2.12 mapped to FUNC1: Oscilloscope input */
	Chip_SCU_PinMuxSet(0x2, 12, (SCU_MODE_INACT | SCU_MODE_FUNC1));

	/* Use SCT0_OUT1 pin */
	Chip_SCTPWM_SetOutPin(SCT_PWM, SCT_PWM_OUT, SCT_PWM_PIN_OUT);
	Chip_SCTPWM_SetOutPin(SCT_PWM, SCT_PWM_LED, SCT_PWM_PIN_LED);

	/* Start with 0% duty cycle */
	Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_OUT, Chip_SCTPWM_GetTicksPerCycle(SCT_PWM)/2);
	Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_LED, Chip_SCTPWM_GetTicksPerCycle(SCT_PWM)/2);
	Chip_SCTPWM_Start(SCT_PWM);


	/* Enable SysTick Timer */
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);

	while (1) {
		cnt1 ++;
		cnt2 ++;
		if (cnt1 >= OUT_STEP_CNT) {
			out_dp += 10;
			if (out_dp > 100) {
				out_dp = 0;
			}

			/* Increase dutycycle by 10% every second */
			Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_OUT,
					Chip_SCTPWM_PercentageToTicks(SCT_PWM, out_dp));
			cnt1 = 0;
		}

		if (cnt2 >= LED_STEP_CNT) {
			led_dp += led_step;
			if (led_dp < 0) {
				led_dp = 0;
				led_step = 1;
			}
			if (led_dp > 200) {
				led_dp = 200;
				led_step = -1;
			}

			/* Increment or Decrement Dutycycle by 0.5% every 10ms */
			Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_LED,
					Chip_SCTPWM_PercentageToTicks(SCT_PWM, led_dp)/2);
			cnt2 = 0;
		}
		__WFI();
	}






	/*	uint8_t valorADC_H, valorADC_L;
	uint16_t valorADC;
	uint32_t i;
	float fVal;

	adc_init(1);
	InitUart(UART2, 9600);
	leds_init();

	 adc_convertir();
	 valorADC=adc_pool(1);

	while(1){
		adc_convertir();
		valorADC=adc_pool(1);

		fVal=valorADC;
		fVal=(fVal*3.3)/1023;

		SendUartFloatAscii(UART2, fVal, 2);
//		 App_Interrupt_Test();


		//valorADC_L=(uint8_t)(valorADC&0x00FF);
		//valorADC_H=(uint8_t)((valorADC>>8)&0x00FF);
		//WriteUartByte(UART2, valorADC_L);
		//WriteUartByte(UART2, valorADC_H);


		for(i=0; i<600000; i++);
	}
	 */
}

void ISR_RIT_Handler(void)
{

}

void ISR_UARTx_IRQHandler(void)
{
	Chip_UART_IRQRBHandler(LPC_UART, &rxring, &txring);
}

void teclas_procesar(void)
{

}

void leds_procesar(void)
{

}




/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

