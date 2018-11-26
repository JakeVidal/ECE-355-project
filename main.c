//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
// This is template code for Part 2 of Introductory Lab.
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

#define SPI_Direction_1Line_Tx ((uint16_t)0xC000)
#define SPI_Mode_Master ((uint16_t)0x0104)
#define SPI_DataSize_8b ((uint16_t)0x0700)
#define SPI_CPOL_Low ((uint16_t)0x0000)
#define SPI_CPHA_1Edge ((uint16_t)0x0000)
#define SPI_CR1_SSM ((uint16_t)0x0200)
#define SPI_NSS_Soft SPI_CR1_SSM
#define SPI_FirstBit_MSB ((uint16_t)0x0000)

void myGPIOA_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);
void myGPIOB_Init(void);
void mySPI_Init(void);
void myLCD_Init(void);
void LCD_command(uint16_t command);
void LCD_8b_transfer(uint8_t data);
void LCD_update(uint16_t frequency, uint16_t resistance);

// Your global variables...
uint8_t firstEdge = 1;
float cycles = 0;
float period = 0;
float frequency = 0;
float resistance = 0;

int
main(int argc, char* argv[])
{

	trace_printf("This is Part 2 of Introductory Lab...\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Initialize I/O port PA */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myEXTI_Init();		/* Initialize EXTI */
	myGPIOB_Init();		/* Initialize I/O port PB */
	mySPI_Init();		/* Initialize SPI */
	myLCD_Init();		/* Initialize LCD */

	while (1)
	{
		//frequency = 4567;
		//resistance = 1234;
		LCD_update((uint16_t)4567, (uint16_t)1234);
	}

	return 0;

}


void myGPIOA_Init()
{
	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= (uint32_t)0x20000; // 1 enables clock, 0 disables clock

	/* Configure PA1 as input */
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= (uint32_t)0xFFFFFFF3;// 0x00 is input

	/* Ensure no pull-up/pull-down for PA1 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= (uint32_t)0xFFFFFFF3; // 0x00 is no pullup/pulldown
}


void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= (uint32_t)0x1; // 0 disables timer 1 enables timer

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->ARR = myTIM2_PERIOD;
	TIM2->CR1 |= (uint16_t)0x8C;
	TIM2->CR1 &= (uint16_t)0xFF8C;

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR |= (uint16_t)0x1;

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(15, 0);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(15);

	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER |= 0x1;
}


void myEXTI_Init()
{
	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] &= (uint32_t)0xFF0F;
	SYSCFG->EXTICR[0] |= (uint32_t)0x80;

	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= (uint32_t)0x2;

	/* Unmask interrupts from EXTI1 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= (uint32_t)0x2;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[1], or use NVIC_SetPriority
	NVIC_SetPriority(5, 0);

	/* Enable EXTI1 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(5);
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= (uint32_t)0xFFFE;

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= (uint16_t)0x1; //start timer
	}
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler()
{
	// Your local variables...

	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		if (firstEdge != 0)
		{
			TIM2->CNT = (uint32_t)0x0; //clear count register
			TIM2->CR1 |= (uint16_t)0x1; //start timer
			firstEdge = 0; //clear first edge flag
		}
		else
		{
			TIM2->CR1 &= (uint16_t)0xFFFE; //stop timer
			cycles = TIM2->CNT; //read count register
			period = cycles/SystemCoreClock; //calculate period
			frequency = 1/period; //calculate frequency

			//print results to console
			trace_printf("Period: %8.9f s,   ", period);
			trace_printf("Frequency: %8.3f Hz\n", frequency);

			firstEdge = 1; //set first edge flag

		}

		EXTI->PR |= (uint32_t)0x2; // clear EXTI interrupt pending flag

	}
}

void myGPIOB_Init(){
    //enable clock for GPOIB peripheral
    RCC->AHBENR |= (uint32_t)0x40000;

    //configure PB3 as alternate function
    GPIOB->MODER |= (uint32_t)(0x2 << 6);

    //ensure no pulldown/pullup for PB3
    GPIOB->PUPDR &= (uint32_t)0xFFFFFFF3;

    //configure PB5 as alternate function
    GPIOB->MODER |= (uint32_t)(0x2 << 10);

    //ensure no pulldown/pullup for PB5
    GPIOB->PUPDR &= (uint32_t)0xFFFFFFF3;

    //configure PB4 as output
    GPIOB->MODER |= (uint32_t)(0x1 << 8);

    //ensure no pulldown/pullup for PB4
    GPIOB->PUPDR &= (uint32_t)0xFFFFFFF3;
}

void mySPI_Init(){
    //enable SPI1 clock
    RCC->APB2ENR |= (uint32_t)0x1000;

    //create an initialization structure
    SPI_InitTypeDef SPI_InitStructInfo;
    SPI_InitTypeDef* SPI_InitStruct = &SPI_InitStructInfo;

    //give values to the initialization structure
    SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;
    SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
    SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
    SPI_InitStruct->SPI_BaudRatePrescaler = ((uint16_t)0x0038);
    SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct->SPI_CRCPolynomial = 7;

    //initialize the SPI using the initialization structure
    SPI_Init(SPI1, SPI_InitStruct);

    //enable the SPI after initialization
    SPI_Cmd(SPI1, ENABLE);
}

void myLCD_Init(){
    //switch LCD to 4-bit commands
	LCD_8b_transfer(0x02); //00xx0010 = en/rs/xx/h
    LCD_8b_transfer(0x82); //10xx0010 = en/rs/xx/h
    LCD_8b_transfer(0x02); //00xx0010 = en/rs/xx/h

    //set LCD to use two lines of 8 characters
    LCD_command(0x0028);

    //turn on LCD display, turn off cursor
    LCD_command(0x000C);

    //auto increment LCD addresses and turn off display shifting
    LCD_command(0x0006);

    //clear display
    LCD_command(0x0001);
}

void LCD_command(uint16_t command){
    //extract eact part of the command
    uint8_t rs = (command & 0x0200) >> 9;
    uint8_t h = (command & 0x00F0) >> 4;
    uint8_t l = (command & 0x000F);

    //generate instructions with enable bit set to 0/1
    uint8_t upper0 = ((rs << 6) | h);
    uint8_t upper1 = ((1 << 7) | (rs << 6) | h);

    uint8_t lower0 = ((rs << 6) | l);
    uint8_t lower1 = ((1 << 7) | (rs << 6) | l);

    //send command
    LCD_8b_transfer(upper0);
    LCD_8b_transfer(upper1);
    LCD_8b_transfer(upper0);

    LCD_8b_transfer(lower0);
    LCD_8b_transfer(lower1);
    LCD_8b_transfer(lower0);

}

void LCD_8b_transfer(uint8_t data){
    //set LCK signal to 0 (PB4)
    GPIOB->BSRR |= (uint32_t)0x100000;

    //wait for SPI busy flag to clear
    while ((SPI1->SR & (uint16_t)0x80) != 0){}

    //send 8-bit data to LCD
    SPI_SendData8(SPI1, data);

    //wait for SPI busy flag to clear
    while ((SPI1->SR & (uint16_t)0x80) != 0){}

    //set LCK signal to 1 (PB4)
    GPIOB->BSRR |= (uint32_t)0x10;
}

void LCD_update(uint16_t frequency, uint16_t resistance){
    //can only display max of 4 digits
    frequency = frequency % 10000;
    resistance = resistance % 10000;

    uint16_t freq4 = 0;
    uint16_t freq3 = 0;
    uint16_t freq2 = 0;
    uint16_t freq1 = 0;

    uint16_t res4 = 0;
    uint16_t res3 = 0;
    uint16_t res2 = 0;
    uint16_t res1 = 0;

    freq4 = (frequency % 10); //get 4th digit of frequency
    freq3 = ((frequency - freq4) % 100) / 10; // get 3rd digit of frequency
    freq2 = ((frequency - freq4 - (10 * freq3)) % 1000) / 100; //get 2nd digit of frequency
    freq1 = frequency / 1000; // get 1st digit of frequency

    res4 = (resistance % 10); //get 4th digit of resistance
    res3 = ((resistance - res4) % 100) / 10; // get 3rd digit of resistance
    res2 = ((resistance - res4 - (10 * res3)) % 1000) / 100; //get 2nd digit of resistance
    res1 = resistance / 1000; // get 1st digit of resistance

    //set address to first line
    LCD_command(0x0080);

    LCD_command(0x0200 | 0x0046); //F
    LCD_command(0x0200 | 0x003A); //:
    LCD_command(0x0200 | 0x0030 | freq1); //digit 1
    LCD_command(0x0200 | 0x0030 | freq2); //digit 2
    LCD_command(0x0200 | 0x0030 | freq3); //digit 3
    LCD_command(0x0200 | 0x0030 | freq4); //digit 4
    LCD_command(0x0200 | 0x0048); //H
    LCD_command(0x0200 | 0x007A); //z

    //set address to second line
    LCD_command(0x00C0);

    LCD_command(0x0200 | 0x0052); //R
    LCD_command(0x0200 | 0x003A); //:
    LCD_command(0x0200 | 0x0030 | res1); //digit 1
    LCD_command(0x0200 | 0x0030 | res2); //digit 2
    LCD_command(0x0200 | 0x0030 | res3); //digit 3
    LCD_command(0x0200 | 0x0030 | res4); //digit 4
    LCD_command(0x0200 | 0x004F); //O
    LCD_command(0x0200 | 0x0068); //h
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
