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


// Clock prescaler for TIM2 timer: no prescaling
#define myTIM2_PRESCALER ((uint16_t)0x0000)
// Maximum possible setting for TIM2 overflow 
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

// SPI initialization variables
#define SPI_Direction_1Line_Tx ((uint16_t)0xC000)
#define SPI_Mode_Master ((uint16_t)0x0104)
#define SPI_DataSize_8b ((uint16_t)0x0700)
#define SPI_CPOL_Low ((uint16_t)0x0000)
#define SPI_CPHA_1Edge ((uint16_t)0x0000)
#define SPI_CR1_SSM ((uint16_t)0x0200)
#define SPI_NSS_Soft SPI_CR1_SSM
#define SPI_FirstBit_MSB ((uint16_t)0x0000)

// Function prototypes
void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myGPIOC_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);
void mySPI_Init(void);
void myLCD_Init(void);
void myDAC_Init(void);
void myADC_Init(void);
void LCD_command(uint16_t command);
void LCD_8b_transfer(uint8_t data);
void LCD_update(uint16_t frequency, uint16_t resistance);
uint16_t POT_value(void);
uint16_t DAC_value(uint16_t pot);

// Global variables
uint8_t firstEdge = 1;
float cycles = 0;
float period = 0;
float frequency = 0;
float resistance = 0;

int main(int argc, char* argv[])
 {
 	// State the clock frequency of the microcontroller
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		// Initialize I/O port PA 
	myGPIOB_Init();		// Initialize I/O port PB 
	myGPIOC_Init();		// Initialize I/O port PC
	myTIM2_Init();		// Initialize timer TIM2 
	myEXTI_Init();		// Initialize EXTI 
	mySPI_Init();		// Initialize SPI 
	myLCD_Init();		// Initialize LCD 
	myADC_Init(); 		// Initialize ADC 
	myDAC_Init();		// Initialize DAC 

	// Main system loop
	while (1)
	{
		// Take a measurement from the ADC
		uint16_t pot =  POT_value();

		// Convert ADC measurement to resistance
		resistance = (pot)/((float)0xFFF);
		resistance = resistance*5000;

		// Update DAC output using the ADC measurement
		DAC->DHR12R1 = (DAC_value(pot) & ((uint16_t)0xFFF));

		// Update the LCD with current frequency and resistance values
		LCD_update((uint16_t)frequency, (uint16_t)resistance);
	}

	return 0;
}

void myGPIOA_Init()
{
	// Enable clock for GPIOA peripheral 
	RCC->AHBENR |= (uint32_t)0x20000;

	// Configure PA1 as input: 00 is input
	GPIOA->MODER &= (uint32_t)0xFFFFFFF3;

	// Ensure no pull-up/pull-down for PA1: 00 is no pu/pd
	GPIOA->PUPDR &= (uint32_t)0xFFFFFFF3;

	// Configure PA4 as analog: 11 is analog
	GPIOA->MODER |= (uint32_t)0x300;

	// Ensure no pull-up/pull-down for PA4: 00 is no pu/pd
	GPIOA->PUPDR &= (uint32_t)0xFFFFFCFF;
}

void myGPIOB_Init()
{
    // Enable clock for GPOIB peripheral
    RCC->AHBENR |= (uint32_t)0x40000;

    // Configure PB3 as alternate function: 10 is alternate function
    GPIOB->MODER |= (uint32_t)(0x2 << 6);

    // Ensure no pulldown/pullup for PB3: 00 is no pu/pd
    GPIOB->PUPDR &= (uint32_t)0xFFFFFFF3;

    // Configure PB5 as alternate function: 10 is alternate function
    GPIOB->MODER |= (uint32_t)(0x2 << 10);

    // Ensure no pulldown/pullup for PB5: 00 is no pu/pd
    GPIOB->PUPDR &= (uint32_t)0xFFFFFFF3;

    // Configure PB4 as output: 01 is output
    GPIOB->MODER |= (uint32_t)(0x1 << 8);

    // Ensure no pulldown/pullup for PB4: 00 is no pu/pd
    GPIOB->PUPDR &= (uint32_t)0xFFFFFFF3;
}

void myGPIOC_Init()
{
	// Enable clock for GPIOC peripheral
	RCC->AHBENR |= (uint32_t)0x80000;

	// Configure PC1 as analog: 11 is analog
	GPIOC->MODER |= (uint32_t)0xC;

	// Ensure no pull-up/pulldown for PC1: 00 is no pu/pd
	GPIOC->PUPDR &= (uint32_t)0xFFFFFFF3;
}

void myTIM2_Init()
{
	// Enable clock for TIM2 peripheral
	RCC->APB1ENR |= (uint32_t)0x1; 

	// Configure TIM2: buffer auto-reload, count up, stop on overflow,
	// enable update events, interrupt on overflow only 
	TIM2->ARR = myTIM2_PERIOD;
	TIM2->CR1 |= (uint16_t)0x8C;
	TIM2->CR1 &= (uint16_t)0xFF8C;

	// Set clock prescaler value
	TIM2->PSC = myTIM2_PRESCALER;

	// Update timer registers with above changes
	TIM2->EGR |= (uint16_t)0x1;

	// Assign TIM2 interrupt priority = 0 (highest) in NVIC
	NVIC_SetPriority(15, 0);

	// Enable TIM2 interrupts in NVIC 
	NVIC_EnableIRQ(15);

	// Enable TIM2 interrupt generation
	TIM2->DIER |= 0x1;
}

void myEXTI_Init()
{
	// Map EXTI1 line to PA1
	SYSCFG->EXTICR[0] &= (uint32_t)0xFF0F;
	SYSCFG->EXTICR[0] |= (uint32_t)0x80;

	// Configure EXTI1 line to interrupt using rising-edge trigger
	EXTI->RTSR |= (uint32_t)0x2;

	// Enable interrupts from EXTI1 line
	EXTI->IMR |= (uint32_t)0x2;

	// Assign EXTI1 interrupt priority = 0 (highest) in NVIC 
	NVIC_SetPriority(5, 0);

	// Enable EXTI1 interrupts in NVIC
	NVIC_EnableIRQ(5);
}

// This handler is declared in system/src/cmsis/vectors_stm32f0xx.c 
void TIM2_IRQHandler()
{
	// Check that TIM2 interrupt flag is set
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{	
		// TIM2 overflow has occurred
		trace_printf("\n*** Overflow! ***\n");

		// Clear interrupt flag 
		TIM2->SR &= (uint32_t)0xFFFE;

		// Restart stopped timer
		TIM2->CR1 |= (uint16_t)0x1;
	}
}

// This handler is declared in system/src/cmsis/vectors_stm32f0xx.c
void EXTI0_1_IRQHandler()
{
	// Check if EXTI1 interrupt flag is set
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		// If this is the first rising-edge of the waveform
		if (firstEdge != 0)
		{	
			// Clear count register, start timer and clear first-edge flag
			TIM2->CNT = (uint32_t)0x0;
			TIM2->CR1 |= (uint16_t)0x1; 
			firstEdge = 0;
		}

		// If this is the second rising-edge of the waveform
		else
		{
			// Stop timer
			TIM2->CR1 &= (uint16_t)0xFFFE;

			// Read count register and use it to calculate period and frequency
			cycles = TIM2->CNT; 
			period = cycles/SystemCoreClock;
			frequency = 1/period; 

			// Set first-edge flag
			firstEdge = 1;
		}

		// Clear EXTI1 interrupt flag
		EXTI->PR |= (uint32_t)0x2;
	}
}

void mySPI_Init()
{
    // Enable SPI1 clock
    RCC->APB2ENR |= (uint32_t)0x1000;

    // Create an SPI initialization structure
    SPI_InitTypeDef SPI_InitStructInfo;
    SPI_InitTypeDef* SPI_InitStruct = &SPI_InitStructInfo;

    // Assign values to the SPI initialization structure
    SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;
    SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
    SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
    SPI_InitStruct->SPI_BaudRatePrescaler = ((uint16_t)0x0038);
    SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct->SPI_CRCPolynomial = 7;

    // Initialize SPI communication using the SPI initialization structure
    SPI_Init(SPI1, SPI_InitStruct);

    // Enable the SPI after initialization
    SPI_Cmd(SPI1, ENABLE);
}

void myLCD_Init()
{
    // Switch LCD to 4-bit commands using the format: en/rs/xx/h to send 3 8-bit commands
	LCD_8b_transfer(0x02); 
    LCD_8b_transfer(0x82); 
    LCD_8b_transfer(0x02); 

    // Set LCD to use two lines of 8 characters
    LCD_command(0x0028);

    // Turn on LCD display, turn off cursor
    LCD_command(0x000C);

    // Auto increment LCD addresses and turn off display shifting
    LCD_command(0x0006);

    // Clear display
    LCD_command(0x0001);
}

void LCD_command(uint16_t command)
{
    // Extract each part of the command
    uint8_t rs = (command & 0x0200) >> 9;
    uint8_t h = (command & 0x00F0) >> 4;
    uint8_t l = (command & 0x000F);

    // Generate instructions with enable bit set to 0/1
    uint8_t upper0 = ((rs << 6) | h);
    uint8_t upper1 = ((1 << 7) | (rs << 6) | h);

    uint8_t lower0 = ((rs << 6) | l);
    uint8_t lower1 = ((1 << 7) | (rs << 6) | l);

    // Send upper bits followed by lower bits
    LCD_8b_transfer(upper0);
    LCD_8b_transfer(upper1);
    LCD_8b_transfer(upper0);

    LCD_8b_transfer(lower0);
    LCD_8b_transfer(lower1);
    LCD_8b_transfer(lower0);
}

void LCD_8b_transfer(uint8_t data)
{
    // Set LCK signal to 0 (PB4)
    GPIOB->BSRR |= (uint32_t)0x100000;

    // Wait for SPI busy flag to clear
    while ((SPI1->SR & (uint16_t)0x80) != 0){}

    // Send 8-bit data to LCD
    SPI_SendData8(SPI1, data);

    // Wait for SPI busy flag to clear
    while ((SPI1->SR & (uint16_t)0x80) != 0){}

    // Set LCK signal to 1 (PB4)
    GPIOB->BSRR |= (uint32_t)0x10;
}

void LCD_update(uint16_t frequency, uint16_t resistance)
{
    // Convert frequency and resistance to a max of 4 digits
    frequency = frequency % 10000;
    resistance = resistance % 10000;

    // Create variables for each individual digit (1 is the leftmost digit on LCD)
    uint16_t freq4 = 0;
    uint16_t freq3 = 0;
    uint16_t freq2 = 0;
    uint16_t freq1 = 0;

    uint16_t res4 = 0;
    uint16_t res3 = 0;
    uint16_t res2 = 0;
    uint16_t res1 = 0;

    // Decode freqency and resistance values to extract their individual digits
    freq4 = (frequency % 10); 
    freq3 = ((frequency - freq4) % 100) / 10; 
    freq2 = ((frequency - freq4 - (10 * freq3)) % 1000) / 100; 
    freq1 = frequency / 1000; 

    res4 = (resistance % 10); 
    res3 = ((resistance - res4) % 100) / 10; 
    res2 = ((resistance - res4 - (10 * res3)) % 1000) / 100; 
    res1 = resistance / 1000; 

    // Delay giving time for LCD to process
    for (int a = 1; a <= 10000; a++){}

    // Set address to first line
    LCD_command(0x0080);

	//F:(digit 1)(digit 2)(digit 3)(digit 4)Hz
    LCD_command(0x0200 | 0x0046); 
    LCD_command(0x0200 | 0x003A); 
    LCD_command(0x0200 | 0x0030 | freq1);
    LCD_command(0x0200 | 0x0030 | freq2);
    LCD_command(0x0200 | 0x0030 | freq3);
    LCD_command(0x0200 | 0x0030 | freq4);
    LCD_command(0x0200 | 0x0048); 
    LCD_command(0x0200 | 0x007A); 

    // Set address to second line
    LCD_command(0x00C0);

	//R:(digit 1)(digit 2)(digit 3)(digit 4)Oh
    LCD_command(0x0200 | 0x0052); 
    LCD_command(0x0200 | 0x003A); 
    LCD_command(0x0200 | 0x0030 | res1); 
    LCD_command(0x0200 | 0x0030 | res2); 
    LCD_command(0x0200 | 0x0030 | res3); 
    LCD_command(0x0200 | 0x0030 | res4); 
    LCD_command(0x0200 | 0x004F); 
    LCD_command(0x0200 | 0x0068); 
}

void myADC_Init()
{
	// Enable clock for ADC peripheral 
	RCC->APB2ENR |= (uint32_t)0x200;

	// Calibrate the ADC, wait for callibration flag
	ADC1->CR |= (uint32_t)0x80000000;
	while ((ADC1->CR & 0x80000000) != 0) {};

	// Configure ADC for continuous mode
	ADC1->CFGR1 |= (uint32_t)0x2000; 

	// Configure ADC for overrun mode 
	ADC1->CFGR1 |= (uint32_t)0x1000; 

	// Configure ADC for right-side data alignment
	ADC1->CFGR1 &= (uint32_t)0xFFFFFFDF;

	// Set ADC to use 12-bit resolution
	ADC1->CFGR1 &= (uint32_t)0xFFFFFFE7;

	// Configure ADC to use channel 11 for conversion (PC1)
	ADC1->CHSELR |= (uint32_t)0x800; 

	// Configure ADC to use fastest sampling speed
	ADC1->SMPR &= (uint32_t)0xFFFFFFF8; 	

	// Enable ADC for conversion, wait for enable flag
	ADC1->CR |= (uint32_t)0x1;
	while((ADC1->ISR & 0x1) != 1) {};
}

uint16_t POT_value(void)
{
	// Start ADC conversion
	ADC1->CR |= (uint32_t)0x4;

	// Wait for ADC to finish
	while((ADC1->ISR & 0x4) == 0) {};

	// Reset ADC conversion flag
	ADC1->ISR &= (uint32_t)0xFFFFFFFB;

	// Return converted ADC value
	return (ADC1->DR & 0x0FFF);
}

void myDAC_Init()
{
	// Enable clock for DAC peripheral
	RCC->APB1ENR |= (uint32_t)0x20000000;

	// Enable DAC
	DAC->CR |= (uint32_t)0x1;
}

uint16_t DAC_value(uint16_t pot)
{
	// Specify DAC output range since optocoupler doesn't turn on until 0.7V
	float range = (3.3 - 0.7);

	// Map ADC value to the specified DAC output range
	float voltage = (((float)pot*range)/((float)0xFFF)) + 0.7;
	float output = (voltage/3.3)*((float)0xFFF);

	// Return DAC output value
	return ((uint16_t)output);
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
