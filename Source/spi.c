/*
 * spi.c
 *
 *  Created on: Jun 18, 2024
 *      Author: VOVAN
 */
#include<spi.h>

#define GPIOAEN		(1U<<0)
#define SPI1EN		(1U<<12)

/*PA5---CLK
 *PA6---MISO
 *PA7---MOSI*/
/*PA9--Slave select(SS)*/
#define CLK			(1U<<5)
#define MISO		(1U<<6)
#define MOSI		(1U<<7)
#define CS			(1U<<9)
#define SR_TXE		(1U<<1)
#define SR_BSY		(1U<<7)
#define SR_RXNE		(1U<<0)

void spi_gpio_init(void)
{
	/*Enable clock access to GPIOA*/
	RCC->AHB1ENR|=GPIOAEN;
	/*******************Set PA5 PA6 PA7 as alternate function**************/

	/*PA5*/
	GPIOA->MODER&=~(1U<<10);
	GPIOA->MODER|=(1U<<11);

	/*PA6*/
	GPIOA->MODER&=~(1U<<12);
	GPIOA->MODER|=(1U<<13);

	/*PA7*/
	GPIOA->MODER&=~(1U<<14);
	GPIOA->MODER|=(1U<<15);

	/*Set PA9 as output pin*/
	GPIOA->MODER&=~(1U<<19);
	GPIOA->MODER|=(1U<<18);
	/***********************Set PA5 PA6 PA5 as alternate function AF05***************/

	/*PA5*/
	GPIOA->AFR[0]|=(1U<<20);
	GPIOA->AFR[0]&=~(1U<<21);
	GPIOA->AFR[0]|=(1U<<22);
	GPIOA->AFR[0]&=~(1U<<23);

	/*PA6*/
	GPIOA->AFR[0]|=(1U<<24);
	GPIOA->AFR[0]&=~(1U<<25);
	GPIOA->AFR[0]|=(1U<<26);
	GPIOA->AFR[0]&=~(1U<<27);

	/*PA7*/
	GPIOA->AFR[0]|=(1U<<28);
	GPIOA->AFR[0]&=~(1U<<29);
	GPIOA->AFR[0]|=(1U<<30);
	GPIOA->AFR[0]&=~(1U<<31);
}

void spi_config(void)
{
	/*Enable clock access to SPI*/
	RCC->APB2ENR|=SPI1EN;

	/*Set clock to fPCLK/4*/
	SPI1->CR1|=(1U<<3);
	SPI1->CR1&=~(1U<<4);
	SPI1->CR1&=~(1U<<5);

	/*Set CPOL and CPHA to 1*/
	SPI1->CR1|=(1U<<0);
	SPI1->CR1|=(1U<<1);

	/*Enable full duplex*/
	SPI1->CR1&=~(1U<<10);

	/*Set MSB first*/
	SPI1->CR1&=~(1U<<7);

	/*Set mode to Master*/
	SPI1->CR1|=(1U<<2);

	/*Set 8 bit data mode*/
	SPI1->CR1&=~(1U<<11);

	/*Select software slave management by setting SSM=1 and SSI=1*/
	SPI1->CR1|=(1U<<8);
	SPI1->CR1|=(1U<<9);

	/*Enable SPI*/
	SPI1->CR1|=(1U<<6);

}
 void spi_transmit(uint32_t* data, uint32_t size)
 {
	 uint32_t i=0;
	 uint32_t temp;
	 while(i<size)
	 {
		 /*Wait until TXE is set*/
		 while(!(SPI1->SR&SR_TXE)){}
		 /*Wait the data to data register*/
		 SPI1->DR= data[i];
		 i++;
	 }
	 /*Wait until TXE is set*/
	 while(!(SPI1->SR&SR_TXE)){}
	 /*Wait for BUSY flag to reset*/
	 while((SPI1->SR&SR_BSY)){}
	 /*Clear OVR flag*/
	 temp=SPI1->DR;
	 temp=SPI1->SR;
 }

 void spi_receive(uint32_t* data, uint32_t size)
 {

	 while(size)
	 {
		 /*Send dummy data */
		 SPI1->DR=0;
		 /*Wait for RXNE flag to be set */
		 while(!(SPI1->SR&SR_RXNE)){}
		 /*Read data from data register*/
		 *data++=(SPI1->DR);
		 size--;
	 }
 }
/*Pull low to enable*/
void cs_enable(void)
 {
	 GPIOA->ODR&=~CS;
 }
/*Pull high to disable*/
void cs_disable(void)
{
	 GPIOA->ODR|=CS;
}

















