/*
 * spi.h
 *
 *  Created on: Jun 18, 2024
 *      Author: VOVAN
 */

#ifndef SPI_H_
#define SPI_H_
#include <stm32f411xe.h>
#include <stdint.h>

 void spi_receive(uint32_t* data, uint32_t size);
 void spi_transmit(uint32_t* data, uint32_t size);
 void spi_config(void);
 void spi_gpio_init(void);
 void cs_enable(void);
 void cs_disable(void);



#endif /* SPI_H_ */
