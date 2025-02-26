/*
 * i2c.c
 */
#include "i2c.h"
#include <stm32l475xx.h>

void i2c_init() {
    // enable GPIOB for PB10 and PB11
    // enable I2C2
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

    // clear bits and set PB10 and PB11 to alternate function mdoe
    GPIOB->MODER &= ~((0b11 << (20)) | (0b11 << (22))); //20 bits for PB10, 22 bits for PB11
    GPIOB->MODER |= ((0b10 << (20)) | (0b10 << (22))); //20 bits for PB10, 22 bits for PB11

    // set AF4 for PB10 and PB11
    // 0100 = AF4
    GPIOB->AFR[1] &= ~((0b1111 << 8) | (0b1111 << 12)); // reset AFRH for PB10 and PB11
    GPIOB->AFR[1] |= ((0b0100 << 8) | (0b0100 << 12)); // PB10 uses [8:11], PB11 uses [12:15]

    //Set bit 10 and 11 to open drain
    GPIOB->OTYPER |= (0b1 << 10) | (0b1 << 11);
    GPIOB->OSPEEDR |= ((0b11 << 20) | (0b11 << 22));
    //Clear bit 20 - 23
    GPIOB->PUPDR &= ~((0b11 << 20) | (0b11 << 22));
    //Set 20 - 23 pull up mode
    GPIOB->PUPDR |= ((0b11 << 20) | (0b11 << 22));

    // configure I2C2
    //Tscl = 1/400000 = 2.5us
	//PRESC   1
	//SCLDEL  0x3
	//SDADEL  0x2
	//SCLH    0x3
	//SCLL    0x9
    I2C2->CR1 &= ~I2C_CR1_PE;
    I2C2->CR1 |= 1<<1;
    I2C2->TIMINGR = 0x1042C3C7; // 0x10320309 // 0x20420F13 // 0x1042C3C7 / 10KHz
    I2C2->CR1 |= I2C_CR1_PE;
}

uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len) {
    I2C2->CR2 = 0; 

    uint32_t cr2_value = 0;

	//bit 0 is R/W flag)
	cr2_value |= (address << 1);
	//number of bytes to transfer
	cr2_value |= (len << 16);
	// set the direction of transfer
	cr2_value |= ((dir & 0b1) << 10); // dir is 10th bit in CR register
	if (dir == 1) {
		cr2_value |= I2C_CR2_RD_WRN; // set to read mode
	}
	// Start
	cr2_value |= I2C_CR2_START; // start transaction
	I2C2->CR2 = cr2_value;

    if (dir == 0) { // writing
        for (uint8_t i = 0; i < len; i++) {
            //I2C_ISR_TXIS (bit 1 in ISR) show if the transmit data register is empty and ready for new data.
		    while (((I2C2->ISR & I2C_ISR_TXIS) == 0) &&
	                   ((I2C2->ISR & I2C_ISR_NACKF) == 0)); //Wait for TXIS flag

		    if (I2C2->ISR & I2C_ISR_NACKF) {
		       I2C2->CR2 |= I2C_CR2_STOP;
		       while ((I2C2->ISR & I2C_ISR_STOPF) == 0);
		       I2C2->ICR |= I2C_ICR_STOPCF;
		       return 1; 
		    }

		    I2C2->TXDR = data[i]; //Write data to TXDR
        }
    }
    else if (dir == 1) { // reading
        for (uint8_t i = 0; i < len; i++) {
		    while ((I2C2->ISR & I2C_ISR_RXNE) == 0); //Wait RXNE flag
		    data[i] = I2C2->RXDR; //Read from RXDR
		}
    }

    while((I2C2->ISR & I2C_ISR_TC) == 0); // wait for transfer to complete
    // set stop bit after writing/ reading has finished
    I2C2->CR2 |= I2C_CR2_STOP;
    while((I2C2->ISR & I2C_ISR_STOPF) == 0); // wait for stop bit
    I2C2->ICR |= I2C_ISR_STOPF; // clear stop bit in interrupt register
    return 0;
}
