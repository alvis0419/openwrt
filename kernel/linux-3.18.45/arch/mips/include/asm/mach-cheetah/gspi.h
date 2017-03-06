/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                           |
|                                                                              |
+=============================================================================*/
/*! 
*   \file 
*   \brief CAMELOT 
*   \author Montage
*/
#ifndef _GSPI_H_
#define _GSPI_H_

#define SPI_INIT_MASK       0xFFFE00C7 //bit 16~8, 5~3
#define SPI_DATA_INVERT     (1<<16)
#define SPI_CLK_W_OFFSET    8
#define SPI_RESET           (1<<7)        /* 1: reset 0: normal operation */
#define SPI_KEEP_CS         (1<<6)
#define SPI_CLK_PARK_HIGH   (1<<5)
#define SPI_THREE_WIRE      (1<<4)
#define SPI_FALLING_SAMPLE  (1<<3)
#define SPI_TXEMPTY         (1<<2)
#define SPI_TXFULL          (1<<1)
#define SPI_RXNONEMPTY      (1<<0)

/*
 * Function Prototypes
 */
int spi_ready(unsigned int idx);
int spi_read(unsigned int idx, unsigned int data_bit_len, unsigned char * rx_buf);
int spi_write(unsigned int idx, unsigned int data_bit_len, unsigned char * tx_buf);
void spi_keep_cs(unsigned int idx, int keep);
void spi_init(unsigned int idx, unsigned int init_val);
#endif /* _GSPI_H_ */
