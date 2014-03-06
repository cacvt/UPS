/*******************************************************************************
 * Copyright © 2014 Virginia Tech Center for Power Electronics Systems
 *
 * Filename:     hw_spi.h
 * Author:       Z Shen
 * Description:
 *   SPI-A hardware support for Ethernet module use only.
 *   Not for general purpose use.
 ******************************************************************************/
#ifndef __HW_SPI_H__
#define __HW_SPI_H__

typedef interrupt void(*ISR_FUNC)(void);

void HW_spi_init(void);


/* Export globale variables defined in the c file */
#ifndef __HW_SPI_C__
#endif /* __HW_SPI_C__ */


#endif /* __HW_SPI_H__ */
