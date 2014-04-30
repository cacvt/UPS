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

#include "device.h"

//==============================================================================
// Type definition
struct SPI {
	int ph;
};

//==============================================================================
// Public function declaration
void HW_spi_init(void);


//==============================================================================
// Public variables declaration
#ifndef __HW_SPI_C__
#endif /* __HW_SPI_C__ */

#endif /* __HW_SPI_H__ */
