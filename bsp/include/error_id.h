//==============================================================================
// Copyright © 2014 Virginia Tech Center for Power Electronics Systems
//
// Filename:     error_id.c
// Author:       Z Shen
// Description:
//   SPI-A hardware support for Ethernet module use only.
//   Not for general purpose use.
//==============================================================================
#ifndef __ERROR_ID_H__
#define __ERROR_ID_H__
typedef enum {
	ERR_OK = 0,
	ERR_MEM = -1,
	ERR_BUSY = -2,
	ERR_INVALID_PARAM = -3,
	ERR_TIMEOUT = -4
} ERR_ID;
#endif//__ERROR_ID_H__
