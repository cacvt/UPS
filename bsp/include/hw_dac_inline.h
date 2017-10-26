/*
 * hw_dac_inline.h
 *
 *  Created on: Nov 19, 2013
 *      Author: shenzy
 */

#ifndef __HW_DAC_INLINE_H__
#define __HW_DAC_INLINE_H__

#include "device.h"

/*******************************************************************************
 * Public inline function definition
 ******************************************************************************/
inline void HW_dac_store_ch(int ch, int data)
{
	dac.data_count++;													/* Increase data count */
	dac.data[2 * dac.dataptr] = DAC_STORE |  (ch & 0x3) << 1;			/* DAC channel to load */
	dac.data[2 * dac.dataptr + 1] = data;								/* DAC value to load */
	dac.data_count = __min(dac.data_count, 4);							/* Limit data count to be maximum 4 */
	dac.dataptr = (dac.dataptr + 1) & 0x03;								/* Increase data pointer, loop within the array */
}

inline int HW_dac_load(void)
{
	/* Return error if data is being transfered */
	if (DAC_BUSY_TANSFER)
		return DAC_ERR_BUSY;

	/* Return no error if there is no data to send */
	if (dac.data_count == 0)
		return DAC_NO_ERR;

	EALLOW;
	/* Setup DMA transfer length */
	DmaRegs.CH3.TRANSFER_SIZE = dac.data_count - 1;
	DmaRegs.CH3.MODE.bit.PERINTE = PERINT_ENABLE;
	/* Start DMA transfer */
	DmaRegs.CH3.CONTROL.bit.PERINTFRC = 1;
	EDIS;

	dac.data_count = 0;
	dac.dataptr = 0;

	return DAC_NO_ERR;

}

inline int HW_dac_load_update(void)
{
	int res;

	dac.data[2 * dac.data_count-2] = DAC_UPDATE_SIM | (0x07 & dac.data[2 * dac.data_count-2]);

	res = HW_dac_load();

	if (res != DAC_NO_ERR)
		dac.data[2 * dac.data_count-2] = DAC_STORE | (0x07 & dac.data[2 * dac.data_count-2]);

	return res;

}

inline int HW_dac_load_ch(int ch, int data)
{
	/* Return error if data is being transfered */
	if (DAC_BUSY_TANSFER)
		return DAC_ERR_BUSY;

	McbspbRegs.DXR2.all = DAC_STORE | ((0x03 & ch ) << 1);
	McbspbRegs.DXR1.all = data;

	return DAC_NO_ERR;
}

inline int HW_dac_update(void)
{
	/* Return error if data is being transfered */
	if (DAC_BUSY_TANSFER)
		return DAC_ERR_BUSY;

	 GpioDataRegs.GPADAT.bit.GPIO25 = 1;
	 GpioDataRegs.GPADAT.bit.GPIO25 = 0;

	 return DAC_NO_ERR;
}


inline int HW_dac_update_ch(int ch, int data)
{
	/* Return error if data is being transfered */
	if (DAC_BUSY_TANSFER)
		return DAC_ERR_BUSY;

	McbspbRegs.DXR2.all = DAC_UPDATE | ((0x03 & ch ) << 1);
	McbspbRegs.DXR1.all = data;

	return DAC_NO_ERR;
}

inline int HW_dac_update_all(int data)
{
	/* Return error if data is being transfered */
	if (DAC_BUSY_TANSFER)
		return DAC_ERR_BUSY;

	McbspbRegs.DXR2.all = DAC_UPDATE_ALL;
	McbspbRegs.DXR1.all = data;

	return DAC_NO_ERR;
}
#endif /* __HW_DAC_INLINE_H__ */
