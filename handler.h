/*
 * handler.h
 *
 *  Created on: Mar 8, 2015
 *      Author: morgan
 */

#ifndef HANDLER_H_
#define HANDLER_H_
// GLOBALS----------------------------------------

#ifdef __cplusplus
extern "C"
{
#endif
void ADC_Handler();
inline void dac_write(int val0, int val1);
inline void dac_write_0(int val0);
inline void dac_write_1(int val0);
#ifdef __cplusplus
}
#endif


#endif /* HANDLER_H_ */
