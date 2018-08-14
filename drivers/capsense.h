/**************************************************************************//**
 * @file
 * @brief Capacitive sense driver
 * @author Energy Micro AS
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012 Energy Micro AS, http://www.energymicro.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 4. The source and compiled code may only be used on Energy Micro "EFM32"
 *    microcontrollers and "EFR4" radios.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 *****************************************************************************/
#ifndef __CAPSENSE_H_
#define __CAPSENSE_H_

/***************************************************************************//**
 * @addtogroup Drivers
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup CapSense
 * @{
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define LESENSE_CHANNELS        16

#define SLIDER_PART0_CHANNEL    8
#define SLIDER_PART1_CHANNEL    9
#define SLIDER_PART2_CHANNEL    10
#define SLIDER_PART3_CHANNEL    11

uint32_t CAPSENSE_getVal(uint8_t channel);
uint32_t CAPSENSE_getNormalizedVal(uint8_t channel);
int32_t CAPSENSE_getSliderPosition(void);
void CAPSENSE_Init(void);
void CAPSENSE_setupLESENSE(bool sleep);
void CAPSENSE_setupCallbacks(void (*scanCb)(void), void (*chCb)(void));
void CAPSENSE_Sleep(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAPSENSE_H_ */

/** @} (end group CapSense) */
/** @} (end group Drivers) */
