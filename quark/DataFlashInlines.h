/**************************************************************************//**
 * @file DataFlashInlines.h
 * @brief File containing definitions for inline methods and functions for the
 * Atmel DataFlash Arduino library.
 *
 * @par Copyright: 
 * - Copyright (C) 2010-2011 by Vincent Cruz.
 * - Copyright (C) 2011 by Volker Kuhlmann. @n
 * All rights reserved.
 *
 * @authors
 * - Vincent Cruz @n
 *   cruz.vincent@gmail.com
 * - Volker Kuhlmann @n
 *   http://volker.top.geek.nz/contact.html
 *
 * @par Description:
 * Please refer to @ref DataFlash.cpp for more informations.
 *
 * @par History:
 * - Version 1.x, 2010-2011.
 * - Version 2.0, 30 Aug 2011.
 * - Version 2.2, 29 Dec 2011.
 *
 * @par Licence: GPLv3
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. @n
 * @n
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details. @n
 * @n
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/
 
/**
 * @addtogroup AT45DBxxxD
 * @{
 **/
#ifndef _DATAFALSHINLINES_
#define _DATAFALSHINLINES_


#include "mraa/gpio.h"
 
#define LOW 0
#define HIGH 1


inline void digitalWrite(mraa_gpio_context chipPin, int state)
{
	//mraa_gpio_context chipPin = mraa_gpio_init(pin);

    mraa_gpio_write ( chipPin, state);

    mraa_gpio_close(chipPin);
}
/**
 * Enable device with chipselect.
 **/
inline void enable()
{
	mraa_gpio_context chipSelectPin = mraa_gpio_init(m_chipSelectPin);
    digitalWrite(chipSelectPin, LOW);
    //mraa_gpio_write ( chipSelectPin, LOW);
}

/**
 * Disable device with chipselect.
 **/
inline void disable()
{
	mraa_gpio_context chipSelectPin = mraa_gpio_init(m_chipSelectPin);
    digitalWrite(chipSelectPin, HIGH);
}

/**
 * Enable write protection.
 **/
inline void writeProtect()
{
    if (m_writeProtectPin >= 0)
    {
    	mraa_gpio_context writeProtectPin = mraa_gpio_init(m_writeProtectPin);
        digitalWrite(writeProtectPin, LOW);
    }
}

/**
 * Disable write protection.
 **/
inline void readWrite()
{
    if (m_writeProtectPin >= 0)
    {
    	mraa_gpio_context writeProtectPin = mraa_gpio_init(m_writeProtectPin);
        digitalWrite(writeProtectPin, HIGH);
    }
}

/** Get chip Select (CS) pin **/
inline int8_t chipSelectPin()
{
    return m_chipSelectPin;
}

/** Get reset (RESET) pin **/
inline int8_t resetPin()
{
    return m_resetPin;
}

/** Get write protect (WP) pin **/
inline int8_t writeProtectPin()
{
    return m_writeProtectPin;
}

/**
 * Compute page address high byte.
 */
inline uint8_t pageToHiU8(uint16_t page)
{
    return page >> (16 - m_bufferSize);
}

/**
 * Compute page address low byte.
 */
inline uint8_t pageToLoU8(uint16_t page)
{
    return page << (m_bufferSize  - 8);
}

/**
 * Same as waitUntilReady
 * @todo This method will be removed.
 **/
inline void endAndWait()
{
    /* Wait for the end of the previous operation. */
    waitUntilReady();
}
/** @} **/
#endif
