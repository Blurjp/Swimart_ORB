/**************************************************************************//**
 * @file DataFlash.cpp
 * @brief AT45DBxxxD Atmel Dataflash library for Arduino.
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

#include "DataFlash.h"
#include "DataFlashInlines.h"
#include "DataFlashCommands.h"
#include "mraa/spi.h"
#include "log_helper.h"
//#include <unistd.h>
#include "ble_app.h"


/**
 * @mainpage Atmel Dataflash library for Arduino.
 *
 * http://www.atmel.com/products/memories/sflash/dataflash.aspx
 **/

/**
 * @defgroup AT45DBxxxD Atmel Dataflash library for Arduino.
 * @{
 **/

//#define HIGH 0x1
//#define LOW  0x0

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#define INPUT 0x0
#define OUTPUT 0x1

mraa_spi_context spi;


 /**< @see AddressingInfos **/
 
/* No need to put this into PROGMEM */
static const struct AddressingInfos m_infos =
{
	/* 1   2   4   8  16  32  64 */
	{  9,  9,  9,  9, 10, 10, 11 },
	{  9, 10, 11, 12, 12, 13, 13 },
	{  2,  3,  3,  4,  4,  6,  5 }
};

/**
 * Constructor
 * This is not a good place to set the pins, because that causes hardware to be
 * initialised in the variable declaration part a long time before setup() is
 * called by the Arduino start-up code.

DataFlash()
{
}
**/
/** Destructor
~DataFlash()
{
}

**/

/**
 * Set pin use, with defaults for reset and write-protect if not
 * specified as argument.
 * Set SPI transfer speed to "low" (can be changed with .speed() ).
 * @note This must be called the first time; afterwards .begin() can
 *       be called without arguments.
 * @param csPin Chip select (Slave select) pin.
 * @param resetPin Reset pin, optional (default none).
 * @param wpPin Write protect pin, optional (default none).
 * **/
void setup(int8_t csPin, int8_t resetPin, int8_t wpPin)
{
	spi = mraa_spi_init(0);

	//m_chipSelectPin   = csPin;
	mraa_gpio_context m_chipSelectPin = mraa_gpio_init(csPin);
    //m_resetPin        = resetPin;
	mraa_gpio_context m_resetPin = mraa_gpio_init(resetPin);
    //m_writeProtectPin = wpPin;
	mraa_gpio_context m_writeProtectPin = mraa_gpio_init(wpPin);

    //pinMode(m_chipSelectPin, OUTPUT);
    if (mraa_gpio_dir (m_chipSelectPin, MRAA_GPIO_OUT) != MRAA_SUCCESS) {
    	pr_info (LOG_MODULE_MAIN, "GPIO dir failed!");
    }

    if (m_resetPin >= 0)
    {
        //pinMode(m_resetPin, OUTPUT);
    	mraa_gpio_dir (m_resetPin, MRAA_GPIO_OUT);
    	//digitalWrite(m_resetPin, HIGH); // set inactive
    	mraa_gpio_write (m_resetPin, HIGH);
    }
    if (m_writeProtectPin >= 0)
    {
        //pinMode(m_writeProtectPin, OUTPUT);
    	mraa_gpio_dir (m_writeProtectPin, MRAA_GPIO_OUT);
        //digitalWrite(m_writeProtectPin, HIGH); // set inactive
    	mraa_gpio_write (m_writeProtectPin, HIGH);
    }

    m_erase = ERASE_AUTO;
#ifdef AT45_USE_SPI_SPEED_CONTROL
    m_speed = SPEED_LOW;
#endif
        
    /* Clear pending SPI interrupts */
#ifdef __AVR__
    uint8_t clr;
    clr = SPSR;
    clr = SPDR;
    (void) clr; // Prevent variable set but unused warning. No code generated.
#endif


    /* Setup SPI */
    //SPI.setDataMode(SPI_MODE3);
    //mraa_spi_mode(spi, MRAA_SPI_MODE3);

    //SPI.setBitOrder(MSBFIRST);
    //mraa_spi_frequency());
    //SPI.setClockDivider(SPI_CLOCK_DIV2);

    /* Save SPI registers with settings for DataFlash for fast restore
     * in .begin(). */
    //m_SPSR = (SPSR & SPI_2XCLOCK_MASK);
    //m_SPCR = SPCR;
    
    /* Get DataFlash status register. */
    uint8_t stat;
    stat = status();

    /* Bit 3 of status register is ignored as it's always 1. Note that it is
     * equal to 0 on the obsolete chip with density higher than 64 MB. */
    uint8_t deviceIndex = ((stat & 0x38) >> 3) - 1; 
	m_bufferSize = m_infos.bufferSize[deviceIndex];
	m_pageSize   = m_infos.pageSize[deviceIndex];  
	m_sectorSize = m_infos.sectorSize[deviceIndex];
}

/** 
 * Initialise SPI interface for use with the %Dataflash,
 * allowing shared use with other SPI devices (which must however use
 * a different chipselect pin).

void begin()
{
    // Clear pending SPI interrupts?
    
    // Quick-load SPI registers for DataFlash use.
    SPCR = m_SPCR;
    SPSR = m_SPSR;
}

**/

/**
 * Disable device and restore SPI configuration.
 **/
void end()
{
    /* Disable device */
    disable();

    /* Don't call SPI.end() here to allow use of SPI interface with
    another chip. */
}

/**
 * Disable (deselect) %Dataflash, then enable (select) it again.
 **/
void reEnable()
{
    disable();
    enable();
}

/**
 * Set erase mode to automatic (default).
 **/
void autoErase()
{
    m_erase = ERASE_AUTO;
}

/**
 * Set erase mode to manual.
 * User must erase pages first, using one of the erase commands.
 **/
void manualErase()
{
    m_erase = ERASE_MANUAL;
}

/**
 * Set transfer speed (33MHz = low, 66MHz = high).
 * Note: Arduino supports 20MHz max, so using "high" is actually slower
 * because additional bytes have to be transferred for no benefit.
 **/
#ifdef AT45_USE_SPI_SPEED_CONTROL
void setTransferSpeed(IOspeed rate)
{
    m_speed = rate;
}

/**
 * Get transfer speed.
 **/
IOspeed getTransferSpeed() const
{
    return m_speed;
}
#endif // AT45_USE_SPI_SPEED_CONTROL

/**
 * Return whether the chip has completed the current operation and is
 * ready for the next.
 * Note that in some situations read/write access to one of the buffers
 * is permitted although the chip is busy.
 **/
uint8_t isReady()
{
    return (status() & AT45_READY);
}

/**
 * Wait until the chip is ready.
 **/
void waitUntilReady()
{
    /* Wait for the end of the transfer taking place. */
    while(!isReady()) {};
}

/** 
 * Read status register.
 * @return The content of the status register.
 * **/
uint8_t status()
{
    uint8_t status;

    reEnable();     // Reset command decoder.
  
    /* Send status read command */
    //SPI.transfer(DATAFLASH_STATUS_REGISTER_READ);
    mraa_spi_transfer_buf(spi, (uint8_t *)DATAFLASH_STATUS_REGISTER_READ, NULL, 1);

    /* Get result with a dummy write */
    //status = SPI.transfer(0);
    mraa_spi_transfer_buf(spi, (uint8_t *)0, &status, 1);

    disable();
    
    return (status);
}

/** 
 * Read Manufacturer and Device ID.
 * @note If id.extendedInfoLength is not equal to zero,
 *       successive calls to SPI.transfer() return
 *       the extended device information bytes.
 * @param id ID structure.
 **/
void readID(struct ID id)
{
    reEnable();     // Reset command decoder.

    /* Send status read command */
    //SPI.transfer(DATAFLASH_READ_MANUFACTURER_AND_DEVICE_ID);
    mraa_spi_transfer_buf(spi, (uint8_t *)DATAFLASH_READ_MANUFACTURER_AND_DEVICE_ID, NULL, 0);
    /* Manufacturer ID */
    //id.manufacturer = SPI.transfer(0);
    mraa_spi_transfer_buf(spi, (uint8_t *)(0), &(id.manufacturer), 1);
    /* Device ID (part 1) */
    //id.device[0] = SPI.transfer(0);
    mraa_spi_transfer_buf(spi, (uint8_t *)(0), &(id.device[0]), 1);
    /* Device ID (part 2) */
    //id.device[1] = SPI.transfer(0);
    mraa_spi_transfer_buf(spi, (uint8_t *)(0), &(id.device[1]), 1);
    /* Extended Device Information String Length */
    //id.extendedInfoLength = SPI.transfer(0);
    mraa_spi_transfer_buf(spi, (uint8_t *)(0), &(id.extendedInfoLength), 1);
    
    disable();
}

/**
 * A main memory page read allows the user to read data directly from
 * any one of the pages in the main memory, bypassing both of the
 * data buffers and leaving the contents of the buffers unchanged.
 * Reading past the end of the page wraps around to the beginning of
 * the page.
 * The chip must remain enabled by this function; it is the user's
 * responsibility to disable the chip when finished reading.
 * @param page Page of the main memory to read.
 * @param offset Starting byte address within the page (default value: 0).
 **/
void pageRead(uint16_t page, uint16_t offset)
{
    reEnable();     // Reset command decoder.
    
    /* Send opcode */
    //SPI.transfer(DATAFLASH_PAGE_READ);
    uint8_t tx_init[] = {DATAFLASH_PAGE_READ};


    mraa_spi_transfer_buf(spi, tx_init, NULL, sizeof(DATAFLASH_PAGE_READ));
    
    /* Address (page | offset)  */
    //SPI.transfer(pageToHiU8(page));
    //SPI.transfer(pageToLoU8(page) | (uint8_t)(offset >> 8));
    //SPI.transfer((uint8_t)(offset & 0xff));
    //uint8_t tx[] = {pageToHiU8(page)};
    //mraa_spi_transfer_buf(spi, tx, NULL, sizeof(tx));
    //mraa_spi_transfer_buf(spi, (uint8_t *)(pageToLoU8(page)| (uint8_t)(offset >> 8)), NULL, sizeof(uint8_t));
    //mraa_spi_transfer_buf(spi, (uint8_t *)(offset & 0xff), NULL, sizeof(uint8_t));
    
    uint8_t tx[] = {pageToHiU8(page), (pageToLoU8(page)|(uint8_t)(offset >> 8)), offset & 0xff};
    mraa_spi_transfer_buf_withindex(spi, tx, 0, NULL, 1);
    mraa_spi_transfer_buf_withindex(spi, tx, 1, NULL, 1);
    mraa_spi_transfer_buf_withindex(spi, tx, 2, NULL, 1);

    /* 4 "don't care" bytes */
    //SPI.transfer(0);
    //SPI.transfer(0);
    //SPI.transfer(0);
    //SPI.transfer(0);
    mraa_spi_transfer_buf(spi, (uint8_t *)(0), NULL, sizeof(0));
    mraa_spi_transfer_buf(spi, (uint8_t *)(0), NULL, sizeof(0));
    mraa_spi_transfer_buf(spi, (uint8_t *)(0), NULL, sizeof(0));
    mraa_spi_transfer_buf(spi, (uint8_t *)(0), NULL, sizeof(0));
    
    // Can't disable the chip here!
}

/**
 * Sequentially read a continuous stream of data at the currently set
 * speed. Reading past the end of the last page wraps around to the
 * beginning of the first page.
 * The chip must remain enabled by this function; it is the user's
 * responsibility to disable the chip when finished reading.
 * @param page Page of the main memory where the sequential read will
 * start.
 * @param offset Starting byte address within the page (default value: 0).
 * @note The legacy mode is not needed and not supported.
 **/
void arrayRead(uint16_t page, uint16_t offset)
{
    reEnable();     // Reset command decoder.

    /* Send opcode */
#ifdef AT45_USE_SPI_SPEED_CONTROL
    SPI.transfer(m_speed == SPEED_LOW ? DATAFLASH_CONTINUOUS_READ_LOW_FREQ :
                        DATAFLASH_CONTINUOUS_READ_HIGH_FREQ);
#else
    //SPI.transfer(DATAFLASH_CONTINUOUS_READ_LOW_FREQ);
    mraa_spi_transfer_buf(spi, (uint8_t *)(DATAFLASH_CONTINUOUS_READ_LOW_FREQ), NULL, sizeof(DATAFLASH_CONTINUOUS_READ_LOW_FREQ));
#endif

    /* Address (page | offset)  */
    //SPI.transfer(pageToHiU8(page));
    //SPI.transfer(pageToLoU8(page) | (uint8_t)(offset >> 8));
    //SPI.transfer((uint8_t)(offset & 0xff));
    uint8_t tx[] = {pageToHiU8(page), (pageToLoU8(page)|(uint8_t)(offset >> 8)), offset & 0xff};
    mraa_spi_transfer_buf_withindex(spi, tx, 0, NULL, 1);
    mraa_spi_transfer_buf_withindex(spi, tx, 1, NULL, 1);
    mraa_spi_transfer_buf_withindex(spi, tx, 2, NULL, 1);
    //mraa_spi_transfer_buf(spi, tx, NULL, sizeof(tx));
    //mraa_spi_transfer_buf(spi, (uint8_t *)(pageToLoU8(page)|(uint8_t)(offset >> 8)), NULL, 1);
    //mraa_spi_transfer_buf(spi, tx, NULL, sizeof(tx));



#ifdef AT45_USE_SPI_SPEED_CONTROL
    /* High frequency continuous read has an additional don't care byte. */
    if(m_speed != SPEED_LOW)
    {
        SPI.transfer(0x00);
    }
#endif
    
    // Can't disable the chip here!
}

/**
 * Read the content of one of the SRAM data buffer at the currently
 * set speed. Reading past the end of the buffer wraps around to the
 * beginning.
 * The chip must remain enabled by this function; it is the user's
 * responsibility to disable the chip when finished reading.
 * @param bufferNum Buffer to read (0 or 1).
 * @param offset Starting byte within the buffer (default value: 0).
 **/
void bufferRead(uint8_t bufferNum, uint16_t offset)
{
    /* Wait for the end of the previous operation. */
    waitUntilReady();
    
    reEnable();     // Reset command decoder.

    /* Send opcode */
#ifdef AT45_USE_SPI_SPEED_CONTROL
    if (bufferNum)
    {
        SPI.transfer((m_speed == SPEED_LOW) ? DATAFLASH_BUFFER_2_READ_LOW_FREQ :
                                              DATAFLASH_BUFFER_2_READ);
    }
    else
    {
        SPI.transfer((m_speed == SPEED_LOW) ? DATAFLASH_BUFFER_1_READ_LOW_FREQ :
                                              DATAFLASH_BUFFER_1_READ);

    }
#else
    //SPI.transfer(bufferNum ? DATAFLASH_BUFFER_2_READ_LOW_FREQ :
    //                         DATAFLASH_BUFFER_1_READ_LOW_FREQ);

    mraa_spi_transfer_buf(spi, (uint8_t *)(bufferNum ? DATAFLASH_BUFFER_2_READ_LOW_FREQ :
    	    DATAFLASH_BUFFER_1_READ_LOW_FREQ), NULL, 1);

#endif
    
    /* 14 "Don't care" bits */
    //SPI.transfer(0x00);
    mraa_spi_transfer_buf(spi, (uint8_t *)(0x00), NULL, 1);
    /* Rest of the "don't care" bits + bits 8,9 of the offset */
    //SPI.transfer((uint8_t)(offset >> 8));
    mraa_spi_transfer_buf(spi, (uint8_t *)(offset >> 8), NULL, 1);
    /* bits 7-0 of the offset */
    //SPI.transfer((uint8_t)(offset & 0xff));
    mraa_spi_transfer_buf(spi, (uint8_t *)(offset & 0xff), NULL, 1);
    
#ifdef AT45_USE_SPI_SPEED_CONTROL
    /* High frequency buffer read has an additional don't care byte. */
    if(m_speed != SPEED_LOW)
    {
        SPI.transfer(0x00);
    }
#endif
    
    // Can't disable the chip here!
}

/**
 * Write data to one of the SRAM data buffers at the currently set
 * speed. Writing past the end of the buffer wraps around to the
 * beginning.
 * The chip must remain enabled by this function; it is the user's
 * responsibility to disable the chip when finished reading.
 * @param bufferNum Buffer to read (0 or 1).
 * @param offset Starting byte within the buffer (default value: 0).
 **/
void bufferWrite(uint8_t bufferNum, uint16_t offset)
{
    /* Wait for the end of the previous operation. */
    waitUntilReady();
    
    reEnable();     // Reset command decoder.

    //SPI.transfer(bufferNum ? DATAFLASH_BUFFER_2_WRITE :
    //                         DATAFLASH_BUFFER_1_WRITE);
    mraa_spi_transfer_buf(spi,(uint8_t *)(bufferNum ? DATAFLASH_BUFFER_2_WRITE :
            DATAFLASH_BUFFER_1_WRITE), NULL, 1);
    
    /* 14 "Don't care" bits */
    //SPI.transfer(0x00);
    mraa_spi_transfer_buf(spi, (uint8_t *)(0x00), NULL, 1);
    /* Rest of the "don't care" bits + bits 8,9 of the offset */
    //SPI.transfer((uint8_t)(offset >> 8));
    mraa_spi_transfer_buf(spi, (uint8_t *)(offset >> 8), NULL, 1);
    /* bits 7-0 of the offset */
    //SPI.transfer((uint8_t)(offset & 0xff));
    mraa_spi_transfer_buf(spi, (uint8_t *)(offset & 0xff), NULL, 1);
    // Can't disable the chip here!
}

/**
 * Transfer data from buffer 0 or 1 to a main memory page, erasing the
 * page first if auto-erase is set. If erase is manual, the page must
 * have been erased previously using one of the erase commands.
 * @param bufferNum Buffer to use (0 or 1).
 * @param page Page to which the content of the buffer is written.
 **/
void bufferToPage(uint8_t bufferNum, uint16_t page)
{
    /* Wait for the end of the previous operation. */
    waitUntilReady();

    reEnable();

    /* Opcode */
    if (m_erase == ERASE_AUTO)
    {
        //SPI.transfer(bufferNum ? DATAFLASH_BUFFER_2_TO_PAGE_WITH_ERASE :
        //                         DATAFLASH_BUFFER_1_TO_PAGE_WITH_ERASE);
        mraa_spi_transfer_buf(spi, (uint8_t *)(bufferNum ? DATAFLASH_BUFFER_2_TO_PAGE_WITH_ERASE :
                DATAFLASH_BUFFER_1_TO_PAGE_WITH_ERASE), NULL, sizeof(bufferNum));
    }
    else
    {
        //SPI.transfer(bufferNum ? DATAFLASH_BUFFER_2_TO_PAGE_WITHOUT_ERASE :
        //                         DATAFLASH_BUFFER_1_TO_PAGE_WITHOUT_ERASE);
        mraa_spi_transfer_buf(spi,(uint8_t *)(bufferNum ? DATAFLASH_BUFFER_2_TO_PAGE_WITHOUT_ERASE :
                DATAFLASH_BUFFER_1_TO_PAGE_WITHOUT_ERASE), NULL, sizeof(bufferNum));
    }
    
    /* see pageToBuffer
    //SPI.transfer(pageToHiU8(page));
    //SPI.transfer(pageToLoU8(page));
    //SPI.transfer(0x00);
    mraa_spi_transfer_buf(spi, &(pageToHiU8(page)), NULL, 0);
    mraa_spi_transfer_buf(spi, &(pageToLoU8(page)), NULL, 0);
    mraa_spi_transfer_buf(spi, &(0x00), NULL, 0);
    */
    uint8_t tx[] = {pageToHiU8(page), pageToLoU8(page), 0x00};

    mraa_spi_transfer_buf_withindex(spi, tx, 0, NULL, 1);
    mraa_spi_transfer_buf_withindex(spi, tx, 1, NULL, 1);
    mraa_spi_transfer_buf_withindex(spi, tx, 2, NULL, 1);
    
    /* Start transfer. If erase was set to automatic, the page will first be
    erased. The chip remains busy until this operation finishes. */
    disable();
}

/**
 * Transfer a page of data from main memory to buffer 0 or 1.
 * @param page Main memory page to transfer.
 * @param bufferNum Buffer (0 or 1) to which the data is written.
 **/
void pageToBuffer(uint16_t page, uint8_t bufferNum)
{
    /* Wait for the end of the previous operation. */
    waitUntilReady();

    reEnable();

    /* Send opcode */
    //SPI.transfer(bufferNum ? DATAFLASH_TRANSFER_PAGE_TO_BUFFER_2 :
    //                         DATAFLASH_TRANSFER_PAGE_TO_BUFFER_1);
    mraa_spi_transfer_buf(spi, (uint8_t *)(bufferNum ? DATAFLASH_TRANSFER_PAGE_TO_BUFFER_2 :
    	                             DATAFLASH_TRANSFER_PAGE_TO_BUFFER_1), NULL, sizeof(uint8_t));


    /* Output the 3 bytes adress.
     * For all DataFlashes 011D to 642D the number of trailing don't care bits
     * is equal to the number of page bits plus 3 (a block consists of 8 (1<<3)
     * pages), and always larger than 8 so the third byte is always 0. */
    //SPI.transfer(pageToHiU8(page));
    //SPI.transfer(pageToLoU8(page));
    //SPI.transfer(0);
    //mraa_spi_transfer_buf(spi, &(pageToHiU8(page)), NULL, 0);
    //mraa_spi_transfer_buf(spi, &(pageToLoU8(page)), NULL, 0);
    //mraa_spi_transfer_buf(spi, &(0x00), NULL, 0);
    uint8_t tx[] = {pageToHiU8(page), pageToLoU8(page), 0x00};

    mraa_spi_transfer_buf_withindex(spi, tx, 0, NULL, 1);
    mraa_spi_transfer_buf_withindex(spi, tx, 1, NULL, 1);
    mraa_spi_transfer_buf_withindex(spi, tx, 2, NULL, 1);
    //mraa_spi_transfer_buf(spi, (uint8_t *)(0), NULL, 1);
    /* Start transfer. The chip remains busy until this operation finishes. */
    disable();
}

/** 
 * Erase a page in the main memory array.
 * @param page Page to erase.
 **/
void pageErase(uint16_t page)
{
    /* Wait for the end of the previous operation. */
    waitUntilReady();

    reEnable();
    
    /* Send opcode */
    //SPI.transfer(DATAFLASH_PAGE_ERASE);
    mraa_spi_transfer_buf(spi, (uint8_t *)(DATAFLASH_PAGE_ERASE), NULL, 1);
    
    /* see pageToBuffer
    SPI.transfer(pageToHiU8(page));
    SPI.transfer(pageToLoU8(page));
    SPI.transfer(0x00);

    mraa_spi_transfer_buf(spi, &(pageToHiU8(page)), NULL, sizeof(uint8_t));
    mraa_spi_transfer_buf(spi, &(pageToLoU8(page)), NULL, sizeof(uint8_t));
    mraa_spi_transfer_buf(spi, &(0x00), NULL, sizeof(0x00));
    */
    uint8_t tx[] = {pageToHiU8(page), pageToLoU8(page), 0x00};
    mraa_spi_transfer_buf_withindex(spi, tx, 0, NULL, 1);
    mraa_spi_transfer_buf_withindex(spi, tx, 1, NULL, 1);
    mraa_spi_transfer_buf_withindex(spi, tx, 2, NULL, 1);

    /* Start page erase. The chip remains busy until this operation finishes. */
    disable();
}

/**
 * Erase a block of pages in a single operation.
 * @param block Block to erase.
 * @warning UNTESTED
 **/
void blockErase(uint16_t block)
{
    /* Wait for the end of the previous operation. */
    waitUntilReady();

    reEnable();

    /* Send opcode */
    //SPI.transfer(DATAFLASH_BLOCK_ERASE);
    mraa_spi_transfer_buf(spi, (uint8_t *)(DATAFLASH_BLOCK_ERASE), NULL, sizeof(DATAFLASH_BLOCK_ERASE));

    /* Output the 3 bytes adress.
     * For all DataFlashes 011D to 642D the number of trailing don't care bits
     * is equal to the number of page bits plus 3 (a block consists of 8 (1<<3)
     * pages), and always larger than 8 so the third byte is always 0. */
    uint8_t rightShift = m_bufferSize + 3 - 8;
    block >>= rightShift;
    //SPI.transfer(highByte(block));
    //SPI.transfer(lowByte(block));
    //SPI.transfer(0x00);
    //mraa_spi_transfer_buf(spi, &(highByte(block)), NULL, sizeof(uint8_t));
    //mraa_spi_transfer_buf(spi, &(lowByte(block)), NULL, sizeof(uint8_t));
    //mraa_spi_transfer_buf(spi, &(0x00), NULL, sizeof(0x00));
    uint8_t tx[] = {highByte(block), lowByte(block), 0x00};

    mraa_spi_transfer_buf_withindex(spi, tx, 0, NULL, 1);
    mraa_spi_transfer_buf_withindex(spi, tx, 1, NULL, 1);
    mraa_spi_transfer_buf_withindex(spi, tx, 2, NULL, 1);
    //mraa_spi_transfer_buf(spi, (uint8_t *)(0), NULL, sizeof(uint8_t));

    /* Start block erase.
    The chip remains busy until this operation finishes. */
    disable();
}

/**
 * Erase a sector of blocks in a single operation.
 * @param sector Sector to erase.
 **/
void sectorErase(int8_t sector)
{
    /* Wait for the end of the previous operation. */
    waitUntilReady();

    reEnable();

    /* Send opcode */
    //SPI.transfer(DATAFLASH_SECTOR_ERASE);
    mraa_spi_transfer_buf(spi, (uint8_t *)(DATAFLASH_SECTOR_ERASE), NULL, sizeof(DATAFLASH_SECTOR_ERASE));


    if((sector == AT45_SECTOR_0A) || (sector == AT45_SECTOR_0B))
    {
        //SPI.transfer(0x00);
        mraa_spi_transfer_buf(spi, (uint8_t *)(0x00), NULL, sizeof(0x00));
        //SPI.transfer((static_cast<uint8_t>(-sector) & 0x01) << (m_bufferSize - 5));
        mraa_spi_transfer_buf(spi, (uint8_t *)(((uint8_t)(-sector) & 0x01) << (m_bufferSize - 5)), NULL, sizeof(uint8_t));
    }
    else
    {
        uint8_t shift = m_bufferSize + m_pageSize - m_sectorSize - 16;
        //SPI.transfer(sector << shift);
        mraa_spi_transfer_buf(spi, (uint8_t *)(sector << shift), NULL, sizeof(int8_t));
        //SPI.transfer(0x00);
        mraa_spi_transfer_buf(spi, (uint8_t *)(0x00), NULL, sizeof(0x00));
    }

	//SPI.transfer(0x00);
    mraa_spi_transfer_buf(spi, (uint8_t *)(0x00), NULL, sizeof(0x00));

    /* Start sector erase.
    The chip remains busy until this operation finishes. */
    disable();
}

#ifdef AT45_CHIP_ERASE_ENABLED
/**
 * Erase the entire chip memory. Sectors protected or locked down will
 * not be erased.
 * @warning UNTESTED
 **/
void chipErase()
{
    uint8_t stat = status();
    uint8_t deviceIndex = ((stat & 0x38) >> 3) - 1; /// \todo Store this at init

    uint8_t sectorCount = 1 << m_infos.sectorSize[deviceIndex];

    sectorErase(AT45_SECTOR_0A);
    sectorErase(AT45_SECTOR_0B);
    for(uint8_t i=1; i<sectorCount; i++)
    {
        sectorErase(i);
    }

#if 0
    /* DO NOT USE THIS CODE!       */
    /* MAY DAMAGE CHIP.            */
    /* READ DATASHEET FOR DETAILS. */

    /* Wait for the end of the previous operation. */
    waitUntilReady();

    enable();

    /* Send chip erase sequence */
    SPI.transfer(DATAFLASH_CHIP_ERASE_0);
    SPI.transfer(DATAFLASH_CHIP_ERASE_1);
    SPI.transfer(DATAFLASH_CHIP_ERASE_2);
    SPI.transfer(DATAFLASH_CHIP_ERASE_3);

    /* Start chip erase.
    The chip remains busy until this operation finishes. */
    disable();
#else
    (void) sector;
#endif
}
#endif // AT45_CHIP_ERASE_ENABLED

/**
 * This a combination of Buffer Write and Buffer to Page with
 * Built-in Erase.
 * To perform a Main Memory Page Program through Buffer using the standard DataFlash page size (264 bytes),
 *  an opcode of 82h for Buffer 1 or 85h for Buffer 2 must first be clocked into the device followed
 *  by three address bytes comprised of 15 page address bits (PA14 - PA0) that specify the page in
 *  the main memory to be written, and 9 buffer address bits (BFA8 - BFA0) that select the first byte
 *  in the buffer to be written.
 * The global erase flag .manual_erase() is ignored.
 * Writing past the end of the page wraps around to the beginning of
 * the page.
 * @note You must call endAndWait in order to start transferring data
 * from buffer to page.
 * @param page Page to which the content of the buffer is written.
 * @param offset Starting byte address within the buffer.
 * @param bufferNum Buffer to use (0 or 1).
 **/
void beginPageWriteThroughBuffer(
        uint16_t page, uint16_t offset, uint8_t bufferNum)
{
    reEnable();     // Reset command decoder.

    /* Send opcode */
    //SPI.transfer(bufferNum ? DATAFLASH_PAGE_THROUGH_BUFFER_2 :
    //                         DATAFLASH_PAGE_THROUGH_BUFFER_1);
    mraa_spi_transfer_buf(spi, (uint8_t *)(bufferNum ? DATAFLASH_PAGE_THROUGH_BUFFER_2 :
            DATAFLASH_PAGE_THROUGH_BUFFER_1), NULL, 1);

    /* Address */
    //SPI.transfer(pageToHiU8(page));
    //SPI.transfer(pageToLoU8(page) | (uint8_t)(offset >> 8));
    //SPI.transfer((uint8_t)(offset & 0xff));
    uint8_t tx[] = {pageToHiU8(page)};
    mraa_spi_transfer_buf(spi, tx, NULL, 1);

    mraa_spi_transfer_buf(spi, (uint8_t *)(pageToLoU8(page)|(uint8_t)(offset >> 8)), NULL, 1);
    mraa_spi_transfer_buf(spi, (uint8_t *)(offset & 0xff), NULL, 1);
}

/**
 * Compare a page of data in main memory to the data in buffer 0 or 1.
 * @param page Page to compare.
 * @param bufferNum Buffer number (0 or 1).
 * @return
 *      - true  If the page and the buffer contains the same data.
 *      - false Otherwise.
 **/
int8_t isPageEqualBuffer(uint16_t page, uint8_t bufferNum)
{
    reEnable();     // Reset command decoder.

    /* Send opcode */
    //SPI.transfer(bufferNum ? DATAFLASH_COMPARE_PAGE_TO_BUFFER_2 :
    //                         DATAFLASH_COMPARE_PAGE_TO_BUFFER_1);

    mraa_spi_transfer_buf(spi, (uint8_t *)(bufferNum ? DATAFLASH_COMPARE_PAGE_TO_BUFFER_2 :
            DATAFLASH_COMPARE_PAGE_TO_BUFFER_1), NULL, sizeof(uint8_t));


    /* Page address */

    //SPI.transfer(pageToHiU8(page));
    //SPI.transfer(pageToLoU8(page));
    uint8_t tx[] = {pageToHiU8(page), pageToLoU8(page)};
    mraa_spi_transfer_buf_withindex(spi, tx, 0, NULL, 1);
    mraa_spi_transfer_buf_withindex(spi, tx, 1, NULL, 1);

    //SPI.transfer(0x00);

    disable();  /* Start comparison */

    /* Wait for the end of the comparison. */
    waitUntilReady();

    /* If bit 6 of the status register is 0 then the data in the
     * main memory page matches the data in the buffer.
     * If it's 1 then the data in the main memory page doesn't match.
     */
    return ((status() & AT45_COMPARE) == 0);
}

/**
 * Put the device into the lowest power consumption mode.
 * Once the device has entered the Deep Power-down mode, all
 * instructions are ignored except the Resume from Deep
 * Power-down command.
 * @warning UNTESTED
 **/
void deepPowerDown()
{
    reEnable();     // Reset command decoder.

    /* Send opcode */
    //SPI.transfer(DATAFLASH_DEEP_POWER_DOWN);
    mraa_spi_transfer_buf(spi, (uint8_t *)DATAFLASH_DEEP_POWER_DOWN, NULL, sizeof(uint8_t));

    /* Enter Deep Power-Down mode */
    disable();
}

/**
 * Takes the device out of Deep Power-down mode.
 **/
void resumeFromDeepPowerDown()
{
    reEnable();     // Reset command decoder.

     //Send opcode
    //SPI.transfer(DATAFLASH_RESUME_FROM_DEEP_POWER_DOWN);
    mraa_spi_transfer_buf(spi, (uint8_t *)DATAFLASH_RESUME_FROM_DEEP_POWER_DOWN, NULL, 1);

    //Resume device
    disable();
    /**
    The CS pin must stay high during t_RDPD microseconds before the device
    * can receive any commands.
    * On the at45db161D t_RDPD = 35 microseconds.
    * Wait 40us (just to be sure).
    **/
    //delayMicroseconds(40);
    usleep(40);
}


void handle_sensor_data_record(uint8_t* accel_cali_data_buf, uint16_t* pageCount, uint16_t* pageOffSet, uint8_t tSize)
{

		/*  write the data into memory chip							 */
		if(*pageOffSet+tSize < DF_45DB642_PAGESIZE)
		{
			WriteIntoDevice(accel_cali_data_buf, 0, tSize,  *pageCount, *pageOffSet);
			*pageOffSet += tSize;
		}
		else
		{
			WriteIntoDevice(accel_cali_data_buf, 0, DF_45DB642_PAGESIZE-(*pageOffSet),  *pageCount, *pageOffSet);
			*pageCount += 1;
			int16_t size = *pageOffSet+tSize-DF_45DB642_PAGESIZE;
			*pageOffSet = 0;
			WriteIntoDevice(accel_cali_data_buf, 0, size,  *pageCount, *pageOffSet);
		}

}

int ret = 0;
/**
 * Write data into device
 * @param val Date to write into device
 * @param buffer Buffer number (0 or 1).
 * @param size Data length
 * @param pageCount Page of the main memory to read.
 * @param offset Starting byte address within the buffer.
**/
    void WriteIntoDevice(uint8_t* val, uint8_t buffer, uint16_t size, uint16_t pageCount, uint16_t offset)
{
    	//beginPageWriteThroughBuffer(page, offset, buffer);
    	bufferWrite(buffer, 0);
    	ret = 0;
    	for(uint16_t i=0; i<sizeof(val); i++)
    	{
    		if((ret = mraa_spi_transfer_buf(spi, &val[i], NULL, 1))!=0)
    		{
    			pr_error(LOG_MODULE_BLE, "WriteIntoDevice failed: %d.", ret);
    			return;
    		}
    	}
    	//this will auto erase the buffer if auto_erase is enabled
    	bufferToPage(buffer, pageCount);
}

    /**
     * Read data from Data flash
     * @param data Date read from flash
     * @param pageIndex Page of the main memory to read.
     * @param buffer Buffer number (0 or 1).
    **/
    int ReadFromDevice(uint8_t* data, uint16_t pageIndex, uint8_t bufferNum)
{
    	uint8_t rxbuf[1];
    	//uint16_t readPageIndex = 0;
    	uint16_t readOffSet = 0;
    	//uint16_t limit = DF_45_BUFFER_SIZE;
    	uint8_t count = 0;
    	ret = 0;

    		pageToBuffer(pageIndex, bufferNum);
    		bufferRead(bufferNum, readOffSet);
    	    do
    		{
    			if((ret = mraa_spi_transfer_buf(spi, (uint8_t *)(0xff), rxbuf, 1))!=0)
    			{
    			    pr_error(LOG_MODULE_BLE, "ReadFromDevice failed: %d.", ret);
    			    return ret;
    			}
    			if(*rxbuf != '\0')
    			{
    			data[count++] = *rxbuf;
    			}

    		}while((*rxbuf != '\0') && (count < DF_45_BUFFER_SIZE));
    	return ret;
}

/**
 * Reset device via the reset pin.
 * If no reset pint was specified (with begin()), this does nothing.
**/

void hardReset()
{
    if (m_resetPin >= 0)
    {
        digitalWrite(m_resetPin, LOW);

        /**
         *The reset pin should stay low for at least 10us (table 18.4).
        **/
        //delayMicroseconds(10);
        usleep(10);

        /**
         According to the Dataflash spec (21.6 Reset Timing),
         * the CS pin should be in high state before RESET
         * is deasserted (ie HIGH).
        **/
        disable();

        /**
         *Just to be sure that the high state is reached
        **/
        //delayMicroseconds(1);
        usleep(1);

        digitalWrite(m_resetPin, HIGH);

        /**
         Reset recovery time = 1us
        **/
        //delayMicroseconds(1);
        usleep(1);
    }
}


/**
 * @}
 **/
