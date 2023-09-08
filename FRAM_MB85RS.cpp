/**************************************************************************/
/*!
    @file     FRAM_MB85RS.cpp
    @author   Ivan Labáth
              Christophe Persoz
    @license  BSD (see license.txt)

    Driver for the MB85RS SPI FRAM series from Fujitsu.

    @section  HISTORY

    v0.8
      - use generic read/write
        Note: all data now native endian
      - fix memory addressing
      - fix 32-bit read
    v0.7 - See ReadMe for more informations

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, SOSAndroid.fr (E. Ha.)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/

#include <SPI.h>
#include <FRAM_MB85RS.h>

/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/


/*!
///     @brief   FRAM_MB85RS()
///              Constructor without write protection management
///     @param   cs, chip select pin - active low
**/
FRAM_MB85RS::FRAM_MB85RS(SPIClass & spi, uint8_t cs)
{
    _spi = &spi;
    _cs = cs;
    _wp = false; // No WP pin connected, WP management inactive

    _framInitialised = false;
}



/*!
///     @brief   FRAM_MB85RS()
///              Constructor with write protection pin
///     @param   cs, chip select pin - active low
///     @param   wp, write protected pin - active low
**/
FRAM_MB85RS::FRAM_MB85RS(SPIClass & spi, uint8_t cs, uint8_t wp)
{
    _cs = cs;

    _wp = true; // WP pin connected and Write Protection enabled
    _wpPin = wp;

    // The init WP management status is define under DEFAULT_WP_STATUS
    DEFAULT_WP_STATUS ? enableWP() : disableWP();

    _framInitialised = false;
}



/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/


/*!
///     @brief   begin_tree()
///              Initialize the CS pin and SPI bus, then
///              Inititalize the F-RAM chip - detect model and size
**/
void FRAM_MB85RS::begin_tree()
{
    _csCONFIG();
    _spi->begin();
    begin();
}



/*!
///     @brief   begin()
///              Inititalize the F-RAM chip - detect model and size
///              Note: we do not initialize SPI nor the CS pin
///     @return  if DEBUG_TRACE, provides all the informations on the chip
**/
void FRAM_MB85RS::begin()
{
    digitalWriteFast(_cs, HIGH);
    delayMicroseconds(1);

    bool deviceFound = identify();

#if defined(DEBUG_TRACE) || defined(CHIP_TRACE)
    if (!Serial)
        Serial.begin(115200);
    while (!Serial) {}

    Serial.println("FRAM_MB85RS created\n");
    Serial.print("Write protect management: ");
    if (_wp)
        Serial.println("active");
    else
        Serial.println("inactive");

    if (deviceFound) {
        Serial.println("Memory Chip initialized");
        _deviceID2Serial();
    }
    else
        Serial.println("ERROR : Memory Chip NOT FOUND\n");
#endif
}



/*!
///     @brief   identify()
///              Check if the device is connected
///     @return  0: device not found
///              1: device connected
**/
bool FRAM_MB85RS::identify()
{
    bool result = _getDeviceID();

    if (result && _manufacturer == FUJITSU_ID && _maxaddress != 0) {
        _framInitialised = true;
        return true;
    }

    _framInitialised = false;
    return false;
}



/*!
///     @brief   readBuf()
///              Read an array made of 8-bits values from the specified F-RAM address
///     @param   addr, the memory address to read from
///     @param   buf, the array of 8-bits value to read
///     @param   size, the number of elements to read
///     @return  0: error
///              1: ok
///     @note    F-RAM provide a continuous reading with auto-increment of the address
**/
bool FRAM_MB85RS::readBuf(uint32_t addr, void * buf, uint32_t size)
{
    if ( addr >= _maxaddress
        || ((addr + size - 1) >= _maxaddress)
        || size == 0
        || !_framInitialised )
        return false;

    uint8_t * mem = (uint8_t *) buf;

    _spi_begin();
    // Read byte operation
    _spi->transfer(FRAM_READ);
    _sendAddr(addr);

    // Read values
    for (uint32_t i = 0; i < size; ++i) {
        mem[i] = _spi->transfer(0);
#ifdef DEBUG_TRACE
        Serial.print("Adr 0x"); Serial.print(addr+i, HEX);
        Serial.print(", Value[");Serial.print(i); Serial.print("] = 0x"); Serial.println(mem[i], HEX);
#endif
    }

    _spi_end();

    _lastaddress = addr + size - 1;

    return true;
}



/*!
///     @brief   writeBuf()
///              Write an array made of 8-bits values from the specified F-RAM address
///     @param   addr, the memory address to write from
///     @param   buf, the array of 8-bits value to write
///     @param   size, the number of elements to write
///     @return  0: error
///              1: ok
///     @note    F-RAM provide a continuous writing with auto-increment of the address
**/
bool FRAM_MB85RS::writeBuf(uint32_t addr, const void * buf, uint32_t size)
{
    if ( addr >= _maxaddress
        || ((addr + size - 1) >= _maxaddress)
        || size == 0
        || !_framInitialised )
        return false;

    const uint8_t * mem = (const uint8_t *) buf;

    // Set Memory Write Enable Latch
    _spi_begin();
        _spi->transfer(FRAM_WREN);
    _spi_end();

    // Write byte operation
    _spi_begin();
        _spi->transfer(FRAM_WRITE);
        _sendAddr(addr);
        // Write values
        for (uint32_t i = 0; i < size; ++i)
            _spi->transfer(mem[i]);
    _spi_end();

    // Reset Memory Write Enable Latch
    _spi_begin();
        _spi->transfer(FRAM_WRDI);
    _spi_end();

    _lastaddress = addr + size - 1;

    return true;
}



/*!
///    @brief   isAvailable()
///             Returns the readiness of the memory chip
///    @return  0: ready
///             1: unavailable
**/
bool FRAM_MB85RS::isAvailable()
{
    if ( _framInitialised && digitalReadFast(_cs) == HIGH )
        return true;

    return false;
}



/*!
///    @brief   getWPStatus()
///             Returns the Write Protect status
///    @return  0: WP is disable
///             1: WP is enable
**/
bool FRAM_MB85RS::getWPStatus()
{
    return _wpStatus;
}



/*!
///    @brief   enableWP()
///             Enable write protect function of the chip by pulling up WP pin
///    @return  0: error, WP is not managed
///             1: success, WP is enable
**/
bool FRAM_MB85RS::enableWP(void)
{
    if (_wp) {
        digitalWriteFast(_wpPin,HIGH);
        _wpStatus = true;
        return true;
    }

    return false;
}



/*!
///    @brief   disableWP()
///             Disable write protect function of the chip by pulling down WP pin
///    @return  0: error, WP is not managed
///             1: success, WP is disable
**/
bool FRAM_MB85RS::disableWP()
{
    if (_wp) {
        digitalWriteFast(_wpPin,LOW);
        _wpStatus = false;
        return true;
    }

    return false;
}


/*!
///    @brief   eraseChip()
///             Erase chip by overwriting it to 0x00
///             Output on Serial if active
///    @return  0: error
///             1: ok
**/
bool FRAM_MB85RS::eraseChip()
{
    if ( !_framInitialised )
        return false;

    uint32_t i = 0;
    bool result = true;

#ifdef DEBUG_TRACE
    Serial.println("Start erasing device");
#endif

    while( i < _maxaddress && result )
        result = write(i++, (uint8_t)0);

#ifdef DEBUG_TRACE
    if ( !result ) {
        Serial.print("ERROR: Device erasing stopped at position ");
        Serial.println(i-1, DEC);
    } else
        Serial.print("Erased from address 0x00 to 0x"); Serial.println(i-1, HEX);
    Serial.println("Device erased!");
#endif

    _lastaddress = _maxaddress;

    return result;
}


/*!
///    @brief   getMaxMemAdr()
///             Return the maximum memory address available
///    @return  _maxaddress
**/
uint32_t FRAM_MB85RS::getMaxMemAdr()
{
    return _maxaddress;
}



/*!
 ///    @brief   getLastMemAdr()
 ///             Return the last memory address writen or read
 ///    @return  _lastaddress
 **/
uint32_t FRAM_MB85RS::getLastMemAdr()
{
#ifdef DEBUG_TRACE
    Serial.print("Last address used in memory: 0x");
    Serial.println(_lastaddress, HEX);
#endif
    return _lastaddress;
}




/*========================================================================*/
/*                           PRIVATE FUNCTIONS                            */
/*========================================================================*/


/*!
///     @brief   _csCONFIG()
///              initialize the chip select line
**/
void FRAM_MB85RS::_csCONFIG()
{
    pinMode(_cs, OUTPUT);
}



/*!
///     @brief   _spi_begin()
///              initialize SPI transactionnal mode and set the chip select
///              line as active for data transmission/reception
**/
void FRAM_MB85RS::_spi_begin()
{
    _spi->beginTransaction(spiSettings);
    digitalWriteFast(_cs, LOW);
}



/*!
///     @brief   _spi_end(), ends SPI transactionnal mode
///              and set the chip select line inactive
**/
void FRAM_MB85RS::_spi_end()
{
    digitalWriteFast(_cs, HIGH);
    _spi->endTransaction();
}



/*!
///     @brief   _getDeviceID()
///              Reads the Manufacturer ID and the Product ID and populate
///              class' variables for devices supporting that feature.
///              _manufacturerID: The 8-bit manufacturer ID (Fujitsu = 0x04)
///              _productID: seems useless
///              _densitycode: Memory density (bytes 5..0)
///                            from 0x03 (64K chip) to 0x08 (2M chip)
///              _density: Human readable memory density, from 64 to 1024K
///              _maxaddress: The memory max address of storage slot
///     @return  0: error
///              1: ok
**/
bool FRAM_MB85RS::_getDeviceID()
{
    uint8_t buffer[3] = { 0, 0, 0 };

    _spi_begin();

    _spi->transfer(FRAM_RDID);
    _manufacturer = _spi->transfer(0);
    buffer[0] = _spi->transfer(0);
    buffer[1] = _spi->transfer(0);
    buffer[2] = _spi->transfer(0);

    _spi_end();

    /* Shift values to separate IDs */
    _densitycode = buffer[1] &= (1<<5)-1; // Only the 5 first bits
    _productID = (buffer[2] << 8) + buffer[3]; // Is really necessary to read this info ?

    if (_manufacturer == FUJITSU_ID) {
        switch (_densitycode) {
            case DENSITY_MB85RS64V:
            case DENSITY_MB85RS128B:
            case DENSITY_MB85RS256B:
            case DENSITY_MB85RS512T:
            case DENSITY_MB85RS1MT:
            case DENSITY_MB85RS2MT:
                _density = pow (2, _densitycode+3);
                _maxaddress = _density*128;
                break;

            default:
                // F-RAM chip unidentified
                _density = 0;
                _maxaddress = 0;
                return false;
                break;
        }
    } else {
        // F-RAM chip unidentified
        _density = 0;
        _maxaddress = 0;
        return false;
    }

    return true;
}



/*!
///     @brief   _deviceID2Serial()
///              Print out F-RAM characteristics
///
///     @return  0: error, no DEBUG_TRACE available
///              1: ok, print out all the datas
**/
bool FRAM_MB85RS::_deviceID2Serial()
{
    if (!Serial)
        return false; // Serial not available

#ifdef CHIP_TRACE
    Serial.println("\n** F-RAM Device IDs");
    Serial.print("Manufacturer 0x"); Serial.println(_manufacturer, HEX);
    Serial.print("ProductID 0x"); Serial.println(_productID, HEX);
    Serial.print("Density code 0x"); Serial.print(_densitycode, HEX);
    Serial.print(", Chip density "); Serial.print(_density, DEC); Serial.println("KBits");
    Serial.print("Max address : 0 to "); Serial.print(_maxaddress-1, DEC); Serial.print(" / "); Serial.println(_maxaddress-1, HEX);
    Serial.println("Device identfied automatically");
#else
    return false;
#endif
    return true;
}



/*!
///     @brief   _sendAddr()
///              Sends memory address to SPI bus.
///              Chips of 1Mbit or above use 24-bit addresses,
///              small ones use 16-bit addresses.
///     @param   framAddr, the address to send
**/
void FRAM_MB85RS::_sendAddr( uint32_t framAddr )
{
    if (_densitycode >= DENSITY_MB85RS1MT)
        _spi->transfer((framAddr >> 16) & 0xFF);  // Bits 16 to 23, MSB
    _spi->transfer((framAddr >> 8) & 0xFF);       // Bits 8 to 15
    _spi->transfer((framAddr     ) & 0xFF);       // Bits 0 to 7,   LSB
}
