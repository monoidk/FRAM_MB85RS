/**************************************************************************/
/*
    @file     FRAM_MB85RS.h
    @author   Ivan Labáth
              Christophe Persoz for SPI version
              SOSAndroid.fr (E. Ha.) for I2C version

    @section  HISTORY

    v0.8 - See ReadMe for more informations

    Driver for the MB85RC SPI FRAM from Fujitsu.

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
#ifndef __FRAM_MB85RS_H__
#define __FRAM_MB85RS_H__

#include <SPI.h>


// DEFINES

#ifndef DEBUG_TRACE
    #define DEBUG_TRACE    // Enabling Debug Trace on Serial
#endif
#ifndef CHIP_TRACE
    #define CHIP_TRACE     // Serial trace for characteristics of the chip
#endif


// Managing Write protect pin
// false means protection off, write enabled
#define DEFAULT_WP_STATUS false


class FRAM_MB85RS
{
 private:
    // IDs - can be extends to any other compatible chip
    static constexpr uint8_t FUJITSU_ID = 0x04;

    // Density codes gives the memory's adressing scheme
    static constexpr uint8_t DENSITY_MB85RS64V  = 0x03; // 64K
    static constexpr uint8_t DENSITY_MB85RS128B = 0x04; // 128K
    static constexpr uint8_t DENSITY_MB85RS256B = 0x05; // 256K
    static constexpr uint8_t DENSITY_MB85RS512T = 0x06; // 512K
    static constexpr uint8_t DENSITY_MB85RS1MT  = 0x07; // 1M
    static constexpr uint8_t DENSITY_MB85RS2MT  = 0x08; // 2M

    // OP-CODES
    static constexpr uint8_t FRAM_WRSR  = 0x01; // 0000 0001 - Write Status Register
    static constexpr uint8_t FRAM_WRITE = 0x02; // 0000 0010 - Write Memory
    static constexpr uint8_t FRAM_READ  = 0x03; // 0000 0011 - Read Memory
    static constexpr uint8_t FRAM_WRDI  = 0x04; // 0000 0100 - Reset Write Enable Latch
    static constexpr uint8_t FRAM_RDSR  = 0x05; // 0000 0101 - Read Status Register
    static constexpr uint8_t FRAM_WREN  = 0x06; // 0000 0110 - Set Write Enable Latch
    static constexpr uint8_t FRAM_FSTRD = 0x0B; // 0000 1011 - Fast Read
    static constexpr uint8_t FRAM_RDID  = 0x9F; // 1001 1111 - Read Device ID
    static constexpr uint8_t FRAM_SLEEP = 0xB9; // 1011 1001 - Sleep mode

    SPISettings spiSettings = SPISettings((uint32_t)10'000'000, MSBFIRST, SPI_MODE0);
 public:
    FRAM_MB85RS(SPIClass & spi, uint8_t cs);
    FRAM_MB85RS(SPIClass & spi, uint8_t cs, uint8_t wp);


    void begin_tree();
    void begin();
    bool identify();

    bool read(uint32_t framAddr, uint8_t *value) { return readBuf(framAddr, value, sizeof(*value)); }
    bool read(uint32_t framAddr, uint16_t *value) { return readBuf(framAddr, value, sizeof(*value)); }
    bool read(uint32_t framAddr, uint32_t *value) { return readBuf(framAddr, value, sizeof(*value)); }
    bool write(uint32_t framAddr, uint8_t value) { return writeBuf(framAddr, &value, sizeof(value)); }
    bool write(uint32_t framAddr, uint16_t value) { return writeBuf(framAddr, &value, sizeof(value)); }
    bool write(uint32_t framAddr, uint32_t value) { return writeBuf(framAddr, &value, sizeof(value)); }

    bool readBuf(uint32_t addr, void * buf, uint32_t size);
    bool readArray(uint32_t startAddr, uint8_t values[], size_t nbItems ) { return readBuf(startAddr, values, nbItems); }
    bool readArray(uint32_t startAddr, uint16_t values[], size_t nbItems ) { return readBuf(startAddr, values, nbItems * 2); }
    bool writeBuf(uint32_t addr, const void * buf, uint32_t size);
    bool writeArray(uint32_t startAddr, uint8_t values[], size_t nbItems ) { return writeBuf(startAddr, values, nbItems); }
    bool writeArray(uint32_t startAddr, uint16_t values[], size_t nbItems ) {return writeBuf(startAddr, values, nbItems * 2); }

    bool isAvailable();
    bool getWPStatus();
    bool enableWP();
    bool disableWP();
    bool eraseChip();
    uint32_t getMaxMemAdr();
    uint32_t getLastMemAdr();


 private:
    SPIClass *  _spi;
    bool                _framInitialised;
    uint8_t     _cs;            // CS pin
    bool        _wp;            // WP management
    uint8_t     _wpPin;         // WP pin connected and Write Protection enabled
    bool        _wpStatus;      // WP Status
    uint8_t     _manufacturer;  // Manufacturer ID
    uint16_t    _productID;     // Product ID
    uint8_t     _densitycode;   // Code which represent the size of the chip
    uint16_t    _density;       // Human readable size of F-RAM chip
    uint32_t    _maxaddress;    // Maximum address suported by F-RAM chip
    uint32_t    _lastaddress;   // Last address used in memory

    void        _csCONFIG();
    void        _spi_begin();
    void        _spi_end();
    bool        _getDeviceID();
    bool        _deviceID2Serial();
    void        _sendAddr(uint32_t framAddr);
};



#endif
