/*
  This file is part of the STM32duinoBLE library.
  Copyright (c) 2019 STMicroelectronics. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "HCISpiTransport.h"

volatile int data_avail = 0;

HCISpiTransportClass::HCISpiTransportClass(SPIClass &spi, BLEChip_t ble_chip, uint8_t cs_pin, uint8_t spi_irq, uint8_t ble_rst, uint32_t frequency, uint8_t spi_mode) :
  _spi(&spi),
  _ble_chip(ble_chip),
  _cs_pin(cs_pin),
  _spi_irq(spi_irq),
  _ble_rst(ble_rst)
{
  _spiSettings = SPISettings(frequency, (BitOrder)BLE_SPI_BYTE_ORDER, spi_mode);
  _read_index = 0;
  _write_index = 0;
}

HCISpiTransportClass::~HCISpiTransportClass()
{
}

extern "C" void SPI_Irq_Callback(void)
{
  data_avail = 1;
}

int HCISpiTransportClass::begin()
{
  _read_index = 0;
  _write_index = 0;
  memset(_rxbuff, 0, sizeof(_rxbuff));
  pinMode(_cs_pin, OUTPUT);
  digitalWrite(_cs_pin, HIGH);

  _spi->begin();

  pinMode(_spi_irq, INPUT);
  attachInterrupt(_spi_irq, SPI_Irq_Callback, RISING);

  // Reset chip
  pinMode(_ble_rst, OUTPUT);
  digitalWrite(_ble_rst, LOW);
  delay(5);
  digitalWrite(_ble_rst, HIGH);
  delay(5);

  return 1;
}

void HCISpiTransportClass::end()
{
  detachInterrupt(_spi_irq);
  _spi->end();
}

void HCISpiTransportClass::wait(unsigned long timeout)
{
  for (unsigned long start = millis(); (millis() - start) < timeout;) {
    if (available()) {
      break;
    }
  }
}

int HCISpiTransportClass::available()
{
  if (_ble_chip != SPBTLE_RF && _ble_chip != SPBTLE_1S && _ble_chip != BLUENRG_M2SP && _ble_chip != BLUENRG_M0 && _ble_chip != BLUENRG_LP) {
    return 0;
  }

  if (_read_index != _write_index) {
    return 1;
  } else if (data_avail) {
    if (digitalRead(_spi_irq) == 0) {
      return 0;
    }

    data_avail = 0;

    while (digitalRead(_spi_irq) == 1 && _write_index != BLE_MODULE_SPI_BUFFER_SIZE) {
      uint8_t header_master[5] = {0x0b, 0x00, 0x00, 0x00, 0x00};

      if (_ble_chip == SPBTLE_1S || _ble_chip == BLUENRG_M2SP || _ble_chip == BLUENRG_LP) {
        detachInterrupt(_spi_irq);
      }

      _spi->beginTransaction(_spiSettings);

      digitalWrite(_cs_pin, LOW);

      /* Write the header */
      _spi->transfer(header_master, 5);

      if (_ble_chip == SPBTLE_RF || _ble_chip == BLUENRG_M0) {
        /* device is ready */
        if (header_master[0] == 0x02) {
          uint16_t byte_count = (header_master[4] << 8) | header_master[3];

          if (byte_count > 0) {
            /* avoid to read more data that available size of the buffer */
            if (byte_count > (BLE_MODULE_SPI_BUFFER_SIZE - _write_index)) {
              byte_count = (BLE_MODULE_SPI_BUFFER_SIZE - _write_index);
              /* SPI buffer is full but we still have data to store, so we set the data_avail flag to true */
              data_avail = 1;
            }

            /* Read the response */
            for (int j = 0; j < byte_count; j++) {
              _rxbuff[_write_index] = _spi->transfer(0xFF);
              _write_index++;
            }
          }
        }
      } else if (_ble_chip == SPBTLE_1S || _ble_chip == BLUENRG_M2SP || _ble_chip == BLUENRG_LP) {
        uint16_t byte_count = (header_master[4] << 8) | header_master[3];

        if (byte_count > 0) {
          /* avoid to read more data that available size of the buffer */
          if (byte_count > (BLE_MODULE_SPI_BUFFER_SIZE - _write_index)) {
            byte_count = (BLE_MODULE_SPI_BUFFER_SIZE - _write_index);
            /* SPI buffer is full but we still have data to store, so we set the data_avail flag to true */
            data_avail = 1;
          }

          /* Read the response */
          for (int j = 0; j < byte_count; j++) {
            _rxbuff[_write_index] = _spi->transfer(0xFF);
            _write_index++;
          }
        }
      }

      digitalWrite(_cs_pin, HIGH);

      _spi->endTransaction();

      if (_ble_chip == SPBTLE_1S || _ble_chip == BLUENRG_M2SP || _ble_chip == BLUENRG_LP) {
        attachInterrupt(_spi_irq, SPI_Irq_Callback, RISING);
      }
    }

    if (_read_index != _write_index) {
      return 1;
    } else {
      return 0;
    }
  } else {
    return 0;
  }
}

int HCISpiTransportClass::peek()
{
  int peek_val = -1;

  if (_read_index != _write_index) {
    peek_val = _rxbuff[_read_index];
  }

  return peek_val;
}

int HCISpiTransportClass::read()
{
  int read_val = -1;

  if (_read_index != _write_index) {
    read_val = _rxbuff[_read_index];
    _read_index++;
    if (_read_index == _write_index) {
      /* Reset buffer index */
      _read_index = 0;
      _write_index = 0;
    }
  }

  return read_val;
}

size_t HCISpiTransportClass::write(const uint8_t *data, size_t length)
{
  uint8_t header_master[5] = {0x0a, 0x00, 0x00, 0x00, 0x00};
  void *my_data = (void *)data;
  int result = 0;
  uint32_t tickstart = millis();

  if (_ble_chip != SPBTLE_RF && _ble_chip != SPBTLE_1S && _ble_chip != BLUENRG_M2SP && _ble_chip != BLUENRG_M0 && _ble_chip != BLUENRG_LP) {
    return 0;
  }

  do {
    if (_ble_chip == SPBTLE_RF || _ble_chip == BLUENRG_M0) {
      result = 0;

      _spi->beginTransaction(_spiSettings);

      digitalWrite(_cs_pin, LOW);

      /* Write the header */
      _spi->transfer(header_master, 5);

      /* device is ready */
      if (header_master[0] == 0x02) {
        if (header_master[1] >= length) {
          /* Write the data */
          _spi->transfer(my_data, length);
        } else {
          result = -2;
        }
      } else {
        result = -1;
      }

      digitalWrite(_cs_pin, HIGH);

      _spi->endTransaction();

      if ((millis() - tickstart) > 1000) {
        result = -3;
        break;
      }
    } else if (_ble_chip == SPBTLE_1S || _ble_chip == BLUENRG_M2SP || _ble_chip == BLUENRG_LP) {
      uint32_t tickstart_data_available = millis();
      result = 0;

      detachInterrupt(_spi_irq);

      _spi->beginTransaction(_spiSettings);

      digitalWrite(_cs_pin, LOW);

      /*
       * Wait until BlueNRG-1 is ready.
       * When ready it will raise the IRQ pin.
       */
      while (!(digitalRead(_spi_irq) == 1)) {
        if ((millis() - tickstart_data_available) > 1000) {
          result = -3;
          break;
        }
      }

      if (result == -3) {
        digitalWrite(_cs_pin, HIGH);
        _spi->endTransaction();
        attachInterrupt(_spi_irq, SPI_Irq_Callback, RISING);
        break;
      }

      /* Write the header */
      _spi->transfer(header_master, 5);

      if ((int)((((uint16_t)header_master[2]) << 8) | ((uint16_t)header_master[1])) >= (int)length) {
        /* Write the data */
        _spi->transfer(my_data, length);
      } else {
        result = -2;
      }

      digitalWrite(_cs_pin, HIGH);

      _spi->endTransaction();

      attachInterrupt(_spi_irq, SPI_Irq_Callback, RISING);

      if ((millis() - tickstart) > 1000) {
        result = -3;
        break;
      }
    }
  } while (result < 0);

  if (result < 0) {
    return 0;
  } else {
    return length;
  }
}
