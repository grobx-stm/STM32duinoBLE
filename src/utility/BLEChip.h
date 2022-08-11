/*
  This file is part of the STM32duinoBLE library.
  Copyright (c) 2022 STMicroelectronics Srl. All rights reserved.

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

#ifndef _BLE_CHIP_H_
#define _BLE_CHIP_H_

typedef enum BLEChip_s {
  SPBTLE_RF,
  SPBTLE_1S,
  BLUENRG_M2SP,
  BLUENRG_M0,
  BLUENRG_LP
} BLEChip_t;

#endif /* _BLE_CHIP_H_ */
