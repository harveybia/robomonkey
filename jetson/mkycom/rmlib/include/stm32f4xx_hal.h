/****************************************************************************
 * Copyright (C) 2019 by Haowen Shi                                         *
 *                                                                          *
 *   This program is free software: you can redistribute it and/or modify   *
 *   it under the terms of the GNU General Public License as published by   *
 *   the Free Software Foundation, either version 3 of the License, or      *
 *   (at your option) any later version.                                    *
 *                                                                          *
 *   This program is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU General Public License for more details.                           *
 *                                                                          *
 *   You should have received a copy of the GNU General Public License      *
 *   along with this program. If not, see <https://www.gnu.org/licenses/>.  *
 ****************************************************************************/

/**
 * @file stm32f4xx_hal.h
 * @author Haowen Shi
 * @date 16 Jan 2019
 * @brief Bridge header for unifying interface between ARM and X86 toolchain.
 *
 * This header is to unify interfaces between RoboMky-Firmware and jetson
 * host PC code.
 *
 * Do not include this header unless you know what you are doing. This is a
 * hack.
 */

#ifndef __STM32F4_BRIDGE_HDR__
#define __STM32F4_BRIDGE_HDR__

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Input byte stream gets written into struct directly.
 *
 * @see https://gcc.gnu.org/onlinedocs/gcc-4.0.4/gcc/Type-Attributes.html
 */
// #define __packed struct __attribute__((__packed__))

#endif /* __STM32F4_BRIDGE_HDR__ */
