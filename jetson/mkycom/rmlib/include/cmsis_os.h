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
 * @file cmsis_os.h
 * @author Haowen Shi
 * @date 16 Jan 2019
 * @brief Bridge header for providing OS features support.
 *
 * This header is to unify interfaces between RoboMky-Firmware and jetson
 * host PC code.
 *
 * Do not include this header unless you know what you are doing. This is a
 * hack.
 */

#ifndef __CMSISOS_BRIDGE_HDR__
#define __CMSISOS_BRIDGE_HDR__

#include <pthread.h>

#define osWaitForever (1)

/**
 * @brief Interface to local locks.
 */
#define osMutexWait(mutex, timeout) pthread_mutex_lock(mutex)

#define osMutexRelease(mutex) pthread_mutex_unlock(mutex)

#define osMutexDelete(mutex) pthread_mutex_destroy(mutex)

typedef pthread_mutex_t * osMutexId;

#endif /* __CMSISOS_BRIDGE_HDR__ */
