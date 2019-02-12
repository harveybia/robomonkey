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
 * @file rmlib.hpp
 * @author Haowen Shi
 * @date 15 Jan 2019
 * @brief Header to solve linking problem where CPP cannot find C symbols.
 *
 * Include this header to use functionalities provided by rmlib.
 *
 * Note: rmlib is created by RoboMaster.
 */

extern "C"
{
  #include "communicate.h"
  #include "data_fifo.h"
  #include "protocol.h"
  #include "infantry_info.h"
}
