/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#ifdef __cplusplus
<<<<<<< HEAD
  extern "C" { /* C-declarations for C++ */
=======
  extern "C" {
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
#endif

void lv_draw_cloud_bind();
void lv_clear_cloud_bind();
void disp_bind_state();
void refresh_bind_ui();
void display_qrcode(uint8_t *qrcode_data);
void cloud_unbind();

#ifdef __cplusplus
  }
#endif
