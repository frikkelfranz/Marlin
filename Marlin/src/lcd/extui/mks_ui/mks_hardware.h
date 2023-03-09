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

<<<<<<< HEAD
<<<<<<<< HEAD:Marlin/src/lcd/extui/mks_ui/mks_hardware.h
=======
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
#include "../../../inc/MarlinConfigPre.h"

#include <lvgl.h>

// Functions for MKS_TEST
#if BOTH(MKS_TEST, SDSUPPORT)
  void mks_hardware_test();
  void mks_test_get();
  void mks_gpio_test();
  extern uint8_t mks_test_flag;
#else
  #define mks_test_flag 0
#endif

// String display and assets
<<<<<<< HEAD
void disp_string(uint16_t x, uint16_t y, const char * string, uint16_t charColor, uint16_t bkColor);
void disp_assets_update();
void disp_assets_update_progress(const char *msg);
========
/**
 * Greek (Cyprus)
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 */

#include "language_el.h"

namespace Language_el_CY {
  using namespace Language_el; // Inherit undefined strings from Greek (or English)

  constexpr uint8_t CHARSIZE              = 2;
  LSTR LANGUAGE                           = _UxGT("Greek (Cyprus)");
}
>>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5:Marlin/src/lcd/language/language_el_CY.h
=======
void disp_string(uint16_t x, uint16_t y, const char * cstr, uint16_t charColor, uint16_t bkColor);
void disp_string(uint16_t x, uint16_t y, FSTR_P const fstr, uint16_t charColor, uint16_t bkColor);
void disp_assets_update();
void disp_assets_update_progress(FSTR_P const msg);
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
