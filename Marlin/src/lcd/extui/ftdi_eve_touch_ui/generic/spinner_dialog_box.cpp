/**************************
 * spinner_dialog_box.cpp *
 **************************/

/****************************************************************************
 *   Written By Mark Pelletier  2017 - Aleph Objects, Inc.                  *
 *   Written By Marcio Teixeira 2018 - Aleph Objects, Inc.                  *
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
 *   To view a copy of the GNU General Public License, go to the following  *
 *   location: <https://www.gnu.org/licenses/>.                             *
 ****************************************************************************/

#include "../config.h"
#include "../screens.h"
#include "../screen_data.h"

#ifdef FTDI_SPINNER_DIALOG_BOX

#define GRID_COLS 2
#define GRID_ROWS 8

using namespace FTDI;
using namespace ExtUI;
using namespace Theme;

constexpr static SpinnerDialogBoxData &mydata = screen_data.SpinnerDialogBox;

void SpinnerDialogBox::onEntry() {
  UIScreen::onEntry();
  mydata.auto_hide = true;
}

void SpinnerDialogBox::onExit() {
  CommandProcessor cmd;
  cmd.stop().execute();
}

void SpinnerDialogBox::onRefresh() {
  using namespace FTDI;
  DLCache dlcache(SPINNER_CACHE);
  CommandProcessor cmd;
  cmd.cmd(CMD_DLSTART);
  if (dlcache.has_data())
    dlcache.append();
  else
    dlcache.store(SPINNER_DL_SIZE);
  cmd.spinner(BTN_POS(1,4), BTN_SIZE(2,3)).execute();
}

void SpinnerDialogBox::onRedraw(draw_mode_t) {
}

<<<<<<< HEAD
void SpinnerDialogBox::show(FSTR_P message) {
=======
void SpinnerDialogBox::show(FSTR_P fstr) {
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
  CommandProcessor cmd;
  if (AT_SCREEN(SpinnerDialogBox)) cmd.stop().execute();
  cmd.cmd(CMD_DLSTART)
     .cmd(CLEAR_COLOR_RGB(bg_color))
     .cmd(CLEAR(true,true,true))
     .cmd(COLOR_RGB(bg_text_enabled))
     .tag(0);
<<<<<<< HEAD
  draw_text_box(cmd, BTN_POS(1,1), BTN_SIZE(2,3), message, OPT_CENTER, font_large);
=======
  draw_text_box(cmd, BTN_POS(1,1), BTN_SIZE(2,3), fstr, OPT_CENTER, font_large);
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
  DLCache dlcache(SPINNER_CACHE);
  if (!dlcache.store(SPINNER_DL_SIZE)) {
    SERIAL_ECHO_MSG("CachedScreen::storeBackground() failed: not enough DL cache space");
    cmd.cmd(CMD_DLSTART).cmd(CLEAR(true,true,true));
    dlcache.store(SPINNER_DL_SIZE);
  }
  if (AT_SCREEN(SpinnerDialogBox))
    cmd.spinner(BTN_POS(1,4), BTN_SIZE(2,3)).execute();
  else
    GOTO_SCREEN(SpinnerDialogBox);
  mydata.auto_hide = false;
}

void SpinnerDialogBox::hide() {
  GOTO_PREVIOUS();
}

<<<<<<< HEAD
void SpinnerDialogBox::enqueueAndWait(FSTR_P message, FSTR_P commands) {
  show(message);
  ExtUI::injectCommands_P((const char*)commands);
=======
void SpinnerDialogBox::enqueueAndWait(FSTR_P fstr, FSTR_P commands) {
  show(fstr);
  ExtUI::injectCommands(commands);
  mydata.auto_hide = true;
}

void SpinnerDialogBox::enqueueAndWait(FSTR_P fstr, char *commands) {
  show(fstr);
  ExtUI::injectCommands(commands);
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
  mydata.auto_hide = true;
}

void SpinnerDialogBox::enqueueAndWait(FSTR_P message, char *commands) {
  show(message);
  ExtUI::injectCommands(commands);
  mydata.auto_hide = true;
}

void SpinnerDialogBox::onIdle() {
<<<<<<< HEAD
  if (mydata.auto_hide && !commandsInQueue() && TERN1(HOST_KEEPALIVE_FEATURE, GcodeSuite::busy_state == GcodeSuite::NOT_BUSY)) {
=======
  if (mydata.auto_hide && !commandsInQueue() && TERN1(HOST_KEEPALIVE_FEATURE, gcode.busy_state == gcode.NOT_BUSY)) {
>>>>>>> 8e03928dc3d482b30dad3e0ac908aff43541aab5
    mydata.auto_hide = false;
    hide();
  }
}

#endif // FTDI_SPINNER_DIALOG_BOX
