/*
 * Copyright (C) 2015  Andreas Rohner
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ESPPinChangeInterruptHandler.h"

#ifndef NOT_AN_INTERRUPT
  #define NOT_AN_INTERRUPT -1
#endif

#define interruptToDigitalPin(p) (p)

int8_t digitalPinToPCINT(int8_t pin) {
  return digitalPinToInterrupt(pin);
}

static ESPPinChangeInterruptHandler *handlers[EXTERNAL_NUM_INTERRUPTS];
static bool state[EXTERNAL_NUM_INTERRUPTS];

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static IRAM_ATTR void changeInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
  for (byte i = 0; i < EXTERNAL_NUM_INTERRUPTS; ++i) {
    bool data = digitalRead(interruptToDigitalPin(i));
    if (data != state[i]) {
      state[i] = data;
      ESPPinChangeInterruptHandler *handler = handlers[i];
      if (handler)
        handler->handlePCInterrupt(i, data);
    }
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void ESPPinChangeInterruptHandler::attachPCInterrupt(int8_t pcIntNum) {
  state[pcIntNum] = digitalRead(pcIntNum);
  attachInterrupt(digitalPinToInterrupt(pcIntNum), changeInterrupt, CHANGE);
  handlers[pcIntNum] = this;
}

void ESPPinChangeInterruptHandler::detachPCInterrupt(int8_t pcIntNum) {
  detachInterrupt(digitalPinToInterrupt(pcIntNum));
  handlers[pcIntNum] = 0;
}

