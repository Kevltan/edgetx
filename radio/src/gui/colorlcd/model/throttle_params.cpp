/*
 * Copyright (C) EdgeTX
 *
 * Based on code named
 *   opentx - https://github.com/opentx/opentx
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "throttle_params.h"
#include "edgetx.h"
#include "sourcechoice.h"
#include "switchchoice.h"
#include "choice.h"

#define SET_DIRTY()     storageDirty(EE_MODEL)

static SetupLineDef setupLines[] = {
  {
    // Throttle reversed
    STR_DEF(STR_THROTTLEREVERSE),
    [](Window* parent, coord_t x, coord_t y) {
      new ToggleSwitch(parent, {x, y, 0, 0}, GET_SET_DEFAULT(g_model.throttleReversed));
    }
  },
  {
    // Throttle source
    STR_DEF(STR_TTRACE),
    [](Window* parent, coord_t x, coord_t y) {
      auto sc = new SourceChoice(parent, {x, y, 0, 0}, 0, MIXSRC_LAST_CH,
                                []() {return throttleSource2Source(g_model.thrTraceSrc); },
                                [](int16_t src) {
                                  int16_t val = source2ThrottleSource(src);
                                  if (val >= 0) {
                                    g_model.thrTraceSrc = val;
                                    SET_DIRTY();
                                  }
                                });
      sc->setAvailableHandler(isThrottleSourceAvailable);
    }
  },
  {
    // Throttle trim
    STR_DEF(STR_TTRIM),
    [](Window* parent, coord_t x, coord_t y) {
      new ToggleSwitch(parent, {x, y, 0, 0}, GET_SET_DEFAULT(g_model.thrTrim));
    }
  },
#if defined(RADIO_TX15) || defined(RADIO_TX16SMK3)
  {
    // Idle-only throttle trim increment
    STR_DEF(STR_TRIMINC),
    [](Window* parent, coord_t x, coord_t y) {
      static constexpr uint16_t customSteps[] = {1, 4, 8, 16, 32, 64, 128, 256};
      static const char* const customStepValues[] = {
          "1", "4", "8", "16", "32", "64", "128", "256"};

      auto normalizeCustomStep = []() {
        uint16_t currentStep = g_model.idleTrimCustomStep;
        if (currentStep == 0)
          currentStep = 4;

        int closestIndex = 1;
        uint16_t closestDistance = currentStep > customSteps[1]
                                       ? currentStep - customSteps[1]
                                       : customSteps[1] - currentStep;
        for (int i = 0; i < DIM(customSteps); i++) {
          uint16_t distance = currentStep > customSteps[i]
                                  ? currentStep - customSteps[i]
                                  : customSteps[i] - currentStep;
          if (distance < closestDistance) {
            closestDistance = distance;
            closestIndex = i;
          }
        }

        if (g_model.idleTrimCustomStep != customSteps[closestIndex]) {
          g_model.idleTrimCustomStep = customSteps[closestIndex];
          SET_DIRTY();
        }
      };

      if (g_model.idleTrimStepMode == IDLE_TRIM_STEP_CUSTOM)
        normalizeCustomStep();

      static const char* const values[] = {"Fine", "Medium", "Coarse", "Custom"};
      auto stepType = new Choice(
          parent, {x, y, 0, 0}, values, 0, 3,
          []() {
            // Keep a valid visible selection if Custom is stored but
            // Idle Only is disabled. The stored Custom setting is preserved.
            if (!g_model.thrTrim &&
                g_model.idleTrimStepMode == IDLE_TRIM_STEP_CUSTOM)
              return 1;

            switch (g_model.idleTrimStepMode) {
              case IDLE_TRIM_STEP_FINE: return 0;
              case IDLE_TRIM_STEP_MEDIUM: return 1;
              case IDLE_TRIM_STEP_COARSE: return 2;
              default: return 3;
            }
          });
      stepType->setAvailableHandler(
          [](int value) { return value != 3 || g_model.thrTrim; });

      auto customStep = new Choice(
          parent, {x + 150, y, 0, 0}, customStepValues, 0, 7,
          []() {
            for (int i = 0; i < DIM(customSteps); i++) {
              if (g_model.idleTrimCustomStep == customSteps[i])
                return i;
            }
            return 1;
          },
          [](int value) {
            value = limit<int>(0, value, DIM(customSteps) - 1);
            g_model.idleTrimCustomStep = customSteps[value];
            SET_DIRTY();
          });
      if (!g_model.thrTrim ||
          g_model.idleTrimStepMode != IDLE_TRIM_STEP_CUSTOM)
        customStep->hide();

      stepType->setSetValueHandler(
          [customStep, normalizeCustomStep](int value) {
            switch (value) {
              case 0: g_model.idleTrimStepMode = IDLE_TRIM_STEP_FINE; break;
              case 1: g_model.idleTrimStepMode = IDLE_TRIM_STEP_MEDIUM; break;
              case 2: g_model.idleTrimStepMode = IDLE_TRIM_STEP_COARSE; break;
              default:
                // The availability handler prevents this while Idle Only is off.
                if (!g_model.thrTrim)
                  return;
                g_model.idleTrimStepMode = IDLE_TRIM_STEP_CUSTOM;
                normalizeCustomStep();
                break;
            }
            if (g_model.thrTrim &&
                g_model.idleTrimStepMode == IDLE_TRIM_STEP_CUSTOM)
              customStep->show();
            else
              customStep->hide();
            SET_DIRTY();
          });
    }
  },
#endif
  {
    // Throttle trim source
    STR_DEF(STR_TTRIM_SW),
    [](Window* parent, coord_t x, coord_t y) {
      new SourceChoice(
          parent, {x, y, 0, 0}, MIXSRC_FIRST_TRIM, MIXSRC_LAST_TRIM,
          []() { return g_model.getThrottleStickTrimSource(); },
          [](int16_t src) {
            g_model.setThrottleStickTrimSource(src);
            SET_DIRTY();
          });
    }
  },
};

ThrottleParams::ThrottleParams() : SubPage(ICON_MODEL_SETUP, STR_MAIN_MENU_MODEL_SETTINGS, STR_THROTTLE_LABEL, setupLines, DIM(setupLines))
{
}
