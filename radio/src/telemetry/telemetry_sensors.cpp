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

#include "edgetx.h"
#define _USE_MATH_DEFINES
#include <math.h>

#include "spektrum.h"

#if defined(CROSSFIRE)
  #include "crossfire.h"
#endif

#if defined(GHOST)
  #include "ghost.h"
#endif

#if defined(RADIO_NV14_FAMILY)
  #include "telemetry/flysky_nv14.h"
#endif

#if defined(MULTIMODULE)
  #include "hitec.h"
  #include "hott.h"
  #include "multi.h"
#endif

#if  defined(MULTIMODULE) || defined(PPM)
  #include "mlink.h"
#endif

#if defined(MULTIMODULE) || defined(AFHDS2) || defined(AFHDS3)
  #include "flysky_ibus.h"
#endif

#if defined(AFHDS3)
  #include "flysky_ibus2.h"
#endif

TelemetryItem telemetryItems[MAX_TELEMETRY_SENSORS];
bool allowNewSensors;

bool isFaiForbidden(source_t idx)
{
  if (idx < MIXSRC_FIRST_TELEM) return false;

  auto unit = g_model.telemetrySensors[(idx - MIXSRC_FIRST_TELEM) / 3].unit;
  if (unit == UNIT_VOLTS || unit == UNIT_DB) return false;

  return true;
}

// TODO in maths
uint32_t getDistFromEarthAxis(int32_t latitude)
{
  uint32_t lat = abs(latitude) / 10000;
  uint32_t angle2 = (lat * lat) / 10000;
  uint32_t angle4 = angle2 * angle2;

  return 139 * (((uint32_t)10000000 - ((angle2 * (uint32_t)123370) / 81) +
                 (angle4 / 25)) /
                12500);
}

void TelemetryItem::setValue(const TelemetrySensor & sensor, const char * val, uint32_t, uint32_t)
{
  strncpy(text, val, sizeof(text));
  // Save hash of string so changes can be detected quickly
  value = hash(text, sizeof(text));
  setFresh();
}

void TelemetryItem::setValue(const TelemetrySensor &sensor, int32_t val,
                             uint32_t unit, uint32_t prec)
{
  int32_t newVal = val;

  if (prec == 255) {
    prec = sensor.prec;
  }

  if (unit == UNIT_CELLS) {
    auto data = uint32_t(newVal);
    uint8_t cellsCount = (data >> 24u);
    uint8_t cellIndex = ((data >> 16u) & 0x0Fu);
    if (cellIndex >= MAX_CELLS)
      return;
    uint16_t cellValue = (data & 0xFFFFu);
    if (cellsCount == 0) {
      cellsCount = (cellIndex >= cells.count ? cellIndex + 1 : cells.count);
      if (cellsCount != cells.count) {
        clear();
        cells.count = cellsCount;
        // we skip this round as we are not sure we received all cells values
        return;
      }
    }
    else if (cellsCount != cells.count) {
      clear();
      cells.count = cellsCount;
    }
    cells.values[cellIndex].set(cellValue);
    if (cellIndex+1 == cells.count) {
      newVal = 0;
      for (int i=0; i<cellsCount; i++) {
        if (cells.values[i].state) {
          newVal += cells.values[i].value;
        }
        else {
          // we didn't receive all cells values
          return;
        }
      }
      newVal = sensor.getValue(newVal, UNIT_VOLTS, 2);
    }
    else {
      // we didn't receive all cells values
      return;
    }
  }
  else if (unit == UNIT_DATETIME) {
    auto data = uint32_t(newVal);
    if (data & 0x000000ff) {
      datetime.year = (uint16_t) ((data & 0xff000000) >> 24) + 2000;  // SPORT GPS year is only two digits
      datetime.month = (uint8_t) ((data & 0x00ff0000) >> 16);
      datetime.day = (uint8_t) ((data & 0x0000ff00) >> 8);
    }
    else {
      datetime.hour = (uint8_t) ((data & 0xff000000) >> 24);
      datetime.min = (uint8_t) ((data & 0x00ff0000) >> 16);
      datetime.sec = (uint8_t) ((data & 0x0000ff00) >> 8);
#if defined(RTCLOCK)
      if (g_eeGeneral.adjustRTC) {
        rtcAdjust(datetime.year, datetime.month, datetime.day, datetime.hour, datetime.min, datetime.sec);
      }
#endif
    }
    value = hash((void *) &datetime, sizeof(datetime));
    setFresh();
    return;
  }
  else if (unit == UNIT_GPS_LATITUDE) {
#if defined(INTERNAL_GPS)
    if (gpsData.fix  && gpsData.hdop < PILOTPOS_MIN_HDOP) {
      pilotLatitude = gpsData.latitude;
      gps.pilotDistFromEarthAxis = getDistFromEarthAxis(pilotLatitude);
    }
#endif
    if (!pilotLatitude) {
      pilotLatitude = newVal;
      gps.pilotDistFromEarthAxis = getDistFromEarthAxis(newVal);
    }
    gps.latitude = newVal;
    setFresh();
    return;
  }
  else if (unit == UNIT_GPS_LONGITUDE) {
#if defined(INTERNAL_GPS)
    if (gpsData.fix && gpsData.hdop < PILOTPOS_MIN_HDOP) {
      pilotLongitude = gpsData.longitude;
    }
#endif
    if (!pilotLongitude) {
      pilotLongitude = newVal;
    }
    gps.longitude = newVal;

    value = hash((void *) &gps, sizeof(gps));
    setFresh();
    return;
  }
  else if (unit == UNIT_DATETIME_YEAR) {
    datetime.year = newVal;
    return;
  }
  else if (unit == UNIT_DATETIME_DAY_MONTH) {
    auto data = uint32_t(newVal);
    datetime.month = data >> 8u;
    datetime.day = data & 0xFFu;
    return;
  }
  else if (unit == UNIT_DATETIME_HOUR_MIN) {
    auto data = uint32_t(newVal);
    datetime.hour = (data & 0xFFu);
    datetime.min = data >> 8u;
    return;
  }
  else if (unit == UNIT_DATETIME_SEC) {
    datetime.sec = newVal & 0xFFu;
    value = hash((void *) &datetime, sizeof(datetime));
    setFresh();
    return;
  }
  else if (sensor.unit == UNIT_RPMS) {
    if (sensor.custom.ratio != 0) {
      newVal = (newVal * sensor.custom.offset) / sensor.custom.ratio;
    }
  }
  else if (unit == UNIT_TEXT) {
    // Should be handled at telemetry protocol level
    return;
  }
  else {
    newVal = sensor.getValue(newVal, unit, prec);
    if (sensor.autoOffset) {
      if (!isAvailable()) {
        std.offsetAuto = -newVal;
      }
      newVal += std.offsetAuto;
    }
    if (sensor.filter) {
      if (!isAvailable()) {
        for (int i=0; i<TELEMETRY_AVERAGE_COUNT; i++) {
          std.filterValues[i] = newVal;
        }
      }
      else {
        // Calculate the average from values[] and value,
        // also shift readings in values [] array.
        // There is a possibility of value overflow in `sum` but
        // in reality no sensor value should be so big to cause it.
        int32_t sum = std.filterValues[0];
        for (int i=0; i<TELEMETRY_AVERAGE_COUNT-1; i++) {
          int32_t tmp = std.filterValues[i+1];
          std.filterValues[i] = tmp;
          sum += tmp;
        }
        std.filterValues[TELEMETRY_AVERAGE_COUNT-1] = newVal;
        sum += newVal;
        newVal = sum/(TELEMETRY_AVERAGE_COUNT+1);
      }
    }
  }

  if (!isAvailable()) {
    valueMin = newVal;
    valueMax = newVal;
  }
  else if (newVal < valueMin) {
    valueMin = newVal;
  }
  else if (newVal > valueMax) {
    valueMax = newVal;
    if (sensor.unit == UNIT_VOLTS) {
      valueMin = newVal; // the batt was changed
    }
  }

  for (int i=0; i<MAX_TELEMETRY_SENSORS; i++) {
    TelemetrySensor & it = g_model.telemetrySensors[i];
    if (it.type == TELEM_TYPE_CALCULATED && it.formula == TELEM_FORMULA_TOTALIZE && &g_model.telemetrySensors[it.consumption.source-1] == &sensor) {
      TelemetryItem & item = telemetryItems[i];
      int32_t increment = it.getValue(val, unit, prec);
      item.setValue(it, item.value+increment, it.unit, it.prec);
    }
  }

  value = newVal;
  setFresh();
}

void TelemetryItem::per10ms(const TelemetrySensor & sensor)
{
  switch (sensor.formula) {
    case TELEM_FORMULA_CONSUMPTION:
      if (sensor.consumption.source) {
        TelemetrySensor & currentSensor = g_model.telemetrySensors[sensor.consumption.source-1];
        TelemetryItem & currentItem = telemetryItems[sensor.consumption.source-1];
        if (!currentItem.isAvailable()) {
          return;
        }
        else if (currentItem.isOld()) {
          setOld();
          return;
        }
        int32_t current = convertTelemetryValue(currentItem.value, currentSensor.unit, currentSensor.prec, UNIT_AMPS, 1);
        currentItem.consumption.prescale += current;
        if (currentItem.consumption.prescale >= 3600) {
          currentItem.consumption.prescale -= 3600;
          setValue(sensor, value+1, sensor.unit, sensor.prec);
        }
        setFresh();
      }
      break;

    default:
      break;
  }
}

void TelemetryItem::eval(const TelemetrySensor & sensor)
{
  switch (sensor.formula) {
    case TELEM_FORMULA_CELL:
      if (sensor.cell.source) {
        TelemetryItem & cellsItem = telemetryItems[sensor.cell.source-1];
        if (cellsItem.isOld()) {
          setOld();
        }
        else {
          unsigned int index = sensor.cell.index;
          if (index == TELEM_CELL_INDEX_LOWEST || index == TELEM_CELL_INDEX_HIGHEST || index == TELEM_CELL_INDEX_DELTA) {
            unsigned int lowest=0, highest=0;
            for (int i=0; i<cellsItem.cells.count; i++) {
              if (cellsItem.cells.values[i].state) {
                if (!lowest || cellsItem.cells.values[i].value < cellsItem.cells.values[lowest-1].value)
                  lowest = i+1;
                if (!highest || cellsItem.cells.values[i].value > cellsItem.cells.values[highest-1].value)
                  highest = i+1;
              }
              else {
                lowest = highest = 0;
              }
            }
            if (lowest) {
              switch (index) {
                case TELEM_CELL_INDEX_LOWEST:
                  setValue(sensor, cellsItem.cells.values[lowest-1].value, UNIT_VOLTS, 2);
                  break;
                case TELEM_CELL_INDEX_HIGHEST:
                  setValue(sensor, cellsItem.cells.values[highest-1].value, UNIT_VOLTS, 2);
                  break;
                case TELEM_CELL_INDEX_DELTA:
                  setValue(sensor, cellsItem.cells.values[highest-1].value - cellsItem.cells.values[lowest-1].value, UNIT_VOLTS, 2);
                  break;
              }
            }
          }
          else {
            index -= 1;
            if (index < cellsItem.cells.count && cellsItem.cells.values[index].state) {
              setValue(sensor, cellsItem.cells.values[index].value, UNIT_VOLTS, 2);
            }
          }
        }
      }
      break;

    case TELEM_FORMULA_DIST:
      if (sensor.dist.gps) {
        TelemetryItem gpsItem = telemetryItems[sensor.dist.gps-1];
        TelemetryItem * altItem = nullptr;
        if (!gpsItem.isAvailable()) {
          return;
        }
        else if (gpsItem.isOld()) {
          setOld();
          return;
        }
        if (sensor.dist.alt) {
          altItem = &telemetryItems[sensor.dist.alt-1];
          if (!altItem->isAvailable()) {
            return;
          }
          else if (altItem->isOld()) {
            setOld();
            return;
          }
        }
        uint32_t angle = abs(gpsItem.gps.latitude - gpsItem.pilotLatitude);
        uint32_t dist = uint64_t(EARTH_RADIUS * M_PI / 180) * angle / 1000000;
        uint32_t result = dist * dist;

        angle = abs(gpsItem.gps.longitude - gpsItem.pilotLongitude);
        dist = uint64_t(gpsItem.gps.pilotDistFromEarthAxis) * angle / 1000000;
        result += dist * dist;

        // Length on ground (ignoring curvature of the earth)
        result = isqrt32(result);

        if (altItem) {
          dist = convertTelemetryValue(abs(altItem->value), g_model.telemetrySensors[sensor.dist.alt-1].unit, g_model.telemetrySensors[sensor.dist.alt-1].prec, UNIT_METERS, 0);
          result = (dist * dist) + (result * result);
          result = isqrt32(result);
        }

        setValue(sensor, result, UNIT_METERS);
      }
      break;

    case TELEM_FORMULA_ADD:
    case TELEM_FORMULA_AVERAGE:
    case TELEM_FORMULA_MIN:
    case TELEM_FORMULA_MAX:
    case TELEM_FORMULA_MULTIPLY:
    {
      int32_t value=0, count=0, available=0, maxitems=4, mulprec=0;
      if (sensor.formula == TELEM_FORMULA_MULTIPLY) {
        maxitems = 2;
        value = 1;
      }
      for (int i=0; i<maxitems; i++) {
        int8_t source = sensor.calc.sources[i];
        if (source) {
          unsigned int index = abs(source)-1;
          TelemetrySensor & telemetrySensor = g_model.telemetrySensors[index];
          TelemetryItem & telemetryItem = telemetryItems[index];
          if (sensor.formula == TELEM_FORMULA_AVERAGE) {
            if (telemetryItem.isAvailable())
              available = 1;
            else
              continue;
            if (telemetryItem.isOld())
              continue;
          }
          else {
            if (!telemetryItem.isAvailable()) {
              return;
            }
            else if (telemetryItem.isOld()) {
              setOld();
              return;
            }
          }
          int32_t sensorValue = telemetryItem.value;
          if (source < 0)
            sensorValue = -sensorValue;
          count += 1;
          if (sensor.formula == TELEM_FORMULA_MULTIPLY) {
            mulprec += telemetrySensor.prec;
            value *= convertTelemetryValue(sensorValue, telemetrySensor.unit, 0, sensor.unit, 0);
          }
          else {
            sensorValue = convertTelemetryValue(sensorValue, telemetrySensor.unit, telemetrySensor.prec, sensor.unit, sensor.prec);
            if (sensor.formula == TELEM_FORMULA_MIN)
              value = (count==1 ? sensorValue : min<int32_t>(value, sensorValue));
            else if (sensor.formula == TELEM_FORMULA_MAX)
              value = (count==1 ? sensorValue : max<int32_t>(value, sensorValue));
            else
              value += sensorValue;
          }
        }
      }
      if (sensor.formula == TELEM_FORMULA_AVERAGE) {
        if (count == 0) {
          if (available)
            setOld();
          return;
        }
        else {
          value = (value + count/2) / count;
        }
      }
      else if (sensor.formula == TELEM_FORMULA_MULTIPLY) {
        if (count == 0)
          return;
        value = convertTelemetryValue(value, sensor.unit, mulprec, sensor.unit, sensor.prec);
      }
      setValue(sensor, value, sensor.unit, sensor.prec);
      break;
    }

    default:
      break;
  }
}

static bool isTelemetrySensorReferenced(uint8_t index);

bool delTelemetryIndex(uint8_t index)
{
  // Do not delete a sensor while the model still refers to its slot.
  // Keeping the sensor definition allows rediscovery to update that same
  // slot instead of assigning a different sensor to an existing reference.
  if (isTelemetrySensorReferenced(index))
    return false;

  memclear(&g_model.telemetrySensors[index], sizeof(TelemetrySensor));
  telemetryItems[index].clear();
  storageDirty(EE_MODEL);
  return true;
}

static bool telemetrySourceReferencesSensor(int32_t source, uint8_t index)
{
  const int32_t value = abs(source);
  const int32_t first = MIXSRC_FIRST_TELEM + 3 * index;
  return value >= first && value <= first + 2;
}

static bool telemetrySwitchReferencesSensor(int32_t swtch, uint8_t index)
{
  return abs(swtch) == SWSRC_FIRST_SENSOR + index;
}

static bool telemetrySensorIndexReferencesSensor(int32_t source, uint8_t index)
{
  return source != 0 && abs(source) == index + 1;
}

// A deleted sensor slot must not be reused while the model still contains
// references to it. Otherwise automatic discovery can silently change the
// meaning of logical switches and other model settings without editing them.
#if defined(COLORLCD)
static bool widgetDataReferencesSensor(WidgetPersistentData & widgetData,
                                        uint8_t index)
{
  for (unsigned int option = 0; option < widgetData.options.size(); option++) {
    if (widgetData.getType(option) == WOV_Source &&
        telemetrySourceReferencesSensor(widgetData.getUnsignedValue(option), index))
      return true;
  }
  return false;
}
#endif

static bool isTelemetrySensorReferenced(uint8_t index)
{
#if defined(COLORLCD)
  for (int screen = 0; screen < MAX_CUSTOM_SCREENS; screen++) {
    if (!g_model.hasScreenData(screen))
      continue;

    CustomScreenData * screenData = g_model.getScreenData(screen);
    for (int zone = 0; zone < MAX_LAYOUT_ZONES; zone++) {
      WidgetPersistentData * widgetData = screenData->layoutData.getWidgetData(zone);
      if (widgetData && widgetDataReferencesSensor(*widgetData, index))
        return true;
    }
  }

  TopBarPersistentData * topbarData = g_model.getTopbarData();
  if (topbarData) {
    for (int zone = 0; zone < MAX_TOPBAR_ZONES; zone++) {
      WidgetPersistentData * widgetData = topbarData->getWidgetData(zone);
      if (widgetData && widgetDataReferencesSensor(*widgetData, index))
        return true;
    }
  }
#endif

#if MAX_SCRIPTS > 0
  // Model Lua script inputs can also hold telemetry source references.
  // The source/value input is a union, so inspect only inputs declared as
  // sources by the loaded script metadata.
  for (int i = 0; i < MAX_SCRIPTS; i++) {
    const ScriptInputsOutputs & script = scriptInputsOutputs[i];
    for (int j = 0; j < script.inputsCount && j < MAX_SCRIPT_INPUTS; j++) {
      if (script.inputs[j].type == INPUT_TYPE_SOURCE &&
          telemetrySourceReferencesSensor(g_model.scriptsData[i].inputs[j].source,
                                          index))
        return true;
    }
  }
#endif

  for (int i = 0; i < MAX_MIXERS; i++) {
    if (telemetrySourceReferencesSensor(g_model.mixData[i].srcRaw, index) ||
        telemetrySwitchReferencesSensor(g_model.mixData[i].swtch, index))
      return true;
  }

  for (int i = 0; i < MAX_EXPOS; i++) {
    if (telemetrySourceReferencesSensor(g_model.expoData[i].srcRaw, index) ||
        telemetrySwitchReferencesSensor(g_model.expoData[i].swtch, index))
      return true;
  }

  for (int i = 0; i < MAX_LOGICAL_SWITCHES; i++) {
    const LogicalSwitchData & ls = g_model.logicalSw[i];
    const uint8_t family = lswFamily(ls.func);

    if (telemetrySwitchReferencesSensor(ls.andsw, index))
      return true;

    if ((family == LS_FAMILY_OFS || family == LS_FAMILY_DIFF ||
         family == LS_FAMILY_COMP) &&
        telemetrySourceReferencesSensor(ls.v1, index))
      return true;

    if (family == LS_FAMILY_COMP &&
        telemetrySourceReferencesSensor(ls.v2, index))
      return true;

    if (family == LS_FAMILY_BOOL || family == LS_FAMILY_STICKY) {
      if (telemetrySwitchReferencesSensor(ls.v1, index) ||
          telemetrySwitchReferencesSensor(ls.v2, index))
        return true;
    }
    else if (family == LS_FAMILY_EDGE &&
             telemetrySwitchReferencesSensor(ls.v1, index)) {
      return true;
    }
  }

  for (int i = 0; i < MAX_TIMERS; i++) {
    if (telemetrySwitchReferencesSensor(g_model.timers[i].swtch, index))
      return true;
  }

  for (int i = 1; i < MAX_FLIGHT_MODES; i++) {
    if (telemetrySwitchReferencesSensor(g_model.flightModeData[i].swtch, index))
      return true;
  }

  for (int i = 0; i < MAX_SPECIAL_FUNCTIONS; i++) {
    const CustomFunctionData & cfn = g_model.customFn[i];
    if (telemetrySwitchReferencesSensor(cfn.swtch, index))
      return true;

    if ((CFN_FUNC(&cfn) == FUNC_VOLUME ||
         CFN_FUNC(&cfn) == FUNC_BACKLIGHT ||
         CFN_FUNC(&cfn) == FUNC_PLAY_VALUE) &&
        telemetrySourceReferencesSensor(CFN_PARAM(&cfn), index))
      return true;

    if (CFN_FUNC(&cfn) == FUNC_RESET &&
        CFN_PARAM(&cfn) >= FUNC_RESET_PARAM_FIRST_TELEM &&
        CFN_PARAM(&cfn) < FUNC_RESET_PARAM_FIRST_TELEM + MAX_TELEMETRY_SENSORS &&
        CFN_PARAM(&cfn) - FUNC_RESET_PARAM_FIRST_TELEM == index)
      return true;

#if defined(GVARS)
    if (CFN_FUNC(&cfn) == FUNC_ADJUST_GVAR &&
        (CFN_GVAR_MODE(&cfn) == FUNC_ADJUST_GVAR_SOURCE ||
         CFN_GVAR_MODE(&cfn) == FUNC_ADJUST_GVAR_SOURCERAW) &&
        telemetrySourceReferencesSensor(CFN_PARAM(&cfn), index))
      return true;
#endif
  }

  if (telemetrySourceReferencesSensor(g_model.swashR.collectiveSource, index) ||
      telemetrySourceReferencesSensor(g_model.swashR.aileronSource, index) ||
      telemetrySourceReferencesSensor(g_model.swashR.elevatorSource, index))
    return true;

  if (telemetrySensorIndexReferencesSensor(g_model.varioData.source, index) ||
      telemetrySensorIndexReferencesSensor(g_model.rssiSource, index))
    return true;

#if defined(PCBX9D) || defined(PCBX9DP) || defined(PCBX9E)
  if (telemetrySensorIndexReferencesSensor(g_model.voltsSource, index) ||
      telemetrySensorIndexReferencesSensor(g_model.altitudeSource, index))
    return true;
#endif

  // Calculated sensors can depend on other telemetry sensor slots.
  for (int i = 0; i < MAX_TELEMETRY_SENSORS; i++) {
    if (i == index)
      continue;

    const TelemetrySensor & sensor = g_model.telemetrySensors[i];
    if (!sensor.isAvailable() || sensor.type != TELEM_TYPE_CALCULATED)
      continue;

    switch (sensor.formula) {
      case TELEM_FORMULA_CELL:
        if (telemetrySensorIndexReferencesSensor(sensor.cell.source, index))
          return true;
        break;

      case TELEM_FORMULA_TOTALIZE:
      case TELEM_FORMULA_CONSUMPTION:
        if (telemetrySensorIndexReferencesSensor(sensor.consumption.source, index))
          return true;
        break;

      case TELEM_FORMULA_DIST:
        if (telemetrySensorIndexReferencesSensor(sensor.dist.gps, index) ||
            telemetrySensorIndexReferencesSensor(sensor.dist.alt, index))
          return true;
        break;

      case TELEM_FORMULA_ADD:
      case TELEM_FORMULA_AVERAGE:
      case TELEM_FORMULA_MIN:
      case TELEM_FORMULA_MAX:
      case TELEM_FORMULA_MULTIPLY:
        for (int j = 0; j < 4; j++) {
          if (telemetrySensorIndexReferencesSensor(sensor.calc.sources[j], index))
            return true;
        }
        break;

      default:
        break;
    }
  }

#if !defined(COLORLCD)
  for (int i = 0; i < MAX_TELEMETRY_SCREENS; i++) {
    for (int j = 0; j < 4; j++) {
      if (telemetrySourceReferencesSensor(g_model.screens[i].bars[j].source, index))
        return true;
      for (int k = 0; k < NUM_LINE_ITEMS; k++) {
        if (telemetrySourceReferencesSensor(g_model.screens[i].lines[j].sources[k], index))
          return true;
      }
    }
  }
#endif

  return false;
}

int availableTelemetryIndex()
{
  for (int index = 0; index < MAX_TELEMETRY_SENSORS; index++) {
    TelemetrySensor & telemetrySensor = g_model.telemetrySensors[index];
    if (!telemetrySensor.isAvailable() &&
        !isTelemetrySensorReferenced(index)) {
      return index;
    }
  }
  return -1;
}

int lastUsedTelemetryIndex()
{
  for (int index=MAX_TELEMETRY_SENSORS-1; index>=0; index--) {
    TelemetrySensor & telemetrySensor = g_model.telemetrySensors[index];
    if (telemetrySensor.isAvailable()) {
      return index;
    }
  }
  return -1;
}

template <class T>
int setTelemetryValue(TelemetryProtocol protocol, uint16_t id, uint8_t subId,
                      uint8_t instance, T value, uint32_t unit = 0,
                      uint32_t prec = 0)
{
  bool sensorFound = false;

  for (int index = 0; index < MAX_TELEMETRY_SENSORS; index++) {
    TelemetrySensor &telemetrySensor = g_model.telemetrySensors[index];

    if (telemetrySensor.type == TELEM_TYPE_CUSTOM && telemetrySensor.id == id &&
        telemetrySensor.subId == subId &&
        (telemetrySensor.isSameInstance(protocol, instance) ||
         g_model.ignoreSensorIds)) {

      telemetryItems[index].setValue(telemetrySensor, value, unit, prec);
      sensorFound = true;
      // we continue search here, because sensors can share the same id and
      // instance
    }
  }

  if (sensorFound || !allowNewSensors) {
    return -1;
  }

  int index = availableTelemetryIndex();
  if (index >= 0) {
    switch (protocol) {
      case PROTOCOL_TELEMETRY_FRSKY_SPORT:
        frskySportSetDefault(index, id, subId, instance);
        break;

      case PROTOCOL_TELEMETRY_FRSKY_D:
        frskyDSetDefault(index, id);
        break;

#if defined(CROSSFIRE)
      case PROTOCOL_TELEMETRY_CROSSFIRE:
        crossfireSetDefault(index, id, instance);
        break;
#endif

#if defined(GHOST)
      case PROTOCOL_TELEMETRY_GHOST:
        ghostSetDefault(index, id, instance);
        break;
#endif

#if defined(MULTIMODULE) || defined(AFHDS3) || defined(AFHDS2)
      case PROTOCOL_TELEMETRY_FLYSKY_IBUS:
        flySkySetDefault(index, id, subId, instance);
        break;
#endif

#if defined(AFHDS3)
      case PROTOCOL_TELEMETRY_FLYSKY_IBUS2:
        flySkyIbus2SetDefault(index, id, subId, instance);
        break;
#endif

#if defined(AFHDS2) && defined(RADIO_NV14_FAMILY)
      case PROTOCOL_TELEMETRY_FLYSKY_NV14:
        flySkyNv14SetDefault(index, id, subId, instance);
        break;
#endif

      case PROTOCOL_TELEMETRY_SPEKTRUM:
        spektrumSetDefault(index, id, subId, instance);
        break;

#if defined(MULTIMODULE)
      case PROTOCOL_TELEMETRY_HITEC:
        hitecSetDefault(index, id, subId, instance);
        break;

      case PROTOCOL_TELEMETRY_HOTT:
        hottSetDefault(index, id, subId, instance);
        break;
#endif

#if defined(MULTIMODULE) || defined(PPM)
      case PROTOCOL_TELEMETRY_MLINK:
        mlinkSetDefault(index, id, subId, instance);
        break;
#endif

#if defined(LUA)
     case PROTOCOL_TELEMETRY_LUA:
        // Sensor will be initialized by calling function
        // This drops the first value
        return index;
#endif

      default:
        return index;
    }
    telemetryItems[index].setValue(g_model.telemetrySensors[index], value, unit, prec);
    return index;
  }
  else {
    allowNewSensors = false;
#if !defined(COLORLCD)
    // Not safe on color LCD - handled in telemetry page instead
    POPUP_WARNING(STR_TELEMETRYFULL);
#endif
    return -1;
  }
}

int setTelemetryValue(TelemetryProtocol protocol, uint16_t id, uint8_t subId, uint8_t instance, int32_t value, uint32_t unit, uint32_t prec)
{
  return setTelemetryValue<int32_t>(protocol, id, subId, instance, value, unit, prec);
}

int setTelemetryText(TelemetryProtocol protocol, uint16_t id, uint8_t subId, uint8_t instance, const char * text)
{
  return setTelemetryValue<const char *>(protocol, id, subId, instance, text);
}

void TelemetrySensor::init(const char * label, uint8_t unit, uint8_t prec)
{
  memclear(this->label, TELEM_LABEL_LEN);
  strncpy(this->label, label, TELEM_LABEL_LEN);
  this->unit = unit;
  this->prec = prec;
  // Log sensors by default
  this->logs = true;
}

void TelemetrySensor::init(uint16_t id)
{
  char label[TELEM_LABEL_LEN]; // 4
  label[0] = hex2char((id & 0xf000) >> 12);
  label[1] = hex2char((id & 0x0f00) >> 8);
  label[2] = hex2char((id & 0x00f0) >> 4);
  label[3] = hex2char((id & 0x000f) >> 0);
  init(label);
}

bool TelemetrySensor::isAvailable() const
{
  return ZLEN(label) > 0;
}

PACK(typedef struct {
  uint8_t unitFrom;
  uint8_t unitTo;
  int16_t multiplier;
  int16_t divisor;
}) UnitConversionRule;

const UnitConversionRule unitConversionTable[] = {
  /* unitFrom     unitTo                    multiplier   divisor */
  { UNIT_METERS,            UNIT_FEET,             105,    32},
  { UNIT_FEET,              UNIT_METERS,            32,   105},
  { UNIT_METERS_PER_SECOND, UNIT_FEET_PER_SECOND,  105,    32},

  { UNIT_KTS, UNIT_KMH,                           1852,  1000}, // 1 knot = 1.85200 kilometers per hour
  { UNIT_KTS, UNIT_MPH,                           1151,  1000}, // 1 knot = 1.15077945 miles per hour
  { UNIT_KTS, UNIT_METERS_PER_SECOND,             1000,  1944}, // 1 knot = 0.514444444 meters / second (divide with 1.94384449)
  { UNIT_KTS, UNIT_FEET_PER_SECOND,               1688,  1000}, // 1 knot = 1.68780986 feet per second

  { UNIT_KMH, UNIT_KTS,                           1000,  1852}, // 1 km/h = 0.539956803 knots (divide with 1.85200)
  { UNIT_KMH, UNIT_MPH,                           1000,  1609}, // 1 km/h = 0.621371192 miles per hour (divide with 1.60934400)
  { UNIT_KMH, UNIT_METERS_PER_SECOND,               10,    36}, // 1 km/h = 0.277777778 meters / second (divide with 3.6)
  { UNIT_KMH, UNIT_FEET_PER_SECOND,                911,  1000}, // 1 km/h = 0.911344415 feet per second

  { UNIT_MILLILITERS, UNIT_FLOZ,                   100,  2957},

  { UNIT_RADIANS, UNIT_DEGREE,                   10000,   175}, // 1 rad = 57.29578 deg
  { UNIT_DEGREE, UNIT_RADIANS,                     175, 10000}, // 1 deg = ‪0,0174533‬ rad

  { 0, 0, 0, 0}   // termination
};

// The MAX precision used in sensors is 3..support up to Prec4. 
const int16_t power10[] = { 1, 10, 100, 1000, 10000 };

int32_t convertTelemetryValue(int32_t value, uint8_t unit, uint8_t prec, uint8_t destUnit, uint8_t destPrec)
{
  // if we need to do Temperature conversions, we need to convert the constant 32 from 
  // Prec0 to max(prec,destPrec).
  int8_t temp_prec = prec; 

  // Ex:  Converting from Prec1 -> Prec2.. need to * 10 the value.
  // Ex:  Converting from Prec0 -> Prec2.. need to * 100 the value. 
  if (prec < destPrec) {
      value    *= power10[destPrec-prec];
      temp_prec = destPrec;
  }
  
  if (unit == UNIT_CELSIUS) {
    if (destUnit == UNIT_FAHRENHEIT) {
      // T(°F) = T(°C)×1,8 + 32
      value = (32*power10[temp_prec]) + (value*18) / 10;
    }
  } else if (unit == UNIT_FAHRENHEIT) {
    if (destUnit == UNIT_CELSIUS) {
      value = (value - (32*power10[temp_prec])) * 10/18;
    }
  }
  else {
    const UnitConversionRule * p = unitConversionTable;
    while (p->divisor) {
      if (p->unitFrom == unit && p->unitTo == destUnit) {
        value = (value * (int32_t)p->multiplier) / (int32_t)p->divisor;
        break;
      }
      ++p;
    }
  }

  // Ex:  Converting from Prec2 -> Prec1.. need to / 10 the value.
  // Ex:  Converting from Prec2 -> Prec0.. need to / 100 the value. 
  if (destPrec < prec) {
      value = value / power10[prec - destPrec];
  }

  return value;
}

int32_t TelemetrySensor::getValue(int32_t value, uint8_t unit, uint8_t prec) const
{
  if (type == TELEM_TYPE_CUSTOM && custom.ratio) {
    /*  farzu:  Not needed, scaling work properly for the 3 types of prec without it  
    if (this->prec == 2) {
      value *= 10;
      prec = 2;
    }
    else {
      prec = 1;
    }
    */
    
    value = (custom.ratio * value + 122) / 255;  //  122/255 (0.48) is to aproximate up (ceiling) 
  }

  // Does it needs any conversion ? 
  if ((unit != this->unit) || (prec != this->prec)) {
    value = convertTelemetryValue(value, unit, prec, this->unit, this->prec);
  }

  if (type == TELEM_TYPE_CUSTOM) {
    value += custom.offset;
    if (value < 0 && onlyPositive) {
      value = 0;
    }
  }

  return value;
}

bool TelemetrySensor::isConfigurable() const
{
  if (type == TELEM_TYPE_CALCULATED) {
    if (formula >= TELEM_FORMULA_CELL) {
      return false;
    }
  }
  else {
    if (unit >= UNIT_FIRST_VIRTUAL)  {
      return false;
    }
  }
  return true;
}

bool TelemetrySensor::isPrecConfigurable() const
{
  if (unit == UNIT_FAHRENHEIT) { // use Prec0
    return false;
  } 
  else if (isConfigurable()) {
    return true;
  }
  else if (unit == UNIT_CELLS) {
    return true;
  }
  else {
    return false;
  }
}

int32_t TelemetrySensor::getPrecMultiplier() const
{
  /*
    Important: the return type must be signed, otherwise
    mathematic operations with a negative telemetry value won't work
  */
  if (prec == 2) return 1;
  if (prec == 1) return 10;
  return 100;
}

int32_t TelemetrySensor::getPrecDivisor() const
{
  if (prec == 2) return 100;
  if (prec == 1) return 10;
  return 1;
}

bool TelemetrySensor::isSameInstance(TelemetryProtocol protocol, uint8_t instance)
{
  if (this->instance == instance)
    return true;

  if (protocol == PROTOCOL_TELEMETRY_FRSKY_SPORT) {
#if defined(SIMU)
    if (((this->instance ^ instance) & 0x1F) == 0)
      return true;
#else
    if (((this->instance ^ instance) & 0x9F) == 0 && (this->instance >> 5) != TELEMETRY_ENDPOINT_SPORT && (instance >> 5) != TELEMETRY_ENDPOINT_SPORT) {
      this->instance = instance; // update the instance in case we had telemetry switching
      return true;
    }
#endif
  }

  return false;
}
