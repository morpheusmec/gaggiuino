/* 09:32 15/03/2023 - change triggering comment */
#include "scales.h"
#include "pindef.h"
#include "remote_scales.h"

#include <HX711.h>
namespace {
  class LoadCellSingleton {
  public:
    static HX711& getInstance() {
      static HX711 instance(TIM3);
      return instance;
    }
  private:
    LoadCellSingleton() = default;
    ~LoadCellSingleton() = default;
  };
}

bool hwScalesPresent = false;

#if defined SINGLE_HX711_BOARD
unsigned char scale_clk = OUTPUT;
#else
unsigned char scale_clk = OUTPUT_OPEN_DRAIN;
#endif

void scalesInit(float scalesF1, float scalesF2) {
  hwScalesPresent = false;
  // Forced predicitve scales in case someone with actual hardware scales wants to use them.
  if (FORCE_PREDICTIVE_SCALES) {
    return;
  }

#ifndef DISABLE_HW_SCALES
  auto& loadCells = LoadCellSingleton::getInstance();
  loadCells.begin(HX711_dout_1, HX711_sck_1, 128U, scale_clk);
  pinMode(HX711_dout_1, INPUT_PULLUP);
  loadCells.set_scale(scalesF1);
  loadCells.power_up();

  if (loadCells.wait_ready_timeout(1000, 10)) {
    loadCells.tare(4);
    hwScalesPresent = true;
    
  }
  else {
    loadCells.power_down();
  }
#endif

  if (!hwScalesPresent && remoteScalesIsPresent()) {
    remoteScalesTare();
  }
}

void scalesTare(void) {
  if (hwScalesPresent) {
    auto& loadCells = LoadCellSingleton::getInstance();
    if (loadCells.wait_ready_timeout(150, 10)) {
      loadCells.tare(4);
    }
  }
  else if (remoteScalesIsPresent()) {
    remoteScalesTare();
  }
}

Measurement scalesGetWeight(void) {
  Measurement currentWeight = Measurement{ .value = 0.f, .millis = 0 };
  if (hwScalesPresent) {
    auto& loadCells = LoadCellSingleton::getInstance();
    if (loadCells.wait_ready_timeout(150, 10)) {
      currentWeight = Measurement{ .value=loadCells.get_units(), .millis=millis() };
    }
  }
  else if (remoteScalesIsPresent()) {
    currentWeight = remoteScalesGetWeight();
  }
  return currentWeight;
}

bool scalesIsPresent(void) {
  // Forced predicitve scales in case someone with actual hardware scales wants to use them.
  if (FORCE_PREDICTIVE_SCALES) {
    return false;
  }
  return hwScalesPresent;// || remoteScalesIsPresent();
}

float scalesDripTrayWeight() {
  long value = 0L;
  if (hwScalesPresent) {
    value = LoadCellSingleton::getInstance().read_average(4);
  }
  return (value);
}
