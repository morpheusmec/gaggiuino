/* 09:32 15/03/2023 - change triggering comment */
#include "pump.h"
#include "pindef.h"
#include <PSM.h>
#include "utils.h"
#include "internal_watchdog.h"

PSM pump(zcPin, dimmerPin, PUMP_RANGE, ZC_MODE, 1, 6);

float flowPerClickAtZeroBar = 0.27f;
int maxPumpClicksPerSecond = 50;
float fpc_multiplier = 1.2f;

int currentPumpValue = 0;

float Pn [] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
float Cn [] = {0, 6, 12, 18, 24, 30, 36, 42, 48, 54, 60};
const int Pcount = sizeof(Pn) / sizeof(Pn[0]);
const int Ccount = sizeof(Cn) / sizeof(Cn[0]);
float Qn[Pcount][Ccount] ={{0.0, 1.98, 3.957468077, 5.621748317, 7.113337769, 8.15506186, 8.28, 8.467274426, 9.009558884, 9.72, 10.35678733},
{0.0, 1.414302547, 2.828605093, 4.24290764, 5.657210187, 6.794637946, 7.2, 7.547331427, 8.003389206, 8.532, 9},
{0.0, 1.096489127, 2.192978253, 3.28946738, 4.385956507, 5.482445633, 5.931764231, 6.354830192, 6.572796735, 7.08973876, 7.450071324},
{0.0, 0.938077189, 1.876154377, 2.814231566, 3.752308754, 4.690385943, 4.968, 5.25, 5.424, 5.562, 5.76},
{0.0, 0.826197524, 1.652395048, 2.478592572, 3.304790096, 4.130987621, 4.32, 4.536, 4.656, 4.782464597, 4.98},
{0.0, 0.750731751, 1.501463501, 2.252195252, 3.002927002, 3.753658753, 3.937885471, 4.266847265, 4.298653178, 4.483667461, 4.627357952},
{0.0, 0.680162406, 1.360324811, 2.040487217, 2.720649622, 3.400812028, 3.614205802, 3.98465104, 4.071534267, 4.292819842, 4.401846895},
{0.0, 0.620054338, 1.240108677, 1.860163015, 2.480217353, 3.100271692, 3.369935137, 3.706572187, 3.936211621, 4.14958613, 4.366461768},
{0.0, 0.562298769, 1.124597539, 1.686896308, 2.249195078, 2.811493847, 3.103312565, 3.385263715, 3.609799528, 3.88206949, 4.213647473},
{0.0, 0.518753877, 1.037507754, 1.556261631, 2.075015508, 2.487750902, 2.782095282, 3.063783556, 3.210024353, 3.551030649, 3.918667235},
{0.0, 0.468635964, 0.937271929, 1.405907893, 1.874543857, 2.166076682, 2.413046573, 2.646927159, 2.8159419, 3.167575743, 3.406268171},
{0.0, 0.409012982, 0.818025965, 1.227038947, 1.636051929, 1.80523259, 2.000891124, 2.176912962, 2.386858793, 2.572562468, 2.826179682},
{0.0, 0.324468943, 0.648937885, 0.973406828, 1.29787577, 1.35121257, 1.512037636, 1.609255108, 1.68, 1.8792, 2.054568162}};

// Function that returns the flow rate in g/s given pressure in bar and cps
float findQ(float p, float c)
{
  int ip = 0, ic = 0;
  float fpc1, fpc2, fractionp, fractionc;

  p = constrain(p, Pn[0], Pn[Pcount - 1]);
  while (!between(p, Pn[ip], Pn[ip + 1])) ip++;
  fractionp = (p - Pn[ip]) / (Pn[ip + 1] - Pn[ip]);

  c = constrain(c, Cn[0], Cn[Ccount - 1]);
  while (!between(c, Cn[ic], Cn[ic + 1])) ic++;
  fractionc = (c - Cn[ic]) / (Cn[ic + 1] - Cn[ic]);

  fpc1 = Qn[ip][ic] + (Qn[ip + 1][ic] - Qn[ip][ic]) * fractionp;
  fpc2 = Qn[ip][ic + 1] + (Qn[ip + 1][ic + 1] - Qn[ip][ic + 1]) * fractionp;
  return fpc1 + (fpc2 - fpc1) * fractionc;
}

// Function that returns the cps, given pressure in bar and flow rate in g/s
float findC(float p, float q)
{
  int ip = 0, ic = 0;
  float Qp[Ccount], fractionp, fractionc;

  p = constrain(p, Pn[0], Pn[Pcount - 1]);
  while (!between(p, Pn[ip], Pn[ip + 1])) ip++;
  fractionp = (p - Pn[ip]) / (Pn[ip + 1] - Pn[ip]);

  for (int i = 0; i < Ccount; i++){
    Qp[i] = Qn[ip][i] + (Qn[ip + 1][i] - Qn[ip][i]) * fractionp;
  }

  q = constrain(q, Qp[0], Qp[Ccount - 1]);
  while (!between(q, Qp[ic], Qp[ic + 1])) ic++;
  fractionc = (q - Qp[ic]) / (Qp[ic + 1] - Qp[ic]);
  
  return Cn[ic] + (Cn[ic + 1] - Cn[ic]) * fractionc;
}

// Initialising some pump specific specs, mainly:
// - max pump clicks(dependant on region power grid spec)
// - pump clicks at 0 pressure in the system
void pumpInit(const int powerLineFrequency, const float pumpFlowAtZero) {
  // pump.freq = powerLineFrequency;
  maxPumpClicksPerSecond = powerLineFrequency;
  flowPerClickAtZeroBar = pumpFlowAtZero;
  fpc_multiplier = 60.f / (float)maxPumpClicksPerSecond;
}

// Function that returns the percentage of clicks the pump makes in it's current phase
inline float getPumpPct(const float targetPressure, const float flowRestriction, const SensorState &currentState) {
  if (targetPressure == 0.f) {
      return 0.f;
  }

  float diff = targetPressure - currentState.smoothedPressure;
  float maxPumpPct = flowRestriction <= 0.f ? 1.f : getClicksPerSecondForFlow(flowRestriction, currentState.smoothedPressure) / (float) maxPumpClicksPerSecond;
  float pumpPctToMaintainFlow = getClicksPerSecondForFlow(currentState.smoothedPumpFlow, currentState.smoothedPressure) / (float) maxPumpClicksPerSecond;

  if (diff > 2.f) {
    return fminf(maxPumpPct, 0.25f + 0.2f * diff);
  }

  if (diff > 0.f) {
    return fminf(maxPumpPct, pumpPctToMaintainFlow * 0.95f + 0.1f + 0.2f * diff);
  }

  if (currentState.pressureChangeSpeed < 0) {
    return fminf(maxPumpPct, pumpPctToMaintainFlow * 0.2f);
  }

  return 0;
}

// Sets the pump output based on a couple input params:
// - live system pressure
// - expected target
// - flow
// - pressure direction
void setPumpPressure(const float targetPressure, const float flowRestriction, const SensorState &currentState) {
  float pumpPct = getPumpPct(targetPressure, flowRestriction, currentState);
  setPumpToPercentage(pumpPct);
}

void setPumpOff(void) {
  setPumpToPercentage(0.0);
}

void setPumpFullOn(void) {
  setPumpToPercentage(1.0);
}

void setPumpToPercentage(float percentage) {
  currentPumpValue = (uint8_t) std::round(percentage * PUMP_RANGE);
  pump.set(currentPumpValue);
}

void pumpStopAfter(const uint8_t val) {
  pump.stopAfter(val);
}

long getAndResetClickCounter(void) {
  long counter = pump.getCounter();
  pump.resetCounter();
  return counter;
}

int getCPS(void) {
  watchdogReload();
  unsigned int cps = pump.cps();
  watchdogReload();
  if (cps > 80u) {
    pump.setDivider(2);
    pump.initTimer(cps > 110u ? 5000u : 6000u, TIM9);
  }
  else {
    pump.initTimer(cps > 55u ? 5000u : 6000u, TIM9);
  }
  return cps;
}

void pumpPhaseShift(void) {
  pump.shiftDividerCounter();
}

// Models the flow per click, follows a compromise between the schematic and recorded findings
// plotted: https://www.desmos.com/calculator/eqynzclagu
// float getPumpFlowPerClick(const float pressure) {
//   return findQ(pressure, (float)currentPumpValue / (float)PUMP_RANGE * (float)maxPumpClicksPerSecond);
// }

// Follows the schematic from https://www.cemegroup.com/solenoid-pump/e5-60 modified to per-click
float getPumpFlow(const float cps, const float pressure) {
  return findQ(pressure, cps);
}

// Currently there is no compensation for pressure measured at the puck, resulting in incorrect estimates
float getClicksPerSecondForFlow(const float flow, const float pressure) {
  if (flow == 0.f) return 0;
  float cps = findC(pressure, flow);
  return fminf(cps, (float)maxPumpClicksPerSecond);
}

// Calculates pump percentage for the requested flow and updates the pump raw value
void setPumpFlow(const float targetFlow, const float pressureRestriction, const SensorState &currentState) {
  // If a pressure restriction exists then the we go into pressure profile with a flowRestriction
  // which is equivalent but will achieve smoother pressure management
  if (pressureRestriction > 0.f && currentState.smoothedPressure > pressureRestriction * 0.5f) {
    setPumpPressure(pressureRestriction, targetFlow, currentState);
  }
  else {
    float pumpPct = getClicksPerSecondForFlow(targetFlow, currentState.smoothedPressure) / (float)maxPumpClicksPerSecond;
    setPumpToPercentage(pumpPct);
  }
}
