'use strict';

// User-defined display scale, not BMS SOC or a motor/power cutoff setting.
const AMR_BATTERY_PROFILE = Object.freeze({
  model: 'DMBP-LF256-8S6P-B35',
  nominalVoltage: 25.6,
  capacityAh: 19.8,
  emptyVoltage: 25.6,
  fullVoltage: 27.4,
});

function estimateOperationalBattery(voltage) {
  if (typeof voltage !== 'number' || !Number.isFinite(voltage) || voltage <= 0) return null;
  const {emptyVoltage, fullVoltage} = AMR_BATTERY_PROFILE;
  const percent = (voltage - emptyVoltage) / (fullVoltage - emptyVoltage) * 100;
  return Math.min(100, Math.max(0, Math.round(percent)));
}
