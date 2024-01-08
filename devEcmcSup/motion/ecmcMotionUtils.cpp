
#include "ecmcMotionUtils.h"
#include <cmath>
#include "ecmcDefinitions.h"

double ecmcMotionUtils::getPosErrorModWithSign(double set,
                                               double setOld,
                                               double act,
                                               double modRange) {
  double normalCaseError = set - act;

  if (modRange == 0) {
    return normalCaseError;
  }

  // Modulo
  if (std::abs(normalCaseError) < modRange * ECMC_OVER_UNDER_FLOW_FACTOR) {
    // No overflows
    return normalCaseError;
  } else {
    // Overflow has happended in either encoder or setpoint
    double overUnderFlowError = modRange - std::abs(normalCaseError);
    double setDiff            = set - setOld;

    // Moving forward (overflow)
    if ((setDiff > 0) || (setDiff < -modRange * ECMC_OVER_UNDER_FLOW_FACTOR)) {
      // Actual lagging setpoint  ACT SET
      if (act > set) {
        return overUnderFlowError;
      } else {   // Actual before setpoint SET ACT
        return -overUnderFlowError;
      }
    } else {
      // Moving backward (underflow)
      // Actual lagging setpoint SET ACT
      if (act < set) {
        return -overUnderFlowError;
      } else {   // Actual before setpoint ACT SET
        return overUnderFlowError;
      }
    }
  }

  return 0;
}

double ecmcMotionUtils::getPosErrorModAbs(double set,
                                          double act,
                                          double modRange) {
  double normalCaseError = set - act;

  if (modRange == 0) {
    return std::abs(normalCaseError);
  }

  // Modulo
  if (std::abs(normalCaseError) < modRange * ECMC_OVER_UNDER_FLOW_FACTOR) {
    // No overflows
    return normalCaseError;
  } else {
    // Overflow has happended in either encoder or setpoint
    return std::abs(modRange - std::abs(normalCaseError));
  }

  return 0;
}
