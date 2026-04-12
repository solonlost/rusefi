#include "pch.h"
#include "trigger_citroen_cx.h"

static CitroenCxTriggerState s_cxState;

void resetCitroenCxTriggerState() {
	s_cxState = CitroenCxTriggerState{};
}

const CitroenCxTriggerState& getCitroenCxTriggerState() {
	return s_cxState;
}

static void handlePrimaryRise145(CitroenCxTriggerState& s, efitick_t timestamp) {
	if (s.lastPrimaryRise != 0) {
		uint32_t thisPeriod = (uint32_t)(timestamp - s.lastPrimaryRise);
		s.lastToothPeriod = thisPeriod;
	}

	s.lastPrimaryRise = timestamp;

	s.toothIndex++;
	if (s.toothIndex >= 145) {
		s.toothIndex = 0;
	}
}

static void handleSecondarySync145P1(CitroenCxTriggerState& s) {
	s.crankSynced = true;
	s.syncSource = CxSyncSource::FlywheelPlusOne;

	// Temporary assumption:
	// secondary sync event defines tooth zero
	s.toothIndex = 0;
}

static void handleCamSync145(CitroenCxTriggerState& s) {
	s.phaseSynced = true;
	s.syncSource = CxSyncSource::CamPulse;
}

bool handleCitroenCxTrigger(trigger_type_e triggerType, trigger_event_e signal, efitick_t timestamp) {
	switch (triggerType) {
	case trigger_type_e::TT_CITROEN_CX_145P1_CRANK:
		if (signal == SHAFT_PRIMARY_RISING) {
			handlePrimaryRise145(s_cxState, timestamp);
			return true;
		}

		if (signal == SHAFT_SECONDARY_RISING) {
			handleSecondarySync145P1(s_cxState);
			return true;
		}

		return true;

	case trigger_type_e::TT_CITROEN_CX_145M1_CRANK:
		if (signal == SHAFT_PRIMARY_RISING) {
			handlePrimaryRise145(s_cxState, timestamp);
			return true;
		}

		if (signal == SHAFT_SECONDARY_RISING) {
			handleCamSync145(s_cxState);
			return true;
		}

		return true;

	default:
		return false;
	}
}
