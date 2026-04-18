#include "pch.h"
#include "trigger_citroen_cx.h"
#include "trigger_universal.h"

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
	s.crankSynced = true;
	s.phaseSynced = true;
	s.syncSource = CxSyncSource::CamPulse;
}

bool handleCitroenCxTrigger(trigger_type_e triggerType, trigger_event_e signal, efitick_t timestamp) {
	switch (triggerType) {
	case trigger_type_e::TT_CITROEN_CX_145P1_CRANK:
		if (signal == SHAFT_PRIMARY_RISING) {
			handlePrimaryRise145(s_cxState, timestamp);
		}
		if (signal == SHAFT_SECONDARY_RISING) {
			handleSecondarySync145P1(s_cxState);
		}
		return false;
	case trigger_type_e::TT_CITROEN_CX_145M1_CRANK:
		if (signal == SHAFT_PRIMARY_RISING) {
			handlePrimaryRise145(s_cxState, timestamp);
		}
		if (signal == SHAFT_SECONDARY_RISING) {
			handleCamSync145(s_cxState);
		}
		return false;
	default:
		return false;
	}
}

void initializeCitroenCxStub(TriggerWaveform *s) {
	// Stub shape — real decoding bypasses TriggerWaveform due to PWM_PHASE_MAX_COUNT limit.
	initializeSkippedToothTrigger(s, 1, 0, FOUR_STROKE_CRANK_SENSOR, SyncEdge::RiseOnly);
	s->isSynchronizationNeeded = false;
}
