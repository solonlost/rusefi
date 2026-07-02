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
		// Stall detection: the standard decoder does this inside decodeTriggerEvent
		// (1 second gap => sync lost), which the CX path bypasses. Without this a
		// stall leaves us "synced" with a stale tooth index on the next start.
		if (timestamp - s.lastPrimaryRise > US2NT(1'000'000)) {
			s = CitroenCxTriggerState{};
			s.lastPrimaryRise = timestamp;
			return;
		}
		uint32_t thisPeriod = (uint32_t)(timestamp - s.lastPrimaryRise);
		s.lastToothPeriod = thisPeriod;
	}

	s.lastPrimaryRise = timestamp;

	s.toothIndex++;
	if (s.toothIndex >= 145) {
		s.toothIndex = 0;
		// New crank revolution within the 720-degree cycle
		s.revolution ^= 1;
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

	// The cam pulse is the absolute once-per-720-degree reference (the shortened
	// flywheel tooth is not decoded). Park the counters so the NEXT primary rise
	// becomes tooth 0 of revolution 0, i.e. the start of the engine cycle. This
	// makes 720-space trigger index 0 fire on a real crank tooth, which
	// rpmShaftPositionCallback requires for its once-per-cycle RPM math, and
	// re-anchors any accumulated tooth-count drift every cam pulse.
	s.toothIndex = 144;
	s.revolution = 1; // wraps to 0 together with toothIndex on the next tooth
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
