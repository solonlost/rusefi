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

// Position in 720-degree tooth space, 0..289
static int position720(const CitroenCxTriggerState& s) {
	return s.toothIndex + s.revolution * CX_TOOTH_COUNT;
}

// Signed circular distance from expected to actual, in teeth, range [-145, 144]
static int signedToothDistance(int actual, int expected) {
	int d = (actual - expected) % (2 * CX_TOOTH_COUNT);
	if (d < 0) {
		d += 2 * CX_TOOTH_COUNT;
	}
	if (d >= CX_TOOTH_COUNT) {
		d -= 2 * CX_TOOTH_COUNT;
	}
	return d;
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
	if (s.toothIndex >= CX_TOOTH_COUNT) {
		s.toothIndex = 0;
		// New crank revolution within the 720-degree cycle
		s.revolution ^= 1;
	}
}

// 145P1: single added flywheel tooth on the second trigger channel, once per
// 360 degrees. Crank-anchored tooth zero: chain wind-up is excluded from the
// timing reference entirely. Parks the counter so the NEXT primary tooth is
// tooth 0; the wrap toggles the revolution bit as usual.
static void handleSecondarySync145P1(CitroenCxTriggerState& s) {
	if (s.crankSynced) {
		// Per-revolution count check: how far is the counter from where the
		// anchor expects it. Ring-gear health measurement; large values mean
		// dropped/spurious teeth.
		s.lastCamDisagreementTeeth = signedToothDistance(s.toothIndex, CX_BASE_PARK_INDEX);
	}
	s.crankSynced = true;
	s.syncSource = CxSyncSource::FlywheelPlusOne;
	s.toothIndex = CX_BASE_PARK_INDEX;
	// revolution intentionally untouched: parity belongs to the cam
}

// Base variant (145M1): the cam is the sole absolute reference. Park the
// counters so the NEXT primary rise becomes tooth 0 of revolution 0 - start of
// the engine cycle. 720-space index 0 then fires on a real crank tooth (which
// rpmShaftPositionCallback requires), and any accumulated tooth-count drift is
// re-anchored every cam pulse. Cost: cam drive wind-up feeds timing one-for-one.
static void handleCamSyncBase(CitroenCxTriggerState& s) {
	s.crankSynced = true;
	s.phaseSynced = true;
	s.syncSource = CxSyncSource::CamPulse;
	s.toothIndex = CX_BASE_PARK_INDEX;
	s.revolution = 1; // wraps to 0 together with toothIndex on the next tooth
}

// Cam-check variant: cam anchors once, then the free-running 145-tooth count is
// authoritative and the cam only verifies it. Re-anchor only when disagreement
// exceeds the threshold (genuine corruption); flutter from cam drive wind-up
// (measured +/-0.65 tooth at cranking) is recorded but does not move timing.
static void handleCamSyncWithCheck(CitroenCxTriggerState& s) {
	if (!s.phaseSynced) {
		// Initial anchor - identical role to the base variant, different park
		// index (see CX_CAM_CHECK_PARK_INDEX comment in the header).
		s.crankSynced = true;
		s.phaseSynced = true;
		s.syncSource = CxSyncSource::CamPulse;
		s.toothIndex = CX_CAM_CHECK_PARK_INDEX;
		s.revolution = 1;
		return;
	}

	int expected = CX_CAM_CHECK_PARK_INDEX + CX_TOOTH_COUNT; // (137, rev 1) in 720 space
	int d = signedToothDistance(position720(s), expected);
	s.lastCamDisagreementTeeth = d;

	if (d > CX_CAM_REANCHOR_THRESHOLD || d < -CX_CAM_REANCHOR_THRESHOLD) {
		s.toothIndex = CX_CAM_CHECK_PARK_INDEX;
		s.revolution = 1;
		s.camReanchorCounter++;
	}
}

// 145P1: cam provides 720-degree parity only. Tolerance is +/-179 degrees, so
// cam drive wind-up is irrelevant here. Mechanical requirement: the cam vane
// must be phased so its pulse arrives within the crank revolution preceding the
// cycle-start flywheel tooth (i.e. during revolution 1).
static void handleCamSync145P1(CitroenCxTriggerState& s) {
	if (s.phaseSynced && s.revolution != 1) {
		// Parity was wrong - either startup guess or a missed flywheel tooth
		// pattern; count it, it should be rare.
		s.camReanchorCounter++;
	}
	s.phaseSynced = true;
	s.revolution = 1;
	// crankSynced and toothIndex intentionally untouched: the flywheel single
	// tooth owns the crank anchor. Without it the engine does not run.
}

void handleCitroenCxCamPulse(trigger_type_e triggerType) {
	switch (triggerType) {
	case trigger_type_e::TT_CITROEN_CX_145M1_CRANK:
		handleCamSyncBase(s_cxState);
		break;
	case trigger_type_e::TT_CITROEN_CX_145_CAM_CHECK:
		handleCamSyncWithCheck(s_cxState);
		break;
	case trigger_type_e::TT_CITROEN_CX_145P1_CRANK:
		handleCamSync145P1(s_cxState);
		break;
	default:
		break;
	}
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
	case trigger_type_e::TT_CITROEN_CX_145_CAM_CHECK:
		if (signal == SHAFT_PRIMARY_RISING) {
			handlePrimaryRise145(s_cxState, timestamp);
		}
		return false;
	default:
		return false;
	}
}

void initializeCitroenCxStub(TriggerWaveform *s) {
	// Stub shape - real decoding bypasses TriggerWaveform due to PWM_PHASE_MAX_COUNT limit.
	initializeSkippedToothTrigger(s, 1, 0, FOUR_STROKE_CRANK_SENSOR, SyncEdge::RiseOnly);
	s->isSynchronizationNeeded = false;
}
