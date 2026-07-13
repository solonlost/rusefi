#pragma once

#include "global.h"
#include "engine_types.h"
#include "trigger_central.h"

enum class CxSyncSource {
	None,
	FlywheelPlusOne,
	CamPulse,
};

// Tooth geometry shared by all CX 145-tooth variants
static constexpr int CX_TOOTH_COUNT = 145;
static constexpr float CX_TOOTH_SPACING = 360.0f / CX_TOOTH_COUNT;
// Base (145M1): cam parks the counter so the very next tooth starts the cycle.
static constexpr int CX_BASE_PARK_INDEX = 144;
// Cam-check variant: park 8 teeth earlier so the cam sits ~20 degrees before the
// cycle boundary, clear of rusEFI's <7 degree "VVT sync position too close to
// trigger sync" warning window, and clear of the wrap in disagreement space.
// NOTE: this shifts tooth zero by 7 teeth (17.4 deg) vs the base variant -
// trigger angle calibration differs accordingly.
static constexpr int CX_CAM_CHECK_PARK_INDEX = 137;
// Cam-check: measured cam flutter at cranking is +/-0.65 tooth (chain wind-up
// plus quantization); re-anchor only above this, i.e. on genuine count corruption.
static constexpr int CX_CAM_REANCHOR_THRESHOLD = 3;

struct CitroenCxTriggerState {
	bool crankSynced = false;
	bool phaseSynced = false;

	int toothIndex = 0; // 0..144

	// Which crank revolution of the 720-degree engine cycle we are in (0 or 1).
	uint8_t revolution = 0;

	efitick_t lastPrimaryRise = 0;
	uint32_t lastToothPeriod = 0;

	// Cam-vs-count disagreement at the most recent cam pulse, in teeth, signed.
	// Live measurement of cam drive wind-up (chain wear gauge) on the checker
	// and P1 variants; always 0 on the base variant (cam re-anchors every pulse).
	int lastCamDisagreementTeeth = 0;
	// Number of times the cam forced a re-anchor after initial sync (checker
	// variant) or a parity correction (P1 variant). Nonzero after initial sync
	// indicates genuine count corruption, not flutter.
	uint32_t camReanchorCounter = 0;

	CxSyncSource syncSource = CxSyncSource::None;
};

inline bool isCitroenCxTrigger(trigger_type_e type) {
	return type == trigger_type_e::TT_CITROEN_CX_145M1_CRANK
		|| type == trigger_type_e::TT_CITROEN_CX_145P1_CRANK
		|| type == trigger_type_e::TT_CITROEN_CX_145_CAM_CHECK;
}

void resetCitroenCxTriggerState();
bool handleCitroenCxTrigger(trigger_type_e triggerType, trigger_event_e signal, efitick_t timestamp);
// Cam pulses arrive via the VVT input path (handleVvtCamSignal hook), never as
// shaft signals; SHAFT_SECONDARY_* in handleCitroenCxTrigger means the physical
// second trigger channel (145P1 flywheel single tooth) only.
void handleCitroenCxCamPulse(trigger_type_e triggerType);
const CitroenCxTriggerState& getCitroenCxTriggerState();

void initializeCitroenCxStub(TriggerWaveform *s);
