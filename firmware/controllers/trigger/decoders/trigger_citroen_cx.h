#pragma once

#include "global.h"
#include "engine_types.h"
#include "trigger_central.h"

enum class CxSyncSource {
	None,
	FlywheelPlusOne,
	CamPulse,
};

struct CitroenCxTriggerState {
	bool crankSynced = false;
	bool phaseSynced = false;

	int toothIndex = 0; // 0..144

	// Which crank revolution of the 720-degree engine cycle we are in (0 or 1).
	// Anchored by the cam pulse (145M1); meaningless for 145P1 until cam support exists.
	uint8_t revolution = 0;

	efitick_t lastPrimaryRise = 0;
	uint32_t lastToothPeriod = 0;

	CxSyncSource syncSource = CxSyncSource::None;
};

void resetCitroenCxTriggerState();
bool handleCitroenCxTrigger(trigger_type_e triggerType, trigger_event_e signal, efitick_t timestamp);
const CitroenCxTriggerState& getCitroenCxTriggerState();

void initializeCitroenCxStub(TriggerWaveform *s);
