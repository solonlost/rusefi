/*
 * @file test_real_citroen_cx.cpp
 *
 * Citroen CX M25 trigger: 145-tooth flywheel, variant 145M1 (one 0.3mm
 * shortened tooth, not decoded) with a single-tooth cam Hall sensor on the
 * VVT input as the once-per-720-degree engine sync reference.
 *
 * The 145-tooth count exceeds PWM_PHASE_MAX_COUNT, so decoding bypasses
 * TriggerWaveform entirely (see trigger_citroen_cx.cpp) and the cam arrives
 * via handleVvtCamSignal, not as a secondary shaft signal.
 */

#include "pch.h"
#include "logicdata_csv_reader.h"
#include "trigger_citroen_cx.h"

TEST(realCitroenCx, cranking145M1) {
	CsvReader reader(1, /* vvtCount */ 1);

	reader.open("tests/trigger/resources/citroen-cx-cranking.csv");
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	engineConfiguration->isFasterEngineSpinUpEnabled = true;
	engineConfiguration->alwaysInstantRpm = false;

	engineConfiguration->vvtMode[0] = VVT_SINGLE_TOOTH;
	eth.setTriggerType(trigger_type_e::TT_CITROEN_CX_145M1_CRANK);

	bool gotRpm = false;
	bool gotFullSync = false;
	int eventCount = 0;
	int rpmAtEvent = 0;
	int fullSyncAtEvent = 0;

	while (reader.haveMore()) {
		reader.processLine(&eth);
		eventCount++;
		engine->rpmCalculator.onSlowCallback();

		auto rpm = Sensor::getOrZero(SensorType::Rpm);
		if (!gotRpm && rpm > 0) {
			gotRpm = true;
			rpmAtEvent = eventCount;
		}

		if (!gotFullSync && engine->triggerCentral.triggerState.hasSynchronizedPhase()) {
			gotFullSync = true;
			fullSyncAtEvent = eventCount;
		}
	}

	const auto& cx = getCitroenCxTriggerState();
	EXPECT_TRUE(cx.crankSynced);
	EXPECT_TRUE(cx.phaseSynced);

	EXPECT_TRUE(gotRpm) << "never got RPM";
	EXPECT_TRUE(gotFullSync) << "never got full (720 degree) sync";

	// Cranking speed on the starter: expect something plausible
	auto rpm = Sensor::getOrZero(SensorType::Rpm);
	EXPECT_GT(rpm, 100);
	EXPECT_LT(rpm, 500);

	// Sync budget: within ~6 crank revolutions. One revolution is 145 rise+fall
	// CSV lines on the crank channel plus cam lines, so allow 6 * 300 events.
	if (gotFullSync) {
		EXPECT_LT(fullSyncAtEvent, 6 * 300) << "full sync too slow, at event " << fullSyncAtEvent;
	}
}
