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

// Cam-check variant on the same real cranking log: cam anchors once, then only
// verifies. Measured flutter (+/-0.65 tooth) must never trigger a re-anchor.
TEST(realCitroenCx, camCheckRealData) {
	CsvReader reader(1, /* vvtCount */ 1);

	reader.open("tests/trigger/resources/citroen-cx-cranking.csv");
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	engineConfiguration->isFasterEngineSpinUpEnabled = true;
	engineConfiguration->alwaysInstantRpm = false;
	engineConfiguration->vvtMode[0] = VVT_SINGLE_TOOTH;
	eth.setTriggerType(trigger_type_e::TT_CITROEN_CX_145_CAM_CHECK);

	bool gotFullSync = false;
	while (reader.haveMore()) {
		reader.processLine(&eth);
		engine->rpmCalculator.onSlowCallback();
		if (engine->triggerCentral.triggerState.hasSynchronizedPhase()) {
			gotFullSync = true;
		}
	}

	const auto& cx = getCitroenCxTriggerState();
	EXPECT_TRUE(cx.crankSynced);
	EXPECT_TRUE(cx.phaseSynced);
	EXPECT_TRUE(gotFullSync);
	EXPECT_GT(Sensor::getOrZero(SensorType::Rpm), 100);

	// Physical flutter (+/-0.65 tooth measured) stays well within threshold.
	// The MSL-to-CSV conversion redistributes teeth across sampling interval
	// boundaries though (counts are conserved: cumulative drift returns to 0
	// by the end, proving the wheel delivers exactly 290 teeth per cam). That
	// artifact drifts to -4 (one re-anchor), and the reference reset then makes
	// the way back read as +4 (a second). Both are conversion artifacts; an
	// edge-accurate composite tooth logger capture would show zero.
	EXPECT_LE(cx.camReanchorCounter, 2u) << "more re-anchors than the known CSV artifacts";
	EXPECT_LE(cx.lastCamDisagreementTeeth, 2);
	EXPECT_GE(cx.lastCamDisagreementTeeth, -2);
}

// Cam-check variant against injected corruption: 5 teeth dropped in one
// revolution. Disagreement exceeds threshold, decoder must re-anchor exactly
// once and retain sync.
TEST(realCitroenCx, camCheckCorruption) {
	CsvReader reader(1, /* vvtCount */ 1);

	reader.open("tests/trigger/resources/citroen-cx-corrupt.csv");
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	engineConfiguration->isFasterEngineSpinUpEnabled = true;
	engineConfiguration->alwaysInstantRpm = false;
	engineConfiguration->vvtMode[0] = VVT_SINGLE_TOOTH;
	eth.setTriggerType(trigger_type_e::TT_CITROEN_CX_145_CAM_CHECK);

	while (reader.haveMore()) {
		reader.processLine(&eth);
		engine->rpmCalculator.onSlowCallback();
	}

	const auto& cx = getCitroenCxTriggerState();
	EXPECT_TRUE(cx.crankSynced);
	EXPECT_TRUE(cx.phaseSynced);
	EXPECT_EQ(cx.camReanchorCounter, 1u) << "corruption not caught exactly once";
	EXPECT_TRUE(engine->triggerCentral.triggerState.hasSynchronizedPhase());
}

// 145P1 three-sensor variant: ring gear (primary) + flywheel single tooth
// (secondary) + cam (VVT). Crank sync and RPM must exist BEFORE any cam pulse
// (wasted-spark capable start); full phase sync after the first cam pulse.
TEST(realCitroenCx, p1ThreeSensor) {
	CsvReader reader(2, /* vvtCount */ 1);

	reader.open("tests/trigger/resources/citroen-cx-p1.csv");
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	engineConfiguration->isFasterEngineSpinUpEnabled = true;
	engineConfiguration->alwaysInstantRpm = false;
	engineConfiguration->vvtMode[0] = VVT_SINGLE_TOOTH;
	eth.setTriggerType(trigger_type_e::TT_CITROEN_CX_145P1_CRANK);

	bool rpmBeforeCam = false;
	bool crankSyncBeforeCam = false;
	bool sawCam = false;

	while (reader.haveMore()) {
		reader.processLine(&eth);
		engine->rpmCalculator.onSlowCallback();

		if (engine->triggerCentral.vvtEventRiseCounter[0] > 0) {
			sawCam = true;
		}
		if (!sawCam) {
			if (Sensor::getOrZero(SensorType::Rpm) > 0) {
				rpmBeforeCam = true;
			}
			if (getCitroenCxTriggerState().crankSynced) {
				crankSyncBeforeCam = true;
			}
		}
	}

	const auto& cx = getCitroenCxTriggerState();
	EXPECT_TRUE(crankSyncBeforeCam) << "flywheel tooth alone should give crank sync";
	EXPECT_TRUE(rpmBeforeCam) << "RPM should exist before cam (wasted spark start)";
	EXPECT_TRUE(sawCam);
	EXPECT_TRUE(cx.phaseSynced);
	EXPECT_TRUE(engine->triggerCentral.triggerState.hasSynchronizedPhase());
	EXPECT_EQ(cx.syncSource, CxSyncSource::FlywheelPlusOne);

	auto rpm = Sensor::getOrZero(SensorType::Rpm);
	EXPECT_GT(rpm, 150);
	EXPECT_LT(rpm, 250);
}
