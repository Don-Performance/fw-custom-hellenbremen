#include "pch.h"

#include "board_can.h"
#include "board_config.h"
#include "board_autotune.h"
#include "board_atIdleBase.h"
#include "board_types.h"

#include <cstddef>
#include <cstring>

#include "rusefi/crc.h"

#include "board_etb_maps.h"
#include "board_instant_accel_shot.h"
#include "board_etb_control.h"
#include "defaults.h"
#include "hellen_meta.h"

#if EFI_PROD_CODE && EFI_USE_OPENBLT
/* communication with OpenBLT that is plain C */
extern "C" {
	#include "openblt/shared_params.h"
};
#endif

#define HARLEY_V_TWIN 45.0

static void o2SensorInitialization() {
	if(config->useWidebandSensor) {
		engineConfiguration->afr.hwChannel  = EFI_ADC_NONE;
		engineConfiguration->afr.hwChannel2 = EFI_ADC_NONE;

		// Heater
		engineConfiguration->gppwm[0].pin = Gpio::Unassigned;
		engineConfiguration->gppwm[1].pin = Gpio::Unassigned;

		engineConfiguration->enableAemXSeries = true;
		engineConfiguration->canWbo[0].type = can_wbo_type_e::RUSEFI;

		engineConfiguration->fuelClosedLoopCorrectionEnabled = true;
	} else {
		engineConfiguration->afr.hwChannel  = EFI_ADC_7; // PA7
		engineConfiguration->afr.hwChannel2 = EFI_ADC_6; // PA6

		engineConfiguration->gppwm[0].pin = Gpio::D15;
		engineConfiguration->gppwm[1].pin = Gpio::D14;

		engineConfiguration->enableAemXSeries = false;
		engineConfiguration->canWbo[0].type = can_wbo_type_e::DISABLED;

		engineConfiguration->fuelClosedLoopCorrectionEnabled = false;
	}

	return;
}

static void setDpecmHardwareInfo();

// Executed after flash when empty or when factory resetted
void boardDefaultConfiguration() {
	setDpecmHardwareInfo();

	// Trigger
	engineConfiguration->overrideTriggerGaps = true;
	engineConfiguration->gapTrackingLengthOverride = 3;
	engineConfiguration->triggerGapOverrideFrom[0] = 0.75;
	engineConfiguration->triggerGapOverrideTo[0] = 1.25;
	engineConfiguration->triggerGapOverrideFrom[1] = 0.05; // this one is custom
	engineConfiguration->triggerGapOverrideTo[1] = 0.5;
	engineConfiguration->triggerGapOverrideFrom[2] = 1.850; // this one is custom
	engineConfiguration->triggerGapOverrideTo[2] = 6;

	// Cam
	// This isn't a physical VVT solenoid - there is none on this bike. VVT_MAP_V_TWIN
	// is this engine's cam/phase-sync mechanism (via MAP signal), since it has no cam sensor.
	engineConfiguration->vvtMode[0] = VVT_MAP_V_TWIN;
	engineConfiguration->mapCamDetectionAnglePosition = 50;

	// Widened cam gaps for VVT_BOSCH_QUICK_START (unused with MAP cam sync): V-twin cranking speed
	// ripple pushes the sync ratios (nominal 0.645 / 1.556) to ~0.47 / ~1.10, just outside the stock +-25%
	// Note: override applies to whichever decoder cam 1 uses - review when selecting a different cam wheel
	engineConfiguration->overrideVvtTriggerGaps = true;
	engineConfiguration->gapVvtTrackingLengthOverride = 2;
	engineConfiguration->triggerVVTGapOverrideFrom[0] = 0.35;
	engineConfiguration->triggerVVTGapOverrideTo[0] = 0.85;
	engineConfiguration->triggerVVTGapOverrideFrom[1] = 0.95;
	engineConfiguration->triggerVVTGapOverrideTo[1] = 2.0;

	// Aux Outputs
	engineConfiguration->fanPin = Gpio::Unassigned;
	engineConfiguration->fanOnTemperature = 0.f;
	engineConfiguration->fanOffTemperature = 0.f;
	engineConfiguration->fan2Pin = Gpio::Unassigned;
	engineConfiguration->fan2OnTemperature = 0.f;
	engineConfiguration->fan2OffTemperature = 0.f;
	engineConfiguration->mainRelayPin = Gpio::Unassigned;

	// Fuel level sensor
	engineConfiguration->fuelLevelSensor = EFI_ADC_2;
	config->fuelLevelTankVolume = 18.9f; // H-D Softail Low Rider ST 2025 FXLRST
	config->fuelLevelAvgConsumption = 5.5f; // TODO This is hardcoded for now. Has to be dynamic
	engineConfiguration->fuelLevelUpdatePeriodSec = 0.1f;
	engineConfiguration->fuelLevelAveragingAlpha = 0.0001f;
	engineConfiguration->fuelLevelLowThresholdVoltage = 0.5f;
	engineConfiguration->fuelLevelHighThresholdVoltage = 3.0f;

	// The fuel level sensor should have a resistance of 250 Ohm (full) - 60 Ohm (empty)
	// At supply voltage of ideally 5V, this should result in 0.968V - 2.5V, respectively.
	// Making it narrower due to supply voltage variation.
	config->fuelLevelBins[0] = 0.975f;
	config->fuelLevelBins[1] = 1.184f;
	config->fuelLevelBins[2] = 1.394f;
	config->fuelLevelBins[3] = 1.603f;
	config->fuelLevelBins[4] = 1.812f;
	config->fuelLevelBins[5] = 2.021f;
	config->fuelLevelBins[6] = 2.231f;
	config->fuelLevelBins[7] = 2.440f;

	config->fuelLevelValues[0] = 100;
	config->fuelLevelValues[1] = 86;
	config->fuelLevelValues[2] = 71;
	config->fuelLevelValues[3] = 57;
	config->fuelLevelValues[4] = 43;
	config->fuelLevelValues[5] = 29;
	config->fuelLevelValues[6] = 14;
	config->fuelLevelValues[7] = 0;

	// AFR - wideband O2 via CAN (rusEFI native protocol on CAN1)
	config->useWidebandSensor = true;
	engineConfiguration->widebandOnSecondBus = false; // Use CAN1
	engineConfiguration->canWbo[1].type = can_wbo_type_e::DISABLED;
	o2SensorInitialization();

	engineConfiguration->afrExpAverageAlpha = 1; // no smoothing

	// MAP
	setCustomMap(/*lowValue*/ 20, /*mapLowValueVoltage*/ 0.79, /*highValue*/ 101.3, /*mapHighValueVoltage*/ 4);

	// TPS
	engineConfiguration->tpsMin = 0.50;
	engineConfiguration->tpsMax = 4.54;
	engineConfiguration->tps1SecondaryMin = 4.53;
	engineConfiguration->tps1SecondaryMax = 0.47;

	engineConfiguration->throttlePedalUpVoltage = 1.36;
	engineConfiguration->throttlePedalWOTVoltage = 4.46;
	engineConfiguration->throttlePedalSecondaryUpVoltage = 1.37;
	engineConfiguration->throttlePedalSecondaryWOTVoltage = 4.48;

	boardEtbMapsApplyDefaults();
	boardInstantAccelApplyDefaults();
	boardEtbControlApplyDefaults();
	engineConfiguration->idlePositionLowerLimit = 14;

	setRpmTableBin(config->estimatedEngineTorqueRpmBins);
	setLinearCurve(config->estimatedEngineTorqueTpsBins, 0.0f, 100.0f, 1.0f);
	setTable(config->estimatedEngineTorqueTable, 0);

	// Vehicle equipment
	config->jssInstalled = true;
	config->prgselInstalled = true;

	// Cruise control
	config->ccFootBrakeDisengageThreshold = 50;
	config->cruiseSetpointRampRate = 2.0f;

	// Dashboard output
	config->canRpmFilterAlpha = 0.2f;

	// PRGSEL
	config->prgselOutputPin = Gpio::D10;
	config->prgselActive = true;
	config->prgselRPM = 2000;
	config->prgselSpeed = 10;
	config->prgselLowerTGS = 5;
	config->prgselUpperTGS = 75;
	config->prgselEtsTemp = 90;
	config->prgselPWMFreq = 32;
	config->prgselPWMDuty = 30;
	config->prgselActAfterTime = 180;

	// Autotune
	// RPM bins
	config->lambdaDelayRpmBins[0] = 800;
	config->lambdaDelayRpmBins[1] = 2190;
	config->lambdaDelayRpmBins[2] = 6500;

	// Load bins - TPS%, matching this engine's VE table load override (veOverrideMode = VE_TPS)
	config->lambdaDelayLoadBins[0] = 2;
	config->lambdaDelayLoadBins[1] = 35;
	config->lambdaDelayLoadBins[2] = 100;

	// [load][rpm], ms
	config->lambdaDelayTable[0][0] = 350; // load=2,   rpm=800
	config->lambdaDelayTable[0][1] = 300; // load=2,   rpm=2190
	config->lambdaDelayTable[0][2] = 20;  // load=2,   rpm=6500
	config->lambdaDelayTable[1][0] = 250; // load=35,  rpm=800
	config->lambdaDelayTable[1][1] = 150; // load=35,  rpm=2190
	config->lambdaDelayTable[1][2] = 100; // load=35,  rpm=6500
	config->lambdaDelayTable[2][0] = 200; // load=100, rpm=800
	config->lambdaDelayTable[2][1] = 70;  // load=100, rpm=2190
	config->lambdaDelayTable[2][2] = 40;  // load=100, rpm=6500

	// Autotune conditions
	config->autotuneMinRPM = 700;
	config->autotuneMinETS = 70;
	config->autotuneMaxETS = 140;
	config->autotuneMinAFR = 8.0f;
	config->autotuneMaxAFR = 20.0f;

	config->autotuneAutoApply = true;
	config->autotuneAutoBurn = true;
	config->autotuneApplyPeriod = 5;

	// Autotune cell change resistance presets
	config->autotuneVeryHighInitialWeight = 100.0f;
	config->autotuneVeryHighWeightThreshold = 0.25f;
	config->autotuneVeryHighDeadband = 0.5f;
	config->autotuneVeryHighMaxWeight = 100000.0f;

	config->autotuneHighInitialWeight = 20.0f;
	config->autotuneHighWeightThreshold = 0.1f;
	config->autotuneHighDeadband = 0.0f;
	config->autotuneHighMaxWeight = 1000.0f;

	config->autotuneNormalInitialWeight = 5.0f;
	config->autotuneNormalWeightThreshold = 0.0f;
	config->autotuneNormalDeadband = 0.0f;
	config->autotuneNormalMaxWeight = 300.0f;

	config->autotuneLowInitialWeight = 3.0f;
	config->autotuneLowWeightThreshold = 0.0f;
	config->autotuneLowDeadband = 0.0f;
	config->autotuneLowMaxWeight = 100.0f;

	config->autotuneVeryLowInitialWeight = 0.5f;
	config->autotuneVeryLowWeightThreshold = 0.0f;
	config->autotuneVeryLowDeadband = 0.0f;
	config->autotuneVeryLowMaxWeight = 5.0f;

	config->autotuneAutomaticCellChangeResistance = true;
	config->autotuneCellChangeResistance = autotuneCellChangeResistance_e::Normal;
	config->autotuneActiveInitialWeight = config->autotuneNormalInitialWeight;
	config->autotuneActiveWeightThreshold = config->autotuneNormalWeightThreshold;
	config->autotuneActiveDeadband = config->autotuneNormalDeadband;
	config->autotuneActiveMaxWeight = config->autotuneNormalMaxWeight;

	config->autotuneMaxAbsoluteChange = 50.0f;
	config->autotuneMaxPercentageChange = 50.0f;

	// Idle base autotune conditions
	config->atIdleBaseMinRpm = 700;
	config->atIdleBaseMinClt = 40;
	config->atIdleBaseMaxClt = 100;
	config->atIdleBaseMaxTgs = 1;

	config->atIdleBaseAutoApply = true;
	config->atIdleBaseAutoBurn  = true;
	config->atIdleBaseApplyPeriod = 5;

	// Idle base autotune cell change resistance presets
	config->atIdleBaseVeryHighInitialWeight = 100.0f;
	config->atIdleBaseVeryHighWeightThreshold = 0.25f;
	config->atIdleBaseVeryHighDeadband = 0.5f;
	config->atIdleBaseVeryHighMaxWeight = 100000.0f;

	config->atIdleBaseHighInitialWeight = 20.0f;
	config->atIdleBaseHighWeightThreshold = 0.1f;
	config->atIdleBaseHighDeadband = 0.0f;
	config->atIdleBaseHighMaxWeight = 1000.0f;

	config->atIdleBaseNormalInitialWeight = 5.0f;
	config->atIdleBaseNormalWeightThreshold = 0.0f;
	config->atIdleBaseNormalDeadband = 0.0f;
	config->atIdleBaseNormalMaxWeight = 300.0f;

	config->atIdleBaseLowInitialWeight = 3.0f;
	config->atIdleBaseLowWeightThreshold = 0.0f;
	config->atIdleBaseLowDeadband = 0.0f;
	config->atIdleBaseLowMaxWeight = 100.0f;

	config->atIdleBaseVeryLowInitialWeight = 0.5f;
	config->atIdleBaseVeryLowWeightThreshold = 0.0f;
	config->atIdleBaseVeryLowDeadband = 0.0f;
	config->atIdleBaseVeryLowMaxWeight = 5.0f;

	config->atIdleBaseCellChangeResistance = autotuneCellChangeResistance_e::Normal;
	config->atIdleBaseActiveInitialWeight = config->atIdleBaseNormalInitialWeight;
	config->atIdleBaseActiveWeightThreshold = config->atIdleBaseNormalWeightThreshold;
	config->atIdleBaseActiveDeadband = config->atIdleBaseNormalDeadband;
	config->atIdleBaseActiveMaxWeight = config->atIdleBaseNormalMaxWeight;

	config->atIdleBaseMaxAbsoluteChange = 10.0f;
	config->atIdleBaseMaxPercentageChange = 50.0f;
}

static void boardSanitizeConfig() {
	// PRGSEL
	if (config->prgselLowerTGS > config->prgselUpperTGS) {
		config->prgselLowerTGS = config->prgselUpperTGS;
	}

	// Vmax
	if (config->vmaxLimit > 0 && config->vmaxLimitRange == 0) {
		config->vmaxLimitRange = 1;
	}

	// Dashboard RPM filter: invalid or missing value (e.g. 0 after layout change) means unfiltered
	if (!(config->canRpmFilterAlpha > 0.0f && config->canRpmFilterAlpha <= 1.0f)) {
		config->canRpmFilterAlpha = 1.0f;
	}

	// Autotune Idle Base
	if (config->atIdleBaseMaxClt < config->atIdleBaseMinClt) {
		config->atIdleBaseMaxClt = config->atIdleBaseMinClt;
	}
}

// Writes the 96-bit STM32 unique ID as 24 uppercase hex characters, most significant word first
// (UID[2] UID[1] UID[0]). The field is fixed-length and not null-terminated, like vinNumber.
static void setDpecmHardwareId() {
	static_assert(sizeof(config->dpecmHardwareID) == 3 * 8, "dpecmHardwareID must hold 3 words as hex");

#if EFI_PROD_CODE
	const uint32_t* uid = reinterpret_cast<const uint32_t*>(UID_BASE);
	const uint32_t words[3] = { uid[2], uid[1], uid[0] };
#else
	const uint32_t words[3] = { 0, 0, 0 };
#endif // EFI_PROD_CODE

	static const char hexDigits[] = "0123456789ABCDEF";
	size_t pos = 0;
	for (uint32_t word : words) {
		for (int shift = 28; shift >= 0; shift -= 4) {
			config->dpecmHardwareID[pos++] = hexDigits[(word >> shift) & 0xF];
		}
	}
}

// Hardware revision block in the STM32F7 OTP area (16 blocks x 64 bytes at FLASH_OTP_BASE).
// Written once in production over SWD; see bin/make_hw_rev_otp.py, which must match this layout.
struct DpecmOtpBlock {
	char magic[4];          // "DPHW"
	uint8_t layoutVersion;  // kDpecmOtpLayoutVersion
	char hardwareRev[16];   // printable ASCII, padded with '\0'
	uint8_t reserved[39];
	uint32_t crc;           // CRC32 (zlib) over bytes 0..59, little-endian
};

static_assert(sizeof(DpecmOtpBlock) == 64, "OTP block must be 64 bytes");
static_assert(offsetof(DpecmOtpBlock, crc) == 60, "CRC must be the last 4 bytes of the OTP block");
static_assert(sizeof(DpecmOtpBlock::hardwareRev) == sizeof(dpecm_hardware_rev_t), "OTP revision must match dpecmHardwareRev");

#if EFI_PROD_CODE
static constexpr size_t kDpecmOtpBlockCount = 16;
static constexpr uint8_t kDpecmOtpLayoutVersion = 1;

static_assert(FLASH_OTP_BASE + kDpecmOtpBlockCount * sizeof(DpecmOtpBlock) <= FLASH_OTP_END + 1, "OTP data blocks out of range");

static bool isOtpBlockErased(const DpecmOtpBlock& block) {
	const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&block);
	for (size_t i = 0; i < sizeof(block); i++) {
		if (bytes[i] != 0xFF) {
			return false;
		}
	}
	return true;
}

static bool isOtpBlockValid(const DpecmOtpBlock& block) {
	if (std::memcmp(block.magic, "DPHW", sizeof(block.magic)) != 0) {
		return false;
	}
	if (block.layoutVersion != kDpecmOtpLayoutVersion) {
		return false;
	}
	if (crc32(&block, offsetof(DpecmOtpBlock, crc)) != block.crc) {
		return false;
	}
	// Printable text, then only '\0' padding
	bool padding = false;
	for (char c : block.hardwareRev) {
		if (c == '\0') {
			padding = true;
		} else if (padding || c < 0x20 || c > 0x7E) {
			return false;
		}
	}
	return block.hardwareRev[0] != '\0';
}
#endif // EFI_PROD_CODE

// The last valid OTP block wins, so a wrong entry can be superseded by programming the next block.
static void setDpecmHardwareRev() {
	const char* fallback = "UNPROGRAMMED";
	const DpecmOtpBlock* found = nullptr;

#if EFI_PROD_CODE
	const DpecmOtpBlock* blocks = reinterpret_cast<const DpecmOtpBlock*>(FLASH_OTP_BASE);
	bool anyProgrammed = false;
	for (size_t i = 0; i < kDpecmOtpBlockCount; i++) {
		if (isOtpBlockErased(blocks[i])) {
			continue;
		}
		anyProgrammed = true;
		if (isOtpBlockValid(blocks[i])) {
			found = &blocks[i];
		}
	}
	if (found == nullptr && anyProgrammed) {
		fallback = "INVALID";
	}
#endif // EFI_PROD_CODE

	std::memset(config->dpecmHardwareRev, 0, sizeof(config->dpecmHardwareRev));
	if (found != nullptr) {
		std::memcpy(config->dpecmHardwareRev, found->hardwareRev, sizeof(config->dpecmHardwareRev));
	} else {
		std::memcpy(config->dpecmHardwareRev, fallback, std::strlen(fallback));
	}
}

// dpecmHardwareID / dpecmHardwareRev live in the persistent configuration but are set by the firmware:
// on boot (first boardConfigOverrides() call), on reset to defaults (boardDefaultConfiguration()) and
// on burn (boardCustomOnConfigurationChange(), which runs before the flash write is scheduled).
// Deliberately NOT on every TS page write: rusEFI calls boardConfigOverrides() after each settings
// chunk write, and rewriting the bytes there makes tools that write a whole map and read it back
// (e.g. the web application) fail their verification.
static void setDpecmHardwareInfo() {
	setDpecmHardwareId();
	setDpecmHardwareRev();
}

// Set on boot, reset to defaults, burn and after every TS write to the settings page
void boardConfigOverrides() {
	static bool hardwareInfoSet = false;
	if (!hardwareInfoSet) {
		setDpecmHardwareInfo();
		hardwareInfoSet = true;
	}

	setHellenAnalogDividers();
	setHellenCan();
	hellenMegaSdWithAccelerometer();

	// VBATT is on PA0
	engineConfiguration->vbattAdcChannel = EFI_ADC_0;
	// set vbatt_divider 4,0303
	// 10k / 3.3k
	engineConfiguration->vbattDividerCoeff = 8.06f;

	// PWR EN
	setHellenEnPin(Gpio::E0, true);

	// Generic
	engineConfiguration->cylindersCount = 2;
	engineConfiguration->firingOrder = FO_1_2;
	strcpy(engineConfiguration->engineMake, "Harley");

	// Knock
	// engineConfiguration->enableSoftwareKnock = true;
	// engineConfiguration->knockFrequency = 0.0f; 

	// Sensors
	engineConfiguration->map.sensor.hwChannel = EFI_ADC_13; // PC3
	engineConfiguration->fuelLevelSensor = EFI_ADC_2; // PA2

	// Gear is detected from AuxLinear1 in board_can.cpp, not rusEFI's ratio-based GearDetector.
	// Keep this 0 so GearDetector never registers SensorType::DetectedGear itself.
	engineConfiguration->totalGearsCount = 0;

	// Switch Inputs
	engineConfiguration->jssPin = Gpio::G11;
	engineConfiguration->opsPin = Gpio::G12;
	engineConfiguration->jssPinMode = PI_PULLDOWN;
	engineConfiguration->opsPinMode = PI_PULLDOWN;

	// ETB
	engineConfiguration->etb_use_two_wires = false;
	engineConfiguration->etbIo[0].directionPin1 = Gpio::D13;
	engineConfiguration->etbIo[0].controlPin = Gpio::D12;
	engineConfiguration->etbIo[0].disablePin = Gpio::C12;
	engineConfiguration->tps1_1AdcChannel = EFI_ADC_4;
	engineConfiguration->tps1_2AdcChannel = EFI_ADC_14;
	engineConfiguration->throttlePedalPositionAdcChannel = EFI_ADC_10;
	engineConfiguration->throttlePedalPositionSecondAdcChannel = EFI_ADC_12;

	// Temp Sensor Pullups
	// As CLT we use ETS because thats the important figure for us
	engineConfiguration->clt.config.bias_resistor = 1000; // ETS
	engineConfiguration->auxTempSensor1.config.bias_resistor = 1000; // ETS
	engineConfiguration->iat.config.bias_resistor = 1000; // IAT

	// Temp Sensors
	engineConfiguration->clt.adcChannel = EFI_ADC_17; // ETS PA1 MUX = 1
	engineConfiguration->auxTempSensor1.adcChannel = EFI_ADC_17; // ETS
	engineConfiguration->iat.adcChannel = EFI_ADC_18; // PA2 MUX = 1

	// Injection
	engineConfiguration->injectionMode = IM_SEQUENTIAL;
	engineConfiguration->injectionPins[0] = Gpio::G8;
	engineConfiguration->injectionPins[1] = Gpio::G7;

	// Ignition
	engineConfiguration->ignitionMode = IM_INDIVIDUAL_COILS;
	engineConfiguration->ignitionPins[0] = Gpio::E5;
	engineConfiguration->ignitionPins[1] = Gpio::C13;

	// Trigger
	engineConfiguration->triggerInputPins[0] = Gpio::B0;
	engineConfiguration->trigger.type = trigger_type_e::TT_TOOTHED_WHEEL_32_2;

	engineConfiguration->engineType = engine_type_e::HARLEY;

	// total 45 degree odd fire, split across two cylinders mostly for fun
	engineConfiguration->timing_offset_cylinder[0] = -HARLEY_V_TWIN / 2;
	engineConfiguration->timing_offset_cylinder[1] = +HARLEY_V_TWIN / 2;

	// work-around for https://github.com/rusefi/rusefi/issues/5894 todo: fix it!
	engineConfiguration->maximumIgnitionTiming = 90;
	engineConfiguration->minimumIgnitionTiming = -90;

	// PRGSEL
	config->prgselOutputPin = Gpio::D10;
}

void boardCustomInitHardware() {
	// Purge Solenoid PWM init
	startSimplePwmExt(&prgselPwm, 
				   "PRGSEL", 
				   &engine->scheduler, 
				   config->prgselOutputPin, 
				   &prgselPin, 
				   NAN, // Frequency
				   config->prgselPWMDuty / 100.0f
	);

	harleyDetectedGearSensor.Register();

	// Not strictly hardware, but does not fit for boardConfigOverrides()
	// or anywhere else.
	autotuneState.initializeStates();
	atIdleBaseState.initializeStates();

	o2SensorInitialization();
}

#if EFI_PROD_CODE
static virtual_timer_t hardResetTimer;

/**
 * Brings the MCU as close as possible to the state after a full power cycle, then resets.
 * Runs from a virtual timer callback (ISR context) so the TS OK response can go out first.
 *
 * A plain NVIC_SystemReset() keeps everything powered by VBAT/backup regulator alive.
 * A real power cycle on this board (no backup battery) loses all of that, so clear it here:
 * - backup domain (RTC, RTC backup registers incl. stepper position / ign counter)
 * - backup SRAM (boot counter, stored error/crash state)
 * - latched reset cause flags
 * - OpenBLT shared params flag, so the bootloader starts the firmware instead of staying resident
 */
static void boardHardResetNow(virtual_timer_t*, void*) {
	__disable_irq();

#if EFI_USE_OPENBLT
	SharedParamsInit();
	SharedParamsWriteByIndex(0, 0x00);
#endif

#if defined(STM32F7XX)
	// Unlock backup domain write access
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	(void)RCC->APB1ENR;
	PWR->CR1 |= PWR_CR1_DBP;
	while ((PWR->CR1 & PWR_CR1_DBP) == 0) {
	}

	// Backup SRAM is not cleared by a backup domain reset: wipe it explicitly
	RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN;
	(void)RCC->AHB1ENR;
	volatile uint32_t* bkpSram = reinterpret_cast<volatile uint32_t*>(BKPSRAM_BASE);
	for (size_t i = 0; i < 4096 / sizeof(uint32_t); i++) {
		bkpSram[i] = 0;
	}

	// Backup domain reset: RTC, LSE config and RTC backup registers
	RCC->BDCR |= RCC_BDCR_BDRST;
	RCC->BDCR &= ~RCC_BDCR_BDRST;

	// Clear latched reset cause flags
	RCC->CSR |= RCC_CSR_RMVF;
#endif

	// Make sure nothing written above (e.g. OpenBLT shared params in SRAM) is lost in D-cache
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
	if (SCB->CCR & SCB_CCR_DC_Msk) {
		SCB_CleanDCache();
	}
#endif
	__DSB();

	// System reset: core and all peripherals, all GPIOs back to reset state (hi-Z)
	NVIC_SystemReset();
}
#endif // EFI_PROD_CODE

static void boardScheduleHardReset() {
	if (!engine->rpmCalculator.isStopped()) {
		efiPrintf("Hard reset refused: engine is not stopped");
		return;
	}

#if EFI_PROD_CODE
	efiPrintf("Hard reset in 500 ms...");
	chibios_rt::CriticalSectionLocker csl;
	// Short delay so the TS OK response for this command is still sent
	chVTSetI(&hardResetTimer, TIME_MS2I(500), boardHardResetNow, nullptr);
#endif
}

void boardHandleTsCommand(uint16_t subsystem, uint16_t index) {
	switch(index) {
		case 0x02:
			setPrgselForce(false);
			break;
		case 0x03:
			setPrgselForce(true);
			break;
		case 0x08:
			autotuneState.toggleRunning();
			break;
		case 0x09:
			autotuneState.burningROM();
			break;
		case 0x0A:
			autotuneState.applyingToRAM();
			break;
		case 0x0B:
			autotuneState.prepareFetchData();
			autotuneState.autotuneFetchDataDone = true;
			break;
		case 0x0C:
			autotuneState.autotuneFetchDataDone = false;
			break;
		case 0x0D:
			autotuneState.toggleAutoApply();
			break;
		case 0x0E:
			autotuneState.resetApplyToRAMIndicator();
			break;
		case 0x0F:
			atIdleBaseState.toggleRunning();
			break;
		case 0x10:
			atIdleBaseState.burningROM();
			break;
		case 0x11:
			atIdleBaseState.applyingToRAM();
			break;
		case 0x12:
			atIdleBaseState.prepareFetchData();
			atIdleBaseState.atIdleBaseFetchDataDone = true;
			break;
		case 0x13:
			atIdleBaseState.atIdleBaseFetchDataDone = false;
			break;
		case 0x14:
			atIdleBaseState.toggleAutoApply();
			break;
		case 0x15:
			atIdleBaseState.resetApplyToRAMIndicator();
			break;
		case 0x17:
			boardScheduleHardReset();
			break;
		default:
			break;
	}
}

void boardCustomOnConfigurationChange(engine_configuration_s* previousConfiguration) {
	// Burn: restore values a TS write may have replaced, before the configuration goes to flash
	setDpecmHardwareInfo();

	boardSanitizeConfig();

	autotuneState.checkCyclicBufferSize();
	autotuneState.applyCellChangeResistancePreset();
	atIdleBaseState.applyCellChangeResistancePreset();

	if(!config->prgselInstalled || !config->prgselActive) {
		prgselPwm.setFrequency(NAN);
	} else {
		prgselPwm.setFrequency(config->prgselPWMFreq);
		prgselPwm.setSimplePwmDutyCycle(config->prgselPWMDuty / 100.0f);
	}
}