/*
 * HidFFB.cpp
 *
 *  Created on: 12.02.2020
 *      Author: Yannick
 *
 * ────── ODrive-FFB-Wheel fork ──────────────────────────────────────────────
 * Este arquivo é uma cópia forkada do OpenFFBoard upstream.
 *   Base: OpenFFBoard v1.17.0 (commit ac68fba)
 *   Localização upstream: Firmware/FFBoard/Src/HidFFB.cpp
 *
 * Modificações locais:
 *   - set_effect: fallback pra single-axis wheel quando axisMagnitudes[0]=0
 *     e axisMagnitudes[1]=0 simultaneamente (ex: directionX=0 polar). Usa
 *     cos(phaseX) ou 1.0 como default — força que jogos com direção north
 *     gerem torque no eixo X do volante.
 *
 * Pra atualizar contra upstream:
 *   diff Odrive-Wheel/src/HidFFB.cpp \
 *        OpenFFBoard-master/OpenFFBoard-master/Firmware/FFBoard/Src/HidFFB.cpp
 * ───────────────────────────────────────────────────────────────────────────
 */

#include <assert.h>
#include "HidFFB.h"
#include "flash_helpers.h"
#include "hid_device.h"
#include "cppmain.h"
#include <math.h>

namespace {
static int16_t gimxU8ToS16(uint8_t value) {
	if (value < 128) {
		++value;
	}
	return static_cast<int16_t>((static_cast<int32_t>(value) - 128) * 32767 / 127);
}

static int16_t gimxU16ToS16(uint16_t value) {
	int32_t signedValue = static_cast<int32_t>(value) - 32768;
	if (signedValue < 0) {
		++signedValue;
	}
	return static_cast<int16_t>(signedValue);
}

static int16_t gimxConditionCoefficient(uint8_t selector, bool highResolution, bool inverted) {
	static const uint8_t lowResolutionNumerators[8] = {1, 2, 3, 4, 6, 8, 12, 16};
	int32_t magnitude = highResolution
		? (32767 * (selector & 0x0f)) / 15
		: (32767 * lowResolutionNumerators[selector & 0x07]) / 16;
	return static_cast<int16_t>(inverted ? -magnitude : magnitude);
}

static int16_t gimxSpringCoefficient(int16_t coefficient, uint8_t gain) {
	// ODrive's default spring gain/scaler is 64/16. Normalize it here so a
	// Logitech coefficient keeps its intended slope in the shared effect engine.
	const float engineScale = ((static_cast<float>(gain) + 1.0f) / 256.0f) * 16.0f;
	if (engineScale <= 0.0f) {
		return 0;
	}
	float scaled = static_cast<float>(coefficient) / engineScale;
	if (scaled > 32767.0f) scaled = 32767.0f;
	if (scaled < -32767.0f) scaled = -32767.0f;
	return static_cast<int16_t>(scaled);
}
}


HidFFB::HidFFB(std::shared_ptr<EffectsCalculator> ec,uint8_t axisCount) : effects_calc(ec), effects(ec->effects),axisCount(axisCount)
{
	directionEnableMask = 1 << axisCount; // Direction enable bit is last bit after axis enable bits
	// Initialize reports
	blockLoad_report.effectBlockIndex = 1;
	blockLoad_report.ramPoolAvailable = (HID_MAX_EFFECTS-used_effects)*sizeof(FFB_Effect);
	blockLoad_report.loadStatus = 1;

	pool_report.ramPoolSize = HID_MAX_EFFECTS*sizeof(FFB_Effect);
	pool_report.maxSimultaneousEffects = HID_MAX_EFFECTS;
	pool_report.memoryManagement = 1;


	this->registerHidCallback();
}

HidFFB::~HidFFB() {
}


/**
 * Sets the mask where the direction enable bit is in the effect
 */
void HidFFB::setDirectionEnableMask(uint8_t mask){
	this->directionEnableMask = mask;
}

void HidFFB::updateSamplerate(float newSamplerate){
	effects_calc->updateSamplerate(newSamplerate);
}


bool HidFFB::getFfbActive(){
	return this->ffb_active;
}

bool HidFFB::HID_SendReport(uint8_t *report,uint16_t len){
	return tud_hid_report(0, report, len); // ID 0 skips ID field
}


/**
 * Sends a status report for a specific effect
 */
void HidFFB::sendStatusReport(uint8_t effect){
//	if(effect != 0){
//		this->reportFFBStatus.effectBlockIndex = effect;
//	}
	this->reportFFBStatus.status = HID_ACTUATOR_POWER;
	if(this->ffb_active){
		this->reportFFBStatus.status |= HID_ENABLE_ACTUATORS;
		this->reportFFBStatus.status |= HID_EFFECT_PLAYING;
	}else{
		this->reportFFBStatus.status |= HID_EFFECT_PAUSE;
	}
//	if(effect > 0 && effects[effect-1].state == 1)
//		this->reportFFBStatus.status |= HID_EFFECT_PLAYING;
	//printf("Status: %d\n",reportFFBStatus.status);
	HID_SendReport(reinterpret_cast<uint8_t*>(&this->reportFFBStatus), sizeof(reportFFB_status_t));
}


/**
 * Called when HID OUT data is received via USB
 */
void HidFFB::hidOut(uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize){
	fxUpdateEvent(); // use uint16_t for timer overflow handling if micros timer is used

	if (report_id == GIMX_LOGITECH_REPORT_ID) {
		processLogitechReport(buffer, bufsize);
		return;
	}

	// FFB Output Message
	const uint8_t* report = buffer;
	uint8_t event_idx = report_id - FFB_ID_OFFSET;

	// -------- Out Reports --------
	switch(event_idx)
	{

		case HID_ID_NEWEFREP: //add Effect Report. Feature
			new_effect((FFB_CreateNewEffect_Feature_Data_t*)(report));
			break;
		case HID_ID_EFFREP: // Set Effect
		{
			FFB_SetEffect_t setEffectRepBuf;
			memcpy(&setEffectRepBuf,report,std::min<uint16_t>(sizeof(FFB_SetEffect_t),bufsize)); // Copy report to buffer. only valid range if less axes are used
			set_effect(&setEffectRepBuf);
			break;
		}
		case HID_ID_CTRLREP: // Control report. 1=Enable Actuators, 2=Disable Actuators, 4=Stop All Effects, 8=Reset, 16=Pause, 32=Continue
			ffb_control(report[1]);
			//sendStatusReport(0);
			break;
		case HID_ID_GAINREP: // Set global gain
			set_gain(report[1]);
			break;
		case HID_ID_ENVREP: // Envelope
			set_envelope((FFB_SetEnvelope_Data_t *)report);
			break;
		case HID_ID_CONDREP: // Spring, Damper, Friction, Inertia
			set_condition((FFB_SetCondition_Data_t*)report);
			break;
		case HID_ID_PRIDREP: // Periodic
			set_periodic((FFB_SetPeriodic_Data_t*)report);
			break;
		case HID_ID_CONSTREP: // Constant
			set_constant_effect((FFB_SetConstantForce_Data_t*)report);
			break;
		case HID_ID_RAMPREP: // Ramp
			set_ramp((FFB_SetRamp_Data_t *)report);
			break;
		case HID_ID_CSTMREP: // Custom. pretty much never used
			//printf("Customrep");
			break;
		case HID_ID_SMPLREP: // Download sample
			//printf("Sampledl");
			break;
		case HID_ID_EFOPREP: //Effect operation
		{
			set_effect_operation((FFB_EffOp_Data_t*)report);
			break;
		}
		case HID_ID_BLKFRREP: // Free a block (PID Block Free Report)
		{
			// Bug 5: decrementa used_effects e atualiza ramPoolAvailable pra
			// evitar underflow do pool após várias criações/liberações num jogo.
			uint8_t eid = report[1];
			if (eid > 0 && eid <= HID_MAX_EFFECTS) {
				effects_calc->free_effect(eid - 1);
				if (used_effects > 0) {
					used_effects--;
				}
				blockLoad_report.ramPoolAvailable = (HID_MAX_EFFECTS - used_effects) * sizeof(FFB_Effect);
			}
			break;
		}

		default:
		{
			break;
		}
	}

}


/**
 * Called on HID feature GET events
 * Any reply is assigned to the return buffer
 *
 * Handles block load reports and pool status which are requested after a new effect has been created
 */
uint16_t HidFFB::hidGet(uint8_t report_id, hid_report_type_t report_type,uint8_t* buffer, uint16_t reqlen){
	// Feature gets go here

	uint8_t id = report_id - FFB_ID_OFFSET;

	switch(id){
	case HID_ID_STATE:
		// Bug 2: PID State Report (Report ID 2). Forza, ACC e vários utilitários
		// DirectInput chamam GetForceFeedbackState() antes de mandar efeitos —
		// se responder zero, entendem "unpowered" e desligam o FFB.
		buffer[0] = this->reportFFBStatus.status;
		return 1;
	case HID_ID_BLKLDREP:
		// Notice: first byte ID is not present in the reply buffer because it is handled by tinyusb internally!
		memcpy(buffer,&this->blockLoad_report,sizeof(FFB_BlockLoad_Feature_Data_t));
		return sizeof(FFB_BlockLoad_Feature_Data_t);
	case HID_ID_POOLREP:
		memcpy(buffer,&this->pool_report,sizeof(FFB_PIDPool_Feature_Data_t));
		return sizeof(FFB_PIDPool_Feature_Data_t);
	default:
		break;
	}
	return 0;
}

void HidFFB::start_FFB(){
#ifdef DEBUGLOG
	CommandHandler::logSerialDebug("FFB on");
#endif
	this->set_FFB(true);
}

void HidFFB::stop_FFB(){
#ifdef DEBUGLOG
	CommandHandler::logSerialDebug("FFB off");
#endif
	this->set_FFB(false);
}

void HidFFB::set_FFB(bool state)
{
	assert(effects_calc != nullptr);
	this->ffb_active = state;
	effects_calc->setActive(state);
}

void HidFFB::set_gain(uint8_t gain){
	assert(effects_calc != nullptr);
	effects_calc->setGain(gain);
}

void HidFFB::set_filters(FFB_Effect *effect){
	assert(effects_calc != nullptr);
	effects_calc->setFilters(effect);
}

void HidFFB::ffb_control(uint8_t cmd){

	if(cmd & 0x01){ // Enable Actuators
		start_FFB();
	}
	if(cmd & 0x02){ // Disable Actuators
		stop_FFB();
	}
	if(cmd & 0x04){ // Stop All Effects
		// Bug 6: precisa realmente PARAR os efeitos ativos (não só stop_FFB()).
		// Sem isso, ao resetar carro/respawn os efeitos voltam a tocar juntos
		// quando o jogo manda Continue.
		for(auto& effect : effects){
			effect.state = 0;
		}
	}
	if(cmd & 0x08){ // Reset
		stop_FFB();
		reset_ffb();
	}
	if(cmd & 0x10){ // Pause
		stop_FFB();
	}
	if(cmd & 0x20){ // Continue
		start_FFB();
	}
}


void HidFFB::set_constant_effect(FFB_SetConstantForce_Data_t* data){
	if(data->effectBlockIndex == 0 || data->effectBlockIndex > HID_MAX_EFFECTS){
		return;
	}
	cfUpdateEvent();
	FFB_Effect& effect_p = effects[data->effectBlockIndex-1];

	effect_p.magnitude = data->magnitude;
//	if(effect_p.state == 0){
//		effect_p.state = 1; // Force start effect
//	}
}

void HidFFB::new_effect(FFB_CreateNewEffect_Feature_Data_t* effect){
	// Allocates a new effect

	int32_t index = effects_calc->find_free_effect(effect->effectType); // next effect
	if(index == -1){
		blockLoad_report.loadStatus = 2;
#ifdef DEBUGLOG
		CommandHandler::logSerialDebug("Can't allocate a new effect");
#endif
		return;
	}
	FFB_Effect new_effect;
	new_effect.type = effect->effectType;

	this->effects_calc->logEffectType(effect->effectType,false);
#ifdef DEBUGLOG
	CommandHandler::logSerialDebug("New effect type:" + std::to_string(effect->effectType) + " idx: " + std::to_string(index-1));
#endif

	set_filters(&new_effect);

	effects[index] = std::move(new_effect);
	// Set block load report
	//reportFFBStatus.effectBlockIndex = index;
	blockLoad_report.effectBlockIndex = index+1;
	used_effects++;
	blockLoad_report.ramPoolAvailable = (HID_MAX_EFFECTS-used_effects)*sizeof(FFB_Effect);
	blockLoad_report.loadStatus = 1;
	sendStatusReport(index+1);
	

}

/**
 * Sets up an effect
 * If the direction enable bit is set then only 1 condition block is used
 */
void HidFFB::set_effect(FFB_SetEffect_t* effect){
	uint8_t index = effect->effectBlockIndex;
	if(index > HID_MAX_EFFECTS || index == 0)
		return;

	FFB_Effect* effect_p = &effects[index-1];

	if (effect_p->type != effect->effectType){
		effect_p->startTime = 0;
		set_filters(effect_p);
	}

	effect_p->gain = effect->gain;
	effect_p->type = effect->effectType;
	effect_p->samplePeriod = effect->samplePeriod;

	bool directionEnable = (effect->enableAxis & this->directionEnableMask);
	bool overridesCondition = false;

//	if(effect_p->useSingleCondition){ // Only allow turning single condition off in case it was overridden by sending multiple conditions previously
//		effect_p->useSingleCondition = directionEnable; // If direction is used only a single parameter block is allowed. Somehow this is still set while 2 conditions are sent...
//	}
	// Conditional effects usually do not use directions
	if(!effect_p->useSingleCondition && (effect->effectType == FFB_EFFECT_SPRING || effect->effectType == FFB_EFFECT_DAMPER || effect->effectType == FFB_EFFECT_INERTIA || effect->effectType == FFB_EFFECT_FRICTION))
	{
		if(effect_p->conditions[0].isActive()){
			effect_p->axisMagnitudes[0] = 1.0f;
			overridesCondition = true;
		}
		if(effect_p->conditions[1].isActive() || effect_p->useSingleCondition){
			effect_p->axisMagnitudes[1] = 1.0f;
			overridesCondition = true;
		}
	}


	if(!overridesCondition){
		float phaseX = M_PI*2.0 * (effect->directionX/36000.0f);

		effect_p->axisMagnitudes[0] = directionEnable ? sin(phaseX) : (effect->enableAxis & X_AXIS_ENABLE ? (effect->directionX - 18000.0f) / 18000.0f : 0); // Angular vector if dirEnable used otherwise linear or 0 if axis enabled
		effect_p->axisMagnitudes[1] = directionEnable ? -cos(phaseX) : (effect->enableAxis & Y_AXIS_ENABLE ? -(effect->directionY - 18000.0f) / 18000.0f : 0);

		// Single-axis wheel fix (Bug 1): em volantes de 1 eixo, se a direção
		// polar for Norte (0°) ou Sul (180°), sin(phaseX) = 0 e toda a força
		// vai pra axisMagnitudes[1] (eixo Y inexistente), zerando o torque.
		// Redireciona pro eixo 0 usando -cosf(phaseX) que preserva o sinal.
		// Em multi-axis a lógica antiga (só ambos zerados) segue valendo.
		if (axisCount == 1) {
			if (effect_p->axisMagnitudes[0] == 0.0f) {
				float fallback = directionEnable ? -cosf(phaseX) :
				                 (effect->enableAxis & Y_AXIS_ENABLE ? -(effect->directionY - 18000.0f) / 18000.0f : 1.0f);
				if (fallback == 0.0f) fallback = 1.0f;
				effect_p->axisMagnitudes[0] = fallback;
			}
		} else {
			if (effect_p->axisMagnitudes[0] == 0.0f && effect_p->axisMagnitudes[1] == 0.0f) {
				effect_p->axisMagnitudes[0] = 1.0f;
			}
		}
	}

#if MAX_AXIS == 3
	float phaseY = M_PI*2.0 * (effect->directionY/36000.0);
	effect_p->axisMagnitudes[3] = directionEnable ? sin(phaseY) : (effect->enableAxis & Z_AXIS_ENABLE ? (effect->directionZ - 18000.0f) / 18000.0f : 0);
#endif
	if(effect->duration == 0){ // Fix for games assuming 0 is infinite
		effect_p->duration = FFB_EFFECT_DURATION_INFINITE;
	}else{
		effect_p->duration = effect->duration;
	}
	effect_p->startDelay = effect->startDelay;
	if(!ffb_active)
		start_FFB();

	sendStatusReport(effect->effectBlockIndex);
	//CommandHandler::logSerialDebug("Setting Effect: " + std::to_string(effect->effectType) +  " at " + std::to_string(index) + "\n");
}

/**
	If the number of Condition report blocks is equal to the number of axes for the effect, then the first report
	block applies to the first axis, the second applies to the second axis, and so on. For example, a two-axis
	spring condition with CP Offset set to zero in both Condition report blocks would have the same effect as
	the joystick self-centering spring. When a condition is defined for each axis in this way, the effect must
	not be rotated.

	If there is a single Condition report block for an effect with more than one axis, then the direction along
	which the parameters of the Condition report block are in effect is determined by the direction parameters
	passed in the Direction field of the Effect report block. For example, a friction condition rotated 45
	degrees (in polar coordinates) would resist joystick motion in the northeast-southwest direction but would
	have no effect on joystick motion in the northwest-southeast direction.
 */
void HidFFB::set_condition(FFB_SetCondition_Data_t *cond){
	if(cond->effectBlockIndex == 0 || cond->effectBlockIndex > HID_MAX_EFFECTS){
		return;
	}
	// Bug 10: parameterBlockOffset empacota axis nos 4 bits inferiores + ordinal
	// nos 4 superiores. Sem mask, jogos da Codemasters (DiRT/F1/GRID) mandavam
	// valores >= 16 e a mola/damper caía num eixo inexistente.
	uint8_t axis = (axisCount > 0)
	    ? std::min<uint8_t>((uint8_t)(axisCount - 1), (uint8_t)(cond->parameterBlockOffset & 0x03))
	    : 0;

	FFB_Effect *effect = &effects[cond->effectBlockIndex - 1];
	effect->conditions[axis].cpOffset = cond->cpOffset;
	effect->conditions[axis].negativeCoefficient = cond->negativeCoefficient;
	effect->conditions[axis].positiveCoefficient = cond->positiveCoefficient;
	effect->conditions[axis].negativeSaturation = cond->negativeSaturation;
	effect->conditions[axis].positiveSaturation = cond->positiveSaturation;
	effect->conditions[axis].deadBand = cond->deadBand;

	// Bug 3 (entrada): promove sat=0 → 0x7FFF na chegada quando coeff != 0,
	// pra bater com a semântica DirectInput ("sem limite"). Complementa a
	// mesma promoção feita no clip do EffectsCalculator.
	if(effect->conditions[axis].positiveSaturation == 0 && effect->conditions[axis].positiveCoefficient != 0){
		effect->conditions[axis].positiveSaturation = 0x7FFF;
	}
	if(effect->conditions[axis].negativeSaturation == 0 && effect->conditions[axis].negativeCoefficient != 0){
		effect->conditions[axis].negativeSaturation = 0x7FFF;
	}

	if(axis>0 && axis < MAX_AXIS && effect->conditions[axis].isActive()){ // Workaround when direction enable is set but multiple conditions are defined... Resets direction and uses conditions again
		effect->useSingleCondition = false;
	}
	if((effect->conditions[axis].isActive() || (axis > 0 && effect->useSingleCondition))  && effect->axisMagnitudes[axis] == 0){
		effect->axisMagnitudes[axis] = 1.0;
	}

//	for(uint8_t i = 0;i<MAX_AXIS;i++){
//		effect->axisMagnitudes[i] = 1.0;
//	}
}

void HidFFB::set_effect_operation(FFB_EffOp_Data_t* report){
	if(report->effectBlockIndex == 0 || report->effectBlockIndex > HID_MAX_EFFECTS){
		return; // Invalid ID
	}
	// Start or stop effect
	uint8_t id = report->effectBlockIndex-1;
	if(report->state == 3){
		effects[id].state = 0; //Stop
#ifdef DEBUGLOG
		CommandHandler::logSerialDebug("Stop effect: " + std::to_string(id));
#endif

	}else{

		// 1 = start, 2 = start solo
		if(report->state == 2){
#ifdef DEBUGLOG
			CommandHandler::logSerialDebug("Start solo: " + std::to_string(id));
#endif
			for(FFB_Effect& effect : effects){
				effect.state = 0; // Stop all other effects
			}
		}
		if(effects[id].state != 1){
			set_filters(&effects[id]);
		}
#ifdef DEBUGLOG
		CommandHandler::logSerialDebug("Start effect: " + std::to_string(id));
#endif
		effects[id].startTime = HAL_GetTick() + effects[id].startDelay; // + effects[id].startDelay;
		effects[id].state = 1; //Start


	}
	//sendStatusReport(report[1]);
	this->effects_calc->logEffectState(effects[id].type,effects[id].state);
}


void HidFFB::set_envelope(FFB_SetEnvelope_Data_t *report){
	if(report->effectBlockIndex == 0 || report->effectBlockIndex > HID_MAX_EFFECTS){
		return;
	}
	FFB_Effect *effect = &effects[report->effectBlockIndex - 1];

	effect->attackLevel = report->attackLevel;
	effect->attackTime = report->attackTime;
	effect->fadeLevel = report->fadeLevel;
	effect->fadeTime = report->fadeTime;
	effect->useEnvelope = true;
}

void HidFFB::set_ramp(FFB_SetRamp_Data_t *report){
	if(report->effectBlockIndex == 0 || report->effectBlockIndex > HID_MAX_EFFECTS){
		return;
	}
	FFB_Effect *effect = &effects[report->effectBlockIndex - 1];
	effect->magnitude = 0x7fff; // Full magnitude for envelope calculation. This effect does not have a periodic report
	effect->startLevel = report->startLevel;
	effect->endLevel = report->endLevel;
}

void HidFFB::set_periodic(FFB_SetPeriodic_Data_t* report){
	if(report->effectBlockIndex == 0 || report->effectBlockIndex > HID_MAX_EFFECTS){
		return;
	}
	FFB_Effect* effect = &effects[report->effectBlockIndex-1];

	effect->period = clip<uint32_t,uint32_t>(report->period,1,0x7fff); // Period is never 0
	effect->magnitude = report->magnitude;
	effect->offset = report->offset;
	effect->phase = report->phase;
	//effect->counter = 0;
}



bool HidFFB::processLogitechReport(const uint8_t* buffer, uint16_t bufsize) {
	if (buffer == nullptr) {
		return false;
	}

	// TinyUSB normally strips the report ID before dispatch. Accept the ID in
	// the payload as well, so the bridge remains compatible with raw HID paths.
	const uint8_t* command = buffer;
	if (bufsize >= GIMX_LOGITECH_REPORT_SIZE + 1 &&
		command[0] == GIMX_LOGITECH_REPORT_ID) {
		++command;
		--bufsize;
	}
	if (bufsize < GIMX_LOGITECH_REPORT_SIZE) {
		return false;
	}

	const uint8_t commandType = command[0] & 0x0f;
	const uint8_t slotMask = (command[0] & 0xf0) >> 4;

	if (commandType == 0x00 || commandType == 0x01) { // Download / Download and play
		if (slotMask == 0 || (slotMask & (slotMask - 1)) != 0) {
			return false; // A downloaded effect belongs to exactly one slot.
		}
		uint8_t slot = 0;
		while (((slotMask >> slot) & 1u) == 0u && slot < GIMX_LOGITECH_SLOT_COUNT) {
			++slot;
		}
		if (slot >= GIMX_LOGITECH_SLOT_COUNT) {
			return false;
		}
		gimx_logitech_slots[slot].report.fill(0);
		memcpy(gimx_logitech_slots[slot].report.data(), command, GIMX_LOGITECH_REPORT_SIZE);
		gimx_logitech_slots[slot].downloaded = true;
		if (commandType == 0x01) {
			applyLogitechEffect(slot, true);
		}
		return true;
	}

	if (commandType == 0x02 || commandType == 0x03) { // Play / Stop
		for (uint8_t slot = 0; slot < GIMX_LOGITECH_SLOT_COUNT; ++slot) {
			if ((slotMask & (1u << slot)) == 0u) {
				continue;
			}
			if (commandType == 0x03) {
				applyLogitechEffect(slot, false);
			} else if (gimx_logitech_slots[slot].downloaded) {
				applyLogitechEffect(slot, true);
			}
		}
		return true;
	}
	if (commandType == 0x0c) { // Refresh keeps a persistent effect alive; do not restart its filters.
		return true;
	}

	// Extended commands and default-spring controls do not carry one of the
	// downloaded slot effects handled in this first bridge increment.
	return false;
}

void HidFFB::applyLogitechEffect(uint8_t slot, bool playing) {
	if (slot >= GIMX_LOGITECH_SLOT_COUNT) {
		return;
	}
	FFB_Effect& effect = effects[GIMX_LOGITECH_FIRST_EFFECT + slot];
	if (!playing) {
		if (effect.type != FFB_EFFECT_NONE) {
			effect.state = 0;
			effects_calc->logEffectState(effect.type, 0);
		}
		return;
	}
	if (!gimx_logitech_slots[slot].downloaded) {
		return;
	}

	const uint8_t* report = gimx_logitech_slots[slot].report.data();
	const uint8_t* parameters = report + 2;
	const uint8_t forceType = report[1];
	uint8_t effectType = FFB_EFFECT_NONE;
	int16_t forceLevel = 0;
	bool hasCondition = false;
	int16_t negativeCoefficient = 0;
	int16_t positiveCoefficient = 0;
	int16_t center = 0;
	uint16_t deadBand = 0;
	uint16_t saturation = 0x7fff;

	switch (forceType) {
	case 0x00: // Logitech constant
		effectType = FFB_EFFECT_CONSTANT;
		forceLevel = gimxU8ToS16(parameters[slot]);
		break;

	case 0x08: // Variable: the non-ramping form is a constant level.
	{
		uint8_t level = 0x80;
		uint8_t step = 0;
		if (slot == 0) {
			level = parameters[0];
			step = parameters[2];
		} else if (slot == 2) {
			level = parameters[1];
			step = parameters[3];
		} else {
			break;
		}
		if ((step & 0x0f) != 0 && (step >> 4) != 0) {
			break; // Timed variable-force ramps are handled in a later increment.
		}
		effectType = FFB_EFFECT_CONSTANT;
		forceLevel = gimxU8ToS16(level);
		break;
	}

	case 0x01: // Logitech spring, 3-bit selectors
	{
		effectType = FFB_EFFECT_SPRING;
		hasCondition = true;
		const uint8_t selectorLow = parameters[2] & 0x07;
		const uint8_t selectorHigh = (parameters[2] >> 4) & 0x07;
		negativeCoefficient = gimxConditionCoefficient(selectorLow, false, parameters[3] & 0x01);
		positiveCoefficient = gimxConditionCoefficient(selectorHigh, false, parameters[3] & 0x10);
		const uint8_t d1 = parameters[0];
		const uint8_t d2 = parameters[1];
		center = gimxU8ToS16(static_cast<uint8_t>((static_cast<uint16_t>(d1) + d2) / 2));
		deadBand = (d2 >= d1) ? static_cast<uint16_t>((static_cast<uint32_t>(d2 - d1) * 65535u) / 255u) : 0;
		saturation = static_cast<uint16_t>((static_cast<uint32_t>(parameters[4]) * 65535u) / 255u);
		break;
	}

	case 0x02: // Logitech damper, 3-bit selectors
	{
		effectType = FFB_EFFECT_DAMPER;
		hasCondition = true;
		negativeCoefficient = gimxConditionCoefficient(parameters[0] & 0x07, false, parameters[1] & 0x01);
		positiveCoefficient = gimxConditionCoefficient(parameters[2] & 0x07, false, parameters[3] & 0x01);
		break;
	}

	case 0x0b: // High-resolution spring (Driving Force Pro / G29)
	{
		effectType = FFB_EFFECT_SPRING;
		hasCondition = true;
		const uint8_t d1Low = (parameters[3] >> 1) & 0x07;
		const uint8_t d2Low = (parameters[3] >> 5) & 0x07;
		const uint16_t d1 = static_cast<uint16_t>((((static_cast<uint16_t>(parameters[0]) << 3) | d1Low) * 65535u) / 0x7ffu);
		const uint16_t d2 = static_cast<uint16_t>((((static_cast<uint16_t>(parameters[1]) << 3) | d2Low) * 65535u) / 0x7ffu);
		const uint8_t selectorLow = parameters[2] & 0x0f;
		const uint8_t selectorHigh = (parameters[2] >> 4) & 0x0f;
		negativeCoefficient = gimxConditionCoefficient(selectorLow, true, parameters[3] & 0x01);
		positiveCoefficient = gimxConditionCoefficient(selectorHigh, true, parameters[3] & 0x10);
		center = gimxU16ToS16(static_cast<uint16_t>((static_cast<uint32_t>(d1) + d2) / 2u));
		deadBand = (d2 >= d1) ? static_cast<uint16_t>(d2 - d1) : 0;
		saturation = static_cast<uint16_t>((static_cast<uint32_t>(parameters[4]) * 65535u) / 255u);
		break;
	}

	case 0x0c: // High-resolution damper
	{
		effectType = FFB_EFFECT_DAMPER;
		hasCondition = true;
		negativeCoefficient = gimxConditionCoefficient(parameters[0] & 0x0f, true, parameters[1] & 0x01);
		positiveCoefficient = gimxConditionCoefficient(parameters[2] & 0x0f, true, parameters[3] & 0x01);
		break;
	}

	default:
		break;
	}

	if (effectType == FFB_EFFECT_NONE) {
		if (effect.type != FFB_EFFECT_NONE) {
			effect.state = 0;
			effects_calc->logEffectState(effect.type, 0);
		}
		effect.type = FFB_EFFECT_NONE;
		return;
	}

	const uint8_t previousType = effect.type;
	const bool wasPlaying = effect.state != 0;
	if (wasPlaying && previousType != effectType) {
		effects_calc->logEffectState(previousType, 0);
	}
	if (previousType != FFB_EFFECT_NONE && previousType != effectType) {
		effects_calc->logEffectType(previousType, true);
	}
	effect = FFB_Effect{};
	effect.type = effectType;
	effect.state = 1;
	effect.duration = FFB_EFFECT_DURATION_INFINITE;
	effect.gain = 255;
	effect.axisMagnitudes[0] = 1.0f;
	effect.magnitude = forceLevel;

	if (hasCondition) {
		if (effectType == FFB_EFFECT_SPRING) {
			negativeCoefficient = gimxSpringCoefficient(negativeCoefficient, effects_calc->getGainStruct().spring);
			positiveCoefficient = gimxSpringCoefficient(positiveCoefficient, effects_calc->getGainStruct().spring);
		}
		effect.conditions[0].cpOffset = center;
		effect.conditions[0].negativeCoefficient = negativeCoefficient;
		effect.conditions[0].positiveCoefficient = positiveCoefficient;
		effect.conditions[0].negativeSaturation = saturation;
		effect.conditions[0].positiveSaturation = saturation;
		effect.conditions[0].deadBand = deadBand;
	}
	set_filters(&effect);
	effect.startTime = HAL_GetTick();
	set_FFB(true);
	if (previousType != effectType) {
		effects_calc->logEffectType(effectType, false);
	}
	effects_calc->logEffectState(effectType, 1);
}

void HidFFB::reset_ffb(){
	for(uint8_t i=0;i<effects.size();i++){
		effects_calc->free_effect(i);
	}
	this->reportFFBStatus.status = (HID_ACTUATOR_POWER) | (HID_ENABLE_ACTUATORS);
	// Bônus (par com Bug 5): reset limpo do contador e do pool disponível.
	used_effects = 0;
	blockLoad_report.ramPoolAvailable = HID_MAX_EFFECTS * sizeof(FFB_Effect);
	for (auto& slot : gimx_logitech_slots) {
		slot.downloaded = false;
		slot.report.fill(0);
	}
}
