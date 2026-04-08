//
// Created by Jon Lidgard on 05/05/2025.
//

#ifndef WM8960_H
#define WM8960_H

#include <stdint.h>
#include "esp_err.h"
#include "wm8960_defs.h"
#include "driver/i2c_master.h"

#define WM8960_REGISTER_COUNT 224

typedef struct wm8960 {
    i2c_device_config_t i2c_config;
    i2c_master_dev_handle_t handle;
    uint16_t register_local_copy[WM8960_REGISTER_COUNT]; // Updated to correct size
    // Other members...
} wm8960_t;

esp_err_t wm8960_init(wm8960_t *wm8960);

esp_err_t wm8960_register_write(wm8960_t *wm8960, uint8_t reg, uint16_t value);

esp_err_t wm8960_enableVREF(wm8960_t *wm8960);
esp_err_t wm8960_disableVREF(wm8960_t *wm8960);
esp_err_t wm8960_reset(wm8960_t *wm8960);

esp_err_t wm8960_enableAINL(wm8960_t *wm8960);
esp_err_t wm8960_disableAINL(wm8960_t *wm8960);
esp_err_t wm8960_enableAINR(wm8960_t *wm8960);
esp_err_t wm8960_disableAINR(wm8960_t *wm8960);

esp_err_t wm8960_enableLMIC(wm8960_t *wm8960);
esp_err_t wm8960_disableLMIC(wm8960_t *wm8960);
esp_err_t wm8960_enableRMIC(wm8960_t *wm8960);
esp_err_t wm8960_disableRMIC(wm8960_t *wm8960);

esp_err_t wm8960_enableLMICBOOST(wm8960_t *wm8960);
esp_err_t wm8960_disbleLMICBOOST(wm8960_t *wm8960);
esp_err_t wm8960_enableRMICBOOST(wm8960_t *wm8960);
esp_err_t wm8960_disableRMICBOOST(wm8960_t *wm8960);

esp_err_t wm8960_pgaLeftNonInvSignalSelect(wm8960_t *wm8960, uint8_t signal);
esp_err_t wm8960_pgaRightNonInvSignalSelect(wm8960_t *wm8960, uint8_t signal);

esp_err_t wm8960_connectLMN1(wm8960_t *wm8960);
esp_err_t wm8960_disconnectLMN1(wm8960_t *wm8960);
esp_err_t wm8960_connectRMN1(wm8960_t *wm8960);
esp_err_t wm8960_disconnectRMN1(wm8960_t *wm8960);

esp_err_t wm8960_connectLMIC2B(wm8960_t *wm8960);
esp_err_t wm8960_disconnectLMIC2B(wm8960_t *wm8960);
esp_err_t wm8960_connectRMIC2B(wm8960_t *wm8960);
esp_err_t wm8960_disconnectRMIC2B(wm8960_t *wm8960);

esp_err_t wm8960_setLINVOL(wm8960_t *wm8960, uint8_t volume);
esp_err_t wm8960_setLINVOLDB(wm8960_t *wm8960, float dB);
esp_err_t wm8960_setRINVOL(wm8960_t *wm8960, uint8_t volume);
esp_err_t wm8960_setRINVOLDB(wm8960_t *wm8960, float dB);

esp_err_t wm8960_enablePgaZeroCross(wm8960_t *wm8960);
esp_err_t wm8960_disablePgaZeroCross(wm8960_t *wm8960);

esp_err_t wm8960_enableLINMUTE(wm8960_t *wm8960);
esp_err_t wm8960_disableLINMUTE(wm8960_t *wm8960);
esp_err_t wm8960_enableRINMUTE(wm8960_t *wm8960);
esp_err_t wm8960_disableRINMUTE(wm8960_t *wm8960);

esp_err_t wm8960_pgaLeftIPVUSet(wm8960_t *wm8960);
esp_err_t wm8960_pgaRightIPVUSet(wm8960_t *wm8960);

esp_err_t wm8960_setLMICBOOST(wm8960_t *wm8960, int8_t boost_gain);
esp_err_t wm8960_setRMICBOOST(wm8960_t *wm8960, int8_t boost_gain);

esp_err_t wm8960_setLIN3BOOST(wm8960_t *wm8960, int8_t boost_gain);
esp_err_t wm8960_setLIN2BOOST(wm8960_t *wm8960, int8_t boost_gain);
esp_err_t wm8960_setRIN3BOOST(wm8960_t *wm8960, int8_t boost_gain);
esp_err_t wm8960_setRIN2BOOST(wm8960_t *wm8960, int8_t boost_gain);

esp_err_t wm8960_enableMicBias(wm8960_t *wm8960);
esp_err_t wm8960_disableMicBias(wm8960_t *wm8960);
esp_err_t wm8960_setMicBiasVoltage(wm8960_t *wm8960, bool voltage);

esp_err_t wm8960_enableAdcLeft(wm8960_t *wm8960);
esp_err_t wm8960_disableAdcLeft(wm8960_t *wm8960);
esp_err_t wm8960_enableAdcRight(wm8960_t *wm8960);
esp_err_t wm8960_disableAdcRight(wm8960_t *wm8960);

esp_err_t wm8960_setAdcLeftDigitalVolume(wm8960_t *wm8960, uint8_t volume);
esp_err_t wm8960_setAdcRightDigitalVolume(wm8960_t *wm8960, uint8_t volume);
esp_err_t wm8960_adcLeftADCVUSet(wm8960_t *wm8960);
esp_err_t wm8960_adcRightADCVUSet(wm8960_t *wm8960);
esp_err_t wm8960_setAdcLeftDigitalVolumeDB(wm8960_t *wm8960, float dB);
esp_err_t wm8960_setAdcRightDigitalVolumeDB(wm8960_t *wm8960, float dB);
esp_err_t wm8960_enableADC(wm8960_t *wm8960);
esp_err_t wm8960_disableADC(wm8960_t *wm8960);
esp_err_t wm8960_setAlcTarget(wm8960_t *wm8960, uint8_t target);
esp_err_t wm8960_enableAlcTarget(wm8960_t *wm8960);
esp_err_t wm8960_setAlcDecay(wm8960_t *wm8960, uint8_t decay);
esp_err_t wm8960_setAlcAttack(wm8960_t *wm8960, uint8_t attack);
esp_err_t wm8960_setAlcMaxGain(wm8960_t *wm8960, uint8_t maxGain);
esp_err_t wm8960_setAlcMinGain(wm8960_t *wm8960, uint8_t minGain);
esp_err_t wm8960_setAlcHold(wm8960_t *wm8960, uint8_t hold);
esp_err_t wm8960_enablePeakLimiter(wm8960_t *wm8960);
esp_err_t wm8960_disablePeakLimiter(wm8960_t *wm8960);

esp_err_t wm8960_enableNoiseGate(wm8960_t *wm8960);
esp_err_t wm8960_disableNoiseGate(wm8960_t *wm8960);
esp_err_t wm8960_setNoiseGateThreshold(wm8960_t *wm8960, int8_t threshold);

esp_err_t wm8960_enableDacLeft(wm8960_t *wm8960);
esp_err_t wm8960_disableDacLeft(wm8960_t *wm8960);
esp_err_t wm8960_enableDacRight(wm8960_t *wm8960);
esp_err_t wm8960_disableDacRight(wm8960_t *wm8960);
esp_err_t wm8960_setDacLeftDigitalVolume(wm8960_t *wm8960, int8_t volume);
esp_err_t wm8960_setDacRightDigitalVolume(wm8960_t *wm8960, int8_t volume);

esp_err_t wm8960_dacLeftDACVUSet(wm8960_t *wm8960);
esp_err_t wm8960_dacRightDACVUSet(wm8960_t *wm8960);
esp_err_t wm8960_setDacLeftDigitalVolumeDB(wm8960_t *wm8960, float dB);
esp_err_t wm8960_setDacRightDigitalVolumeDB(wm8960_t *wm8960, float dB);
esp_err_t wm8960_enableDacMute(wm8960_t *wm8960);
esp_err_t wm8960_disableDacMute(wm8960_t *wm8960);
esp_err_t wm8960_enable3d(wm8960_t *wm8960);
esp_err_t wm8960_disable3d(wm8960_t *wm8960);
esp_err_t wm8960_set3dDepth(wm8960_t *wm8960, int8_t depth);
esp_err_t wm8960_enableDac6dbAttenuation(wm8960_t *wm8960);
esp_err_t wm8960_disableDac6dbAttentuation(wm8960_t *wm8960);

esp_err_t wm8960_setDacMonoMix(wm8960_t *wm8960, bool setting);


esp_err_t wm8960_enableLOMIX(wm8960_t *wm8960);
esp_err_t wm8960_disableLOMIX(wm8960_t *wm8960);
esp_err_t wm8960_enableROMIX(wm8960_t *wm8960);
esp_err_t wm8960_disableROMIX(wm8960_t *wm8960);
esp_err_t wm8960_enableOUT3MIX(wm8960_t *wm8960);
esp_err_t wm8960_disableOUT3MIX(wm8960_t *wm8960);
esp_err_t wm8960_enableLI2LO(wm8960_t *wm8960);
esp_err_t wm8960_disableLI2LO(wm8960_t *wm8960);
esp_err_t wm8960_setLI2LOVOL(wm8960_t *wm8960, int8_t volume);
esp_err_t wm8960_enableLB2LO(wm8960_t *wm8960);
esp_err_t wm8960_disableLB2LO(wm8960_t *wm8960);
esp_err_t wm8960_setLB2LOVOL(wm8960_t *wm8960, int8_t volume);
esp_err_t wm8960_enableLD2LO(wm8960_t *wm8960);
esp_err_t wm8960_disableLD2LO(wm8960_t *wm8960);
esp_err_t wm8960_enableRI2RO(wm8960_t *wm8960);
esp_err_t wm8960_disableRI2RO(wm8960_t *wm8960);
esp_err_t wm8960_setRI2ROVOL(wm8960_t *wm8960, int8_t volume);
esp_err_t wm8960_enableRB2RO(wm8960_t *wm8960);
esp_err_t wm8960_disableRB2RO(wm8960_t *wm8960);
esp_err_t wm8960_setRB2ROVOL(wm8960_t *wm8960, int8_t volume);
esp_err_t wm8960_enableRD2RO(wm8960_t *wm8960);
esp_err_t wm8960_disableRD2RO(wm8960_t *wm8960);
esp_err_t wm8960_enableLI2MO(wm8960_t *wm8960);
esp_err_t wm8960_disableLI2MO(wm8960_t *wm8960);
esp_err_t wm8960_enableRI2MO(wm8960_t *wm8960);
esp_err_t wm8960_disableRI2MO(wm8960_t *wm8960);
esp_err_t wm8960_enableVMID(wm8960_t *wm8960);
esp_err_t wm8960_disableVMID(wm8960_t *wm8960);
esp_err_t wm8960_setVMID(wm8960_t *wm8960, int8_t setting);

esp_err_t wm8960_enableHeadphones(wm8960_t *wm8960);
esp_err_t wm8960_disableHeadphones(wm8960_t *wm8960);
esp_err_t wm8960_enableRightHeadphone(wm8960_t *wm8960);
esp_err_t wm8960_disableRightHeadphone(wm8960_t *wm8960);
esp_err_t wm8960_enableLeftHeadphone(wm8960_t *wm8960);
esp_err_t wm8960_disableLeftHeadphone(wm8960_t *wm8960);
esp_err_t wm8960_enableHeadphoneStandby(wm8960_t *wm8960);
esp_err_t wm8960_disableHeadphoneStandby(wm8960_t *wm8960);
esp_err_t wm8960_setHeadphoneVolume(wm8960_t *wm8960, uint8_t volume);
esp_err_t wm8960_enableHeadphoneJackDetect(wm8960_t *wm8960);
esp_err_t wm8960_disableHeadphoneJackDetect(wm8960_t *wm8960);
esp_err_t wm8960_setHeadphoneJackDetectInput(wm8960_t *wm8960, int8_t setting);
esp_err_t wm8960_setHeadphoneVolumeDB(wm8960_t *wm8960, float dB);
esp_err_t wm8960_enableHeadphoneZeroCross(wm8960_t *wm8960);
esp_err_t wm8960_disableHeadphoneZeroCross(wm8960_t *wm8960);

esp_err_t wm8960_enableSpeakers(wm8960_t *wm8960);
esp_err_t wm8960_disableSpeakers(wm8960_t *wm8960);
esp_err_t wm8960_enableRightSpeaker(wm8960_t *wm8960);
esp_err_t wm8960_disableRightSpeaker(wm8960_t *wm8960);
esp_err_t wm8960_enableLeftSpeaker(wm8960_t *wm8960);
esp_err_t wm8960_disableLeftSpeaker(wm8960_t *wm8960);
esp_err_t wm8960_setSpeakerVolume(wm8960_t *wm8960, uint8_t volume);
esp_err_t wm8960_setSpeakerVolumeDB(wm8960_t *wm8960, float dB);
esp_err_t wm8960_enableSpeakerZeroCross(wm8960_t *wm8960);
esp_err_t wm8960_disableSpeakerZeroCross(wm8960_t *wm8960);
esp_err_t wm8960_setSpeakerDcGain(wm8960_t *wm8960, uint8_t gain);
esp_err_t wm8960_setSpeakerAcGain(wm8960_t *wm8960, uint8_t gain);

esp_err_t wm8960_enableLoopBack(wm8960_t *wm8960);
esp_err_t wm8960_disableLoopBack(wm8960_t *wm8960);
esp_err_t wm8960_enablePLL(wm8960_t *wm8960);
esp_err_t wm8960_disablePLL(wm8960_t *wm8960);
esp_err_t wm8960_setPLLPRESCALE(wm8960_t *wm8960, bool div);
esp_err_t wm8960_setPLLN(wm8960_t *wm8960, uint8_t n);
esp_err_t wm8960_setPLLK(wm8960_t *wm8960, uint8_t one, uint8_t two, uint8_t three);
esp_err_t wm8960_setSMD(wm8960_t *wm8960, bool mode);
esp_err_t wm8960_setCLKSEL(wm8960_t *wm8960, bool sel);
esp_err_t wm8960_setSYSCLKDIV(wm8960_t *wm8960, uint8_t div);
esp_err_t wm8960_setADCDIV(wm8960_t *wm8960, uint8_t div);
esp_err_t wm8960_setDACDIV(wm8960_t *wm8960, uint8_t div);
esp_err_t wm8960_setBCLKDIV(wm8960_t *wm8960, uint8_t div);
esp_err_t wm8960_setDCLKDIV(wm8960_t *wm8960, uint8_t div);
esp_err_t wm8960_setALRCGPIO(wm8960_t *wm8960);

esp_err_t wm8960_enableMasterMode(wm8960_t *wm8960);
esp_err_t wm8960_enablePeripheralMode(wm8960_t *wm8960);
esp_err_t wm8960_setWL(wm8960_t *wm8960, uint8_t word_length);
esp_err_t wm8960_setLRP(wm8960_t *wm8960, bool polarity);
esp_err_t wm8960_setALRSWAP(wm8960_t *wm8960, bool swap);
esp_err_t wm8960_setVROI(wm8960_t *wm8960, bool setting);
esp_err_t wm8960_setVSEL(wm8960_t *wm8960, uint8_t setting);

uint8_t wm8960_convertDBtoSetting(float dB, float offset, float stepSize, float minDB, float maxDB);

#endif //WM8960_H
