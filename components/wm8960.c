//
// Created by Jon Lidgard on 05/05/2025.
//

#include "string.h"
#include "math.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "wm8960.h"

#define I2C_MASTER_TIMEOUT_MS       1000


esp_err_t wm8960_register_write(wm8960_t *wm8960, uint8_t reg, uint16_t value);
esp_err_t wm8960_register_write_bit(wm8960_t *wm8960, uint8_t register_address, uint8_t bit_number, bool bit_value);
esp_err_t wm8960_register_write_multi_bits(wm8960_t *wm8960, uint8_t register_address, uint8_t setting_msb_num, uint8_t setting_lsb_num, uint8_t setting);
uint8_t convertDBtoSetting(float dB, float offset, float step_size, float min_DB, float max_DB);


// writeRegister(uint8_t reg, uint16_t value)
// General-purpose write to a register
// Returns 1 if successful, 0 if something failed (I2C error)
// The WM8960 has 9 bit registers.
// To write a register, we must do the following
// Send 3 bytes
// Byte0 = device address + read/write bit
// Control_byte_1 = register-to-write address (7-bits) plus 9th bit of data
// Control_byte_2 = remaining 8 bits of register data
esp_err_t wm8960_register_write(wm8960_t *wm8960, uint8_t reg, uint16_t value)
{
  // Shift reg over one spot to make room for the 9th bit of register data
  uint8_t control_byte_1 = (reg << 1);

  // Shift value so only 9th bit is still there, plug it into 0th bit of
  // control_byte_1
  control_byte_1 |= (value >> 8);

  uint8_t control_byte_2 = (uint8_t)(value & 0xFF);

  uint8_t write_buf[2] = {control_byte_1, control_byte_2};
  return i2c_master_transmit(wm8960->handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

// writeRegisterBit
// Writes a 0 or 1 to the desired bit in the desired register
esp_err_t wm8960_register_write_bit(wm8960_t *wm8960, uint8_t register_address, uint8_t bit_number, bool bit_value)
{
  // Get the local copy of the register
  uint16_t reg_value = wm8960->register_local_copy[register_address];

  if(bit_value == 1)
  {
    reg_value |= (1<<bit_number); // Set only the bit we want
  }
  else {
    reg_value &= ~(1<<bit_number); // Clear only the bit we want
  }

  // Write modified value to device
  // If successful, update local copy
  if (wm8960_register_write(wm8960, register_address, reg_value) == ESP_OK)
  {
    wm8960->register_local_copy[register_address] = reg_value;
    return ESP_OK;
  }
  return ESP_FAIL;
}


// writeRegisterMultiBits
// This function writes data into the desired bits within the desired register
// Some settings require more than just flipping a single bit within a register.
// For these settings use this more advanced register write helper function.
//
// For example, to change the LIN2BOOST setting to +6dB,
// I need to write a setting of 7 (aka +6dB) to the bits [3:1] in the
// WM8960_REG_INPUT_BOOST_MIXER_1 register. Like so...
// _writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_1, 3, 1, 7);
esp_err_t wm8960_register_write_multi_bits(wm8960_t *wm8960, uint8_t register_address, uint8_t setting_msb_num, uint8_t setting_lsb_num, uint8_t setting)
{
  uint8_t num_of_bits = (setting_msb_num - setting_lsb_num) + 1;

  // Get the local copy of the register
  uint16_t reg_value = wm8960->register_local_copy[register_address];

  for(int i = 0 ; i < num_of_bits ; i++)
  {
    reg_value &= ~(1 << (setting_lsb_num + i)); // Clear bits we care about
  }

  // Shift and set the bits from in incoming desired setting value
  reg_value |= (setting << setting_lsb_num);

  // Write modified value to device
  // If successful, update local copy
  if (wm8960_register_write(wm8960, register_address, reg_value) == ESP_OK)
  {
    wm8960->register_local_copy[register_address] = reg_value;
    return ESP_OK;
  }
  return ESP_FAIL;
}

// reset
// Use this to reset all registers to their default state
// Note, this can also be done by cycling power to the device
// Returns 1 if successful, 0 if something failed (I2C error)
esp_err_t wm8960_reset(wm8960_t *wm8960)
{
  // Update our local copy of the registers to reflect the reset
  memcpy(wm8960->register_local_copy, WM8960_REGISTER_DEFAULTS, sizeof(WM8960_REGISTER_DEFAULTS));
  // Doesn't matter which bit we flip, writing anything will cause the reset
  esp_err_t ret = wm8960_register_write_bit(wm8960, WM8960_REG_RESET, 7, 1);
  // Update our local copy of the reset register to reflect the reset
  wm8960->register_local_copy[WM8960_REG_RESET] =WM8960_REGISTER_DEFAULTS[WM8960_REG_RESET];
  return ret;
}


// enableVREF
// Necessary for all other functions of the CODEC
// VREF is a single bit we can flip in Register 25 (19h), WM8960_REG_PWR_MGMT_1
// VREF is bit 6, 0 = power down, 1 = power up
// Returns 1 if successful, 0 if something failed (I2C error)
esp_err_t wm8960_enableVREF(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_1, 6, 1);
}


// disableVREF
// Use this to save power
// VREF is a single bit we can flip in Register 25 (19h), WM8960_REG_PWR_MGMT_1
// VREF is bit 6, 0 = power down, 1 = power up
// Returns 1 if successful, 0 if something failed (I2C error)
esp_err_t wm8960_disableVREF(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_1, 6, 0);
}

esp_err_t wm8960_enableAINL(wm8960_t *wm8960)
{ 
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_1, 5, 1);
}

esp_err_t wm8960_disableAINL(wm8960_t *wm8960)
{ 
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_1, 5, 0);
}

esp_err_t wm8960_enableAINR(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_1, 4, 1);
}

esp_err_t wm8960_disableAINR(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_1, 4, 0);
}

esp_err_t wm8960_enableLMIC(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_3, 5, 1);
}

esp_err_t wm8960_disableLMIC(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_3, 5, 0);
}

esp_err_t wm8960_enableRMIC(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_3, 4, 1);
}

esp_err_t wm8960_disableRMIC(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_3, 4, 0);
}

esp_err_t wm8960_enableLMICBOOST(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_3, 5, 1);
}

esp_err_t wm8960_disableLMICBOOST(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_3, 5, 0);
}

esp_err_t wm8960_enableRMICBOOST(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_3, 4, 1);
}

esp_err_t wm8960_disableRMICBOOST(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_3, 4, 0);
}

// PGA input signal select
// Each PGA (left and right) has a switch on its non-inverting input.
// On PGA_LEFT:
// 	*You can select between VMID, LINPUT2 or LINPUT3
// 	*Note, the inverting input of PGA_LEFT is perminantly connected to LINPUT1
// On PGA_RIGHT:
//	*You can select between VMIN, RINPUT2 or RINPUT3
// 	*Note, the inverting input of PGA_RIGHT is perminantly connected to RINPUT1

// 3 options: WM8960_PGAL_LINPUT2, WM8960_PGAL_LINPUT3, WM8960_PGAL_VMID
esp_err_t wm8960_pgaLeftNonInvSignalSelect(wm8960_t *wm8960, uint8_t signal)
{
  // Clear LMP2 and LMP3
  // Necessary because the previous setting could have either set,
  // And we don't want to confuse the codec.
  // Only 1 input can be selected.

  // LMP3
  esp_err_t result1 = wm8960_register_write_bit(wm8960, WM8960_REG_ADCL_SIGNAL_PATH, 7, 0);

  // LMP2
  esp_err_t result2 = wm8960_register_write_bit(wm8960, WM8960_REG_ADCL_SIGNAL_PATH, 6, 0);
  esp_err_t result3 = ESP_FAIL;

  if(signal == WM8960_PGAL_LINPUT2)
  {
    // LMP2
    result3 = wm8960_register_write_bit(wm8960, WM8960_REG_ADCL_SIGNAL_PATH, 6, 1); 
  }
  else if(signal == WM8960_PGAL_LINPUT3)
  {
    // LMP3
    result3 = wm8960_register_write_bit(wm8960, WM8960_REG_ADCL_SIGNAL_PATH, 7, 1); 
  }
  else if(signal == WM8960_PGAL_VMID)
  {
    // Don't set any bits. When both LMP2 and LMP3 are cleared, then the signal 
    // is set to VMID
  }
  if (result1  == ESP_OK && result2  == ESP_OK && result3 == ESP_OK) {
    return ESP_OK;
   } else {
     return ESP_FAIL;
   }
}

 // 3 options: WM8960_PGAR_RINPUT2, WM8960_PGAR_RINPUT3, WM8960_PGAR_VMID
esp_err_t wm8960_pgaRightNonInvSignalSelect(wm8960_t *wm8960, uint8_t signal)
{
  // Clear RMP2 and RMP3
  // Necessary because the previous setting could have either set,
  // And we don't want to confuse the codec.
  // Only 1 input can be selected.

  // RMP3
  esp_err_t result1 = wm8960_register_write_bit(wm8960, WM8960_REG_ADCR_SIGNAL_PATH, 7, 0);

  // RMP2
  esp_err_t result2 = wm8960_register_write_bit(wm8960, WM8960_REG_ADCR_SIGNAL_PATH, 6, 0);
  esp_err_t result3 = ESP_FAIL;

  if(signal == WM8960_PGAR_RINPUT2)
  {
    // RMP2
    result3 = wm8960_register_write_bit(wm8960, WM8960_REG_ADCR_SIGNAL_PATH, 6, 1); 
  }
  else if(signal == WM8960_PGAR_RINPUT3)
  {
    // RMP3
    result3 = wm8960_register_write_bit(wm8960, WM8960_REG_ADCR_SIGNAL_PATH, 7, 1); 
  }
  else if(signal == WM8960_PGAR_VMID)
  {
    // Don't set any bits. When both RMP2 and RMP3 are cleared, then the signal 
    // is set to VMID
  }
  return (result1  == ESP_OK && result2  == ESP_OK && result3 == ESP_OK ) ? ESP_OK : ESP_FAIL;
}

// Connection from each INPUT1 to the inverting input of its PGA
esp_err_t wm8960_connectLMN1(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_ADCL_SIGNAL_PATH, 8, 1);
}

// Disconnect LINPUT1 to inverting input of Left Input PGA
esp_err_t wm8960_disconnectLMN1(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_ADCL_SIGNAL_PATH, 8, 0);
}

// Connect RINPUT1 from inverting input of Right Input PGA
esp_err_t wm8960_connectRMN1(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_ADCR_SIGNAL_PATH, 8, 1);
}

// Disconnect RINPUT1 to inverting input of Right Input PGA
esp_err_t wm8960_disconnectRMN1(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_ADCR_SIGNAL_PATH, 8, 0);
}

// Connections from output of PGAs to downstream "boost mixers".

// Connect Left Input PGA to Left Input Boost mixer
esp_err_t wm8960_connectLMIC2B(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_ADCL_SIGNAL_PATH, 3, 1);
}

// Disconnect Left Input PGA to Left Input Boost mixer
esp_err_t wm8960_disconnectLMIC2B(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_ADCL_SIGNAL_PATH, 3, 0);
}

// Connect Right Input PGA to Right Input Boost mixer
esp_err_t wm8960_connectRMIC2B(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_ADCR_SIGNAL_PATH, 3, 1);
}

// Disconnect Right Input PGA to Right Input Boost mixer
esp_err_t wm8960_disconnectRMIC2B(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_ADCR_SIGNAL_PATH, 3, 0);
}

// 0-63, (0 = -17.25dB) <<-- 0.75dB steps -->> (63 = +30dB)
esp_err_t wm8960_setLINVOL(wm8960_t *wm8960, uint8_t volume)
{
  if(volume > 63) volume = 63; // Limit incoming values max
  esp_err_t result1 = wm8960_register_write_multi_bits(wm8960, WM8960_REG_LEFT_INPUT_VOLUME,5,0,volume);
  esp_err_t result2 = wm8960_pgaLeftIPVUSet(wm8960);
  return (result1  == ESP_OK && result2 == ESP_OK ) ? ESP_OK : ESP_FAIL;
}

// setLINVOLDB
// Sets the volume of the PGA input buffer amp to a specified dB value 
// passed in as a float argument.
// Valid dB settings are -17.25 up to +30.00
// -17.25 = -17.25dB (MIN)
// ... 0.75dB steps ...
// 30.00 = +30.00dB  (MAX)
esp_err_t wm8960_setLINVOLDB(wm8960_t *wm8960, float dB)
{
  // Create an unsigned integer volume setting variable we can send to 
  // setLINVOL()
  uint8_t volume = wm8960_convertDBtoSetting(dB, WM8960_PGA_GAIN_OFFSET, WM8960_PGA_GAIN_STEPSIZE, WM8960_PGA_GAIN_MIN, WM8960_PGA_GAIN_MAX);

  return wm8960_setLINVOL(wm8960, volume);
}

// 0-63, (0 = -17.25dB) <<-- 0.75dB steps -->> (63 = +30dB)
esp_err_t wm8960_setRINVOL(wm8960_t *wm8960, uint8_t volume)
{
  if(volume > 63) volume = 63; // Limit incoming values max
  esp_err_t result1 = wm8960_register_write_multi_bits(wm8960, WM8960_REG_RIGHT_INPUT_VOLUME,5,0,volume);
  esp_err_t result2 = wm8960_pgaRightIPVUSet(wm8960);
  return (result1  == ESP_OK && result2 == ESP_OK ) ? ESP_OK : ESP_FAIL;
}

// setRINVOLDB
// Sets the volume of the PGA input buffer amp to a specified dB value 
// passed in as a float argument.
// Valid dB settings are -17.25 up to +30.00
// -17.25 = -17.25dB (MIN)
// ... 0.75dB steps ...
// 30.00 = +30.00dB  (MAX)
esp_err_t wm8960_setRINVOLDB(wm8960_t *wm8960, float dB)
{
  // Create an unsigned integer volume setting variable we can send to 
  // setRINVOL(wm8960_t *wm8960)
  uint8_t volume = wm8960_convertDBtoSetting(dB, WM8960_PGA_GAIN_OFFSET, WM8960_PGA_GAIN_STEPSIZE, WM8960_PGA_GAIN_MIN, WM8960_PGA_GAIN_MAX);

  return wm8960_setRINVOL(wm8960, volume);
}

// Zero Cross prevents zipper sounds on volume changes
// Sets both left and right PGAs
esp_err_t wm8960_enablePgaZeroCross(wm8960_t *wm8960)
{
  if (wm8960_register_write_bit(wm8960, WM8960_REG_LEFT_INPUT_VOLUME, 6, 1) != ESP_OK) return ESP_FAIL;
  return wm8960_register_write_bit(wm8960, WM8960_REG_RIGHT_INPUT_VOLUME, 6, 1);
}

esp_err_t wm8960_disablePgaZeroCross(wm8960_t *wm8960)
{
  if (wm8960_register_write_bit(wm8960, WM8960_REG_LEFT_INPUT_VOLUME, 6, 0) != ESP_OK) return ESP_FAIL;
  return wm8960_register_write_bit(wm8960, WM8960_REG_RIGHT_INPUT_VOLUME, 6, 0);
}

esp_err_t wm8960_enableLINMUTE(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_LEFT_INPUT_VOLUME, 7, 1);
}

esp_err_t wm8960_disableLINMUTE(wm8960_t *wm8960)
{
  wm8960_register_write_bit(wm8960, WM8960_REG_LEFT_INPUT_VOLUME, 7, 0);
  return wm8960_register_write_bit(wm8960, WM8960_REG_LEFT_INPUT_VOLUME, 8, 1);
}

esp_err_t wm8960_enableRINMUTE(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_RIGHT_INPUT_VOLUME, 7, 1);
}

esp_err_t wm8960_disableRINMUTE(wm8960_t *wm8960)
{
  wm8960_register_write_bit(wm8960, WM8960_REG_RIGHT_INPUT_VOLUME, 7, 0);
  return wm8960_register_write_bit(wm8960, WM8960_REG_RIGHT_INPUT_VOLUME, 8, 1);
}

// Causes left and right input PGA volumes to be updated (LINVOL and RINVOL)
esp_err_t wm8960_pgaLeftIPVUSet(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_LEFT_INPUT_VOLUME, 8, 1);
}

 // Causes left and right input PGA volumes to be updated (LINVOL and RINVOL)
esp_err_t wm8960_pgaRightIPVUSet(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_RIGHT_INPUT_VOLUME, 8, 1);
}


// Input Boosts

// 0-3, 0 = +0dB, 1 = +13dB, 2 = +20dB, 3 = +29dB
esp_err_t wm8960_setLMICBOOST(wm8960_t *wm8960, int8_t boost_gain)
{
  if(boost_gain > 3) boost_gain = 3; // Limit incoming values max
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_ADCL_SIGNAL_PATH,5,4,boost_gain);
}

// 0-3, 0 = +0dB, 1 = +13dB, 2 = +20dB, 3 = +29dB
esp_err_t wm8960_setRMICBOOST(wm8960_t *wm8960, int8_t boost_gain)
{
  if(boost_gain > 3) boost_gain = 3; // Limit incoming values max
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_ADCR_SIGNAL_PATH,5,4,boost_gain);
}

// 0-7, 0 = Mute, 1 = -12dB ... 3dB steps ... 7 = +6dB
esp_err_t wm8960_setLIN3BOOST(wm8960_t *wm8960, int8_t boost_gain)
{
  if(boost_gain > 7) boost_gain = 7; // Limit incoming values max
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_INPUT_BOOST_MIXER_1,6,4,boost_gain);
}

// 0-7, 0 = Mute, 1 = -12dB ... 3dB steps ... 7 = +6dB
esp_err_t wm8960_setLIN2BOOST(wm8960_t *wm8960, int8_t boost_gain)
{
  if(boost_gain > 7) boost_gain = 7; // Limit incoming values max
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_INPUT_BOOST_MIXER_1,3,1,boost_gain);
}

// 0-7, 0 = Mute, 1 = -12dB ... 3dB steps ... 7 = +6dB
esp_err_t wm8960_setRIN3BOOST(wm8960_t *wm8960, int8_t boost_gain)
{
  if(boost_gain > 7) boost_gain = 7; // Limit incoming values max
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_INPUT_BOOST_MIXER_2,6,4,boost_gain);
}

// 0-7, 0 = Mute, 1 = -12dB ... 3dB steps ... 7 = +6dB	
esp_err_t wm8960_setRIN2BOOST(wm8960_t *wm8960, int8_t boost_gain)
{
  if(boost_gain > 7) boost_gain = 7; // Limit incoming values max
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_INPUT_BOOST_MIXER_2,3,1,boost_gain);
}

// Mic Bias control
esp_err_t wm8960_enableMicBias(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_1, 1, 1);
}

esp_err_t wm8960_disableMicBias(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_1, 1, 0);
}

// WM8960_MIC_BIAS_VOLTAGE_0_9_AVDD (0.9*AVDD) 
// or WM8960_MIC_BIAS_VOLTAGE_0_65_AVDD (0.65*AVDD)
esp_err_t wm8960_setMicBiasVoltage(wm8960_t *wm8960, bool voltage)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_ADDITIONAL_CONTROL_4, 0, voltage);
}

/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// ADC
/////////////////////////////////////////////////////////

esp_err_t wm8960_enableAdcLeft(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_1, 3, 1);
}

esp_err_t wm8960_disableAdcLeft(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_1, 3, 0);
}

esp_err_t wm8960_enableAdcRight(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_1, 2, 1);
}

esp_err_t wm8960_disableAdcRight(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_1, 2, 0);
}

// ADC digital volume
// Note, also needs to handle control of the ADCVU bits (volume update).
// Valid inputs are 0-255
// 0 = mute
// 1 = -97dB
// ... 0.5dB steps up to
// 195 = +0dB
// 255 = +30dB

esp_err_t wm8960_setAdcLeftDigitalVolume(wm8960_t *wm8960, uint8_t volume)
{
  esp_err_t result1 = wm8960_register_write_multi_bits(wm8960, WM8960_REG_LEFT_ADC_VOLUME,7,0,volume);
  esp_err_t result2 = wm8960_adcLeftADCVUSet(wm8960);
  return (result1  == ESP_OK && result2 == ESP_OK ) ? ESP_OK : ESP_FAIL;
}
esp_err_t wm8960_setAdcRightDigitalVolume(wm8960_t *wm8960, uint8_t volume)
{
  esp_err_t result1 = wm8960_register_write_multi_bits(wm8960, WM8960_REG_RIGHT_ADC_VOLUME,7,0,volume);
  esp_err_t result2 = wm8960_adcRightADCVUSet(wm8960);
  return (result1  == ESP_OK && result2 == ESP_OK ) ? ESP_OK : ESP_FAIL;
}

// Causes left and right input adc digital volumes to be updated
esp_err_t wm8960_adcLeftADCVUSet(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_LEFT_ADC_VOLUME, 8, 1);
}

// Causes left and right input adc digital volumes to be updated
esp_err_t wm8960_adcRightADCVUSet(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_RIGHT_ADC_VOLUME, 8, 1);
}

// ADC digital volume DB
// Sets the volume of the ADC to a specified dB value passed in as a float 
// argument.
// Valid dB settings are -97.00 up to +30.0 (0.5dB steps)
// -97.50 (or lower) = MUTE
// -97.00 = -97.00dB (MIN)
// ... 0.5dB steps ...
// 30.00 = +30.00dB  (MAX)

esp_err_t wm8960_setAdcLeftDigitalVolumeDB(wm8960_t *wm8960, float dB)
{
  // Create an unsigned integer volume setting variable we can send to 
  // setAdcLeftDigitalVolume()
  uint8_t volume = wm8960_convertDBtoSetting(dB, WM8960_ADC_GAIN_OFFSET, WM8960_ADC_GAIN_STEPSIZE, WM8960_ADC_GAIN_MIN, WM8960_ADC_GAIN_MAX);

  return wm8960_setAdcLeftDigitalVolume(wm8960, volume);
}
  esp_err_t wm8960_setAdcRightDigitalVolumeDB(wm8960_t *wm8960, float dB)
{
  // Create an unsigned integer volume setting variable we can send to 
  // setAdcRightDigitalVolume(wm8960_t *wm8960)
  uint8_t volume = wm8960_convertDBtoSetting(dB, WM8960_ADC_GAIN_OFFSET, WM8960_ADC_GAIN_STEPSIZE, WM8960_ADC_GAIN_MIN, WM8960_ADC_GAIN_MAX);

  return wm8960_setAdcRightDigitalVolume(wm8960, volume);
}

/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// ALC
/////////////////////////////////////////////////////////

// Automatic Level Control
// Note that when the ALC function is enabled, the settings of registers 0 and 
// 1 (LINVOL, IPVU, LIZC, LINMUTE, RINVOL, RIZC and RINMUTE) are ignored.
esp_err_t wm8960_enableAlc(wm8960_t *wm8960, uint8_t mode)
{
  bool bit8 = (mode>>1);
  bool bit7 = (mode & 0b00000001);
  if (wm8960_register_write_bit(wm8960, WM8960_REG_ALC1, 8, bit8) != ESP_OK) return ESP_FAIL;
  return wm8960_register_write_bit(wm8960, WM8960_REG_ALC1, 7, bit7);
}

 // Also sets alc sample rate to match global sample rate.
esp_err_t wm8960_disableAlc(wm8960_t *wm8960)
{
  if (wm8960_register_write_bit(wm8960, WM8960_REG_ALC1, 8, 0) != ESP_OK) return ESP_FAIL;
  return wm8960_register_write_bit(wm8960, WM8960_REG_ALC1, 7, 0);
}

// Valid inputs are 0-15, 0 = -22.5dB FS, ... 1.5dB steps ... , 15 = -1.5dB FS
esp_err_t wm8960_setAlcTarget(wm8960_t *wm8960, uint8_t target)
{
  if(target > 15) target = 15; // Limit incoming values max
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_ALC1,3,0,target);
}

// Valid inputs are 0-10, 0 = 24ms, 1 = 48ms, ... 10 = 24.58seconds
esp_err_t wm8960_setAlcDecay(wm8960_t *wm8960, uint8_t decay)
{
  if(decay > 10) decay = 10; // Limit incoming values max
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_ALC3,7,4,decay);
}

// Valid inputs are 0-10, 0 = 6ms, 1 = 12ms, 2 = 24ms, ... 10 = 6.14seconds
esp_err_t wm8960_setAlcAttack(wm8960_t *wm8960, uint8_t attack)
{
  if(attack > 10) attack = 10; // Limit incoming values max
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_ALC3,3,0,attack);
}

// Valid inputs are 0-7, 0 = -12dB, ... 7 = +30dB
esp_err_t wm8960_setAlcMaxGain(wm8960_t *wm8960, uint8_t maxGain)
{
  if(maxGain > 7) maxGain = 7; // Limit incoming values max
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_ALC1,6,4,maxGain);
}

// Valid inputs are 0-7, 0 = -17.25dB, ... 7 = +24.75dB
esp_err_t wm8960_setAlcMinGain(wm8960_t *wm8960, uint8_t minGain)
{
  if(minGain > 7) minGain = 7; // Limit incoming values max
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_ALC2,6,4,minGain);
}

// Valid inputs are 0-15, 0 = 0ms, ... 15 = 43.691s
esp_err_t wm8960_setAlcHold(wm8960_t *wm8960, uint8_t hold)
{
  if(hold > 15) hold = 15; // Limit incoming values max
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_ALC2,3,0,hold);
}

// Peak Limiter
esp_err_t wm8960_enablePeakLimiter(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_ALC3, 8, 1);
}

esp_err_t wm8960_disablePeakLimiter(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_ALC3, 8, 0);
}

// Noise Gate
esp_err_t wm8960_enableNoiseGate(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_NOISE_GATE, 0, 1);
}

esp_err_t wm8960_disableNoiseGate(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_NOISE_GATE, 0, 0);
}

// 0-31, 0 = -76.5dBfs, 31 = -30dBfs
esp_err_t wm8960_setNoiseGateThreshold(wm8960_t *wm8960, int8_t threshold)
{
  return ESP_OK;
}

/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// DAC
/////////////////////////////////////////////////////////

// Enable/disble each channel
esp_err_t wm8960_enableDacLeft(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_2, 8, 1);
}

esp_err_t wm8960_disableDacLeft(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_2, 8, 0);
}

esp_err_t wm8960_enableDacRight(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_2, 7, 1);
}

esp_err_t wm8960_disableDacRight(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_2, 7, 0);
}

// DAC digital volume
// Valid inputs are 0-255
// 0 = mute
// 1 = -127dB
// ... 0.5dB steps up to
// 255 = 0dB
esp_err_t wm8960_setDacLeftDigitalVolume(wm8960_t *wm8960, int8_t volume)
{
  esp_err_t result1 = wm8960_register_write_multi_bits(wm8960, WM8960_REG_LEFT_DAC_VOLUME,7,0,volume);
  esp_err_t result2 = wm8960_dacLeftDACVUSet(wm8960);
  return (result1  == ESP_OK && result2 == ESP_OK ) ? ESP_OK : ESP_FAIL;
}

esp_err_t wm8960_setDacRightDigitalVolume(wm8960_t *wm8960, int8_t volume)
{
  esp_err_t result1 = wm8960_register_write_multi_bits(wm8960, WM8960_REG_RIGHT_DAC_VOLUME,7,0,volume);
  esp_err_t result2 = wm8960_dacRightDACVUSet(wm8960);
  return (result1  == ESP_OK && result2 == ESP_OK ) ? ESP_OK : ESP_FAIL;
}

// Causes left and right input dac digital volumes to be updated
esp_err_t wm8960_dacLeftDACVUSet(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_LEFT_DAC_VOLUME, 8, 1);
}

 // Causes left and right input dac digital volumes to be updated
esp_err_t wm8960_dacRightDACVUSet(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_RIGHT_DAC_VOLUME, 8, 1);
}	

// DAC digital volume DB
// Sets the volume of the DAC to a specified dB value passed in as a float 
// argument.
// Valid dB settings are -97.00 up to +30.0 (0.5dB steps)
// -97.50 (or lower) = MUTE
// -97.00 = -97.00dB (MIN)
// ... 0.5dB steps ...
// 30.00 = +30.00dB  (MAX)

esp_err_t wm8960_setDacLeftDigitalVolumeDB(wm8960_t *wm8960, float dB)
{
  // Create an unsigned integer volume setting variable we can send to 
  // setDacLeftDigitalVolume(wm8960_t *wm8960)
  uint8_t volume = wm8960_convertDBtoSetting(dB, WM8960_DAC_GAIN_OFFSET, WM8960_DAC_GAIN_STEPSIZE, WM8960_DAC_GAIN_MIN, WM8960_DAC_GAIN_MAX);

  return wm8960_setDacLeftDigitalVolume(wm8960, volume);
}

esp_err_t wm8960_setDacRightDigitalVolumeDB(wm8960_t *wm8960, float dB)
{
  // Create an unsigned integer volume setting variable we can send to 
  // setDacRightDigitalVolume()
  uint8_t volume = wm8960_convertDBtoSetting(dB, WM8960_DAC_GAIN_OFFSET, WM8960_DAC_GAIN_STEPSIZE, WM8960_DAC_GAIN_MIN, WM8960_DAC_GAIN_MAX);

  return wm8960_setDacRightDigitalVolume(wm8960, volume);
}

// DAC mute
esp_err_t wm8960_enableDacMute(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_ADC_DAC_CTRL_1, 3, 1);
}

esp_err_t wm8960_disableDacMute(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_ADC_DAC_CTRL_1, 3, 0);
}

// 3D Stereo Enhancement
// 3D enable/disable
esp_err_t wm8960_enable3d(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_3D_CONTROL, 0, 1);
}

esp_err_t wm8960_disable3d(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_3D_CONTROL, 0, 0);
}

esp_err_t wm8960_set3dDepth(wm8960_t *wm8960, int8_t depth) // 0 = 0%, 15 = 100%
{
  if(depth > 15) depth = 15; // Limit incoming values max
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_3D_CONTROL,4,1,depth);
}

// DAC output -6dB attentuation enable/disable
esp_err_t wm8960_enableDac6dbAttenuation(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_ADC_DAC_CTRL_1, 7, 1);
}

esp_err_t wm8960_disableDac6dbAttentuation(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_ADC_DAC_CTRL_1, 7, 0);
}

/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// OUTPUT mixers
/////////////////////////////////////////////////////////

// What's connected to what? Oh so many options...
// LOMIX	Left Output Mixer
// ROMIX	Right Output Mixer
// OUT3MIX		Mono Output Mixer

// Enable/disable left and right output mixers
esp_err_t wm8960_enableLOMIX(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_3, 3, 1);
}

esp_err_t wm8960_disableLOMIX(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_3, 3, 0);
}

esp_err_t wm8960_enableROMIX(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_3, 2, 1);
}

esp_err_t wm8960_disableROMIX(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_3, 2, 0);
}

esp_err_t wm8960_enableOUT3MIX(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_2, 1, 1);
}

esp_err_t wm8960_disableOUT3MIX(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_2, 1, 0);
}

// Enable/disable audio path connections/vols to/from output mixers
// See datasheet page 35 for a nice image of all the connections.
esp_err_t wm8960_enableLI2LO(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_LEFT_OUT_MIX_1, 7, 1);
}

esp_err_t wm8960_disableLI2LO(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_LEFT_OUT_MIX_1, 7, 0);
}

// Valid inputs are 0-7. 0 = 0dB ...3dB steps... 7 = -21dB
esp_err_t wm8960_setLI2LOVOL(wm8960_t *wm8960, int8_t volume)
{
  if(volume > 7) volume = 7; // Limit incoming values max
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_LEFT_OUT_MIX_1,6,4,volume);
}

esp_err_t wm8960_enableLB2LO(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_BYPASS_1, 7, 1);
}

esp_err_t wm8960_disableLB2LO(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_LEFT_OUT_MIX_1, 6, 0);
}

// Valid inputs are 0-7. 0 = 0dB ...3dB steps... 7 = -21dB
esp_err_t wm8960_setLB2LOVOL(wm8960_t *wm8960, int8_t volume)
{
  if(volume > 7) volume = 7; // Limit incoming values max
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_BYPASS_1,6,4,volume);
}

esp_err_t wm8960_enableLD2LO(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_LEFT_OUT_MIX_1, 8, 1);
}

esp_err_t wm8960_disableLD2LO(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_LEFT_OUT_MIX_1, 8, 0);
}

esp_err_t wm8960_enableRI2RO(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_RIGHT_OUT_MIX_2, 7, 1);
}

esp_err_t wm8960_disableRI2RO(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_RIGHT_OUT_MIX_2, 7, 0);
}

// Valid inputs are 0-7. 0 = 0dB ...3dB steps... 7 = -21dB
esp_err_t wm8960_setRI2ROVOL(wm8960_t *wm8960, int8_t volume)
{
  if(volume > 7) volume = 7; // Limit incoming values max
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_RIGHT_OUT_MIX_2,6,4,volume);
}

esp_err_t wm8960_enableRB2RO(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_BYPASS_2, 7, 1);
}

esp_err_t wm8960_disableRB2RO(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_RIGHT_OUT_MIX_2, 6, 0);
}

// Valid inputs are 0-7. 0 = 0dB ...3dB steps... 7 = -21dB
esp_err_t wm8960_setRB2ROVOL(wm8960_t *wm8960, int8_t volume)
{
  if(volume > 7) volume = 7; // Limit incoming values max
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_BYPASS_2,6,4,volume);
}

esp_err_t wm8960_enableRD2RO(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_RIGHT_OUT_MIX_2, 8, 1);
}

esp_err_t wm8960_disableRD2RO(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_RIGHT_OUT_MIX_2, 8, 0);
}

// Mono Output mixer. 
// Note, for capless HPs, we'll want this to output a buffered VMID.
// To do this, we need to disable both of these connections.
  esp_err_t wm8960_enableLI2MO(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_MONO_OUT_MIX_1, 7, 1);
}

esp_err_t wm8960_disableLI2MO(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_MONO_OUT_MIX_1, 7, 0);
}

esp_err_t wm8960_enableRI2MO(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_MONO_OUT_MIX_2, 7, 1);
}

esp_err_t wm8960_disableRI2MO(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_MONO_OUT_MIX_2, 7, 0);
}

// Enables VMID in the WM8960_REG_PWR_MGMT_2 register, and set's it to 
// playback/record settings of 2*50Kohm.
// Note, this function is only hear for backwards compatibility with the
// original releases of this library. It is recommended to use the
// setVMID() function instead.
esp_err_t wm8960_enableVMID(wm8960_t *wm8960)
{
  return wm8960_setVMID(wm8960, WM8960_VMIDSEL_2X50KOHM);
}

esp_err_t wm8960_disableVMID(wm8960_t *wm8960)
{
  return wm8960_setVMID(wm8960, WM8960_VMIDSEL_DISABLED);
}

// setVMID
// Sets the VMID signal to one of three possible settings.
// 4 options:
// WM8960_VMIDSEL_DISABLED
// WM8960_VMIDSEL_2X50KOHM (playback / record)
// WM8960_VMIDSEL_2X250KOHM (for low power / standby)
// WM8960_VMIDSEL_2X5KOHM (for fast start-up)
esp_err_t wm8960_setVMID(wm8960_t *wm8960, int8_t setting)
{
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_PWR_MGMT_1, 8, 7, setting);
}

/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// Headphones
/////////////////////////////////////////////////////////

// Enable and disable headphones (mute)
esp_err_t wm8960_enableHeadphones(wm8960_t *wm8960)
{
  return (wm8960_enableRightHeadphone(wm8960) & wm8960_enableLeftHeadphone(wm8960));
}

esp_err_t wm8960_disableHeadphones(wm8960_t *wm8960)
{
  return (wm8960_disableRightHeadphone(wm8960) & wm8960_disableLeftHeadphone(wm8960));
}

esp_err_t wm8960_enableRightHeadphone(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_2, 5, 1);
}

esp_err_t wm8960_disableRightHeadphone(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_2, 5, 0);
}

esp_err_t wm8960_enableLeftHeadphone(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_2, 6, 1);
}

esp_err_t wm8960_disableLeftHeadphone(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_2, 6, 0);
}

esp_err_t wm8960_enableHeadphoneStandby(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_ANTI_POP_1, 0, 1);
}

esp_err_t wm8960_disableHeadphoneStandby(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_ANTI_POP_1, 0, 0);
}

// SetHeadphoneVolume
// Sets the volume for both left and right headphone outputs
// 
// Although you can control each headphone output independently, here we are
// Going to assume you want both left and right to do the same thing.
// 
// Valid inputs: 47-127. 0-47 = mute, 48 = -73dB ... 1dB steps ... 127 = +6dB
esp_err_t wm8960_setHeadphoneVolume(wm8960_t *wm8960, uint8_t volume)
{		
  // Updates both left and right channels
	// Handles the OUT1VU (volume update) bit control, so that it happens at the 
  // same time on both channels. Note, we must also make sure that the outputs 
  // are enabled in the WM8960_REG_PWR_MGMT_2 [6:5]
  // Grab local copy of register
  // Modify the bits we need to
  // Write register in device, including the volume update bit write
  // If successful, save locally.

  // Limit inputs
  if (volume > 127) volume = 127;

  // LEFT
    esp_err_t result1 = wm8960_register_write_multi_bits(wm8960, WM8960_REG_LOUT1_VOLUME,6,0,volume);
  // RIGHT
    esp_err_t result2 = wm8960_register_write_multi_bits(wm8960, WM8960_REG_ROUT1_VOLUME,6,0,volume);
  // UPDATES

  // Updated left channel
    esp_err_t result3 = wm8960_register_write_bit(wm8960, WM8960_REG_LOUT1_VOLUME, 8, 1);

  // Updated right channel
    esp_err_t result4 = wm8960_register_write_bit(wm8960, WM8960_REG_ROUT1_VOLUME, 8, 1);

    return (result1 == ESP_OK && result2 == ESP_OK && result3 == ESP_OK && result4 == ESP_OK) ? ESP_OK : ESP_FAIL; // If all writes ACK'd
}

esp_err_t wm8960_enableHeadphoneJackDetect(wm8960_t *wm8960) {
  esp_err_t result1 = wm8960_register_write_bit(wm8960, WM8960_REG_ADDITIONAL_CONTROL_2, 6, 1);
  esp_err_t result2 = wm8960_register_write_bit(wm8960, WM8960_REG_ADDITIONAL_CONTROL_1, 0, 1); // TOEN - slow clock enable
  return (result1  == ESP_OK && result2 == ESP_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t wm8960_disableHeadphoneJackDetect(wm8960_t *wm8960) {
  esp_err_t result1 = wm8960_register_write_bit(wm8960, WM8960_REG_ADDITIONAL_CONTROL_2, 6, 0);
  esp_err_t result2 = wm8960_register_write_bit(wm8960, WM8960_REG_ADDITIONAL_CONTROL_1, 0, 0); // TOEN - slow clock disable
  return (result1 == ESP_OK && result2 == ESP_OK) ? ESP_OK : ESP_FAIL;
}

// SetHeadphoneJackDetectInput
// Sets the input pin for jack insertion detection
//
// Valid inputs: WM8960_JACKDETECT_LINPUT3, WM8960_JACKDETECT_RINPUT3, WM8960_JACKDETECT_GPIO1
esp_err_t wm8960_setHeadphoneJackDetectInput(wm8960_t *wm8960, int8_t setting) {
  esp_err_t result1;
  esp_err_t result2;
  switch (setting) {
    case WM8960_JACKDETECT_LINPUT3:
      result1 = wm8960_register_write_bit(wm8960, WM8960_REG_ADDITIONAL_CONTROL_4, 3, 1);
      result2 = wm8960_register_write_bit(wm8960, WM8960_REG_ADDITIONAL_CONTROL_4, 2, 0); // JD2
      break;
    case WM8960_JACKDETECT_RINPUT3:
      result1 = wm8960_register_write_bit(wm8960, WM8960_REG_ADDITIONAL_CONTROL_4, 3, 1);
      result2 = wm8960_register_write_bit(wm8960, WM8960_REG_ADDITIONAL_CONTROL_4, 2, 1); // JD3
      break;
    default:
      result1 = wm8960_register_write_bit(wm8960, WM8960_REG_ADDITIONAL_CONTROL_4, 3, 0); // GPIO1
      result2 = wm8960_register_write_bit(wm8960, WM8960_REG_ADDITIONAL_CONTROL_4, 2, 0); // defaults to 0
      break;
  }
  return (result1 == ESP_OK && result2 == ESP_OK) ? ESP_OK : ESP_FAIL;
}

// Set headphone volume dB
// Sets the volume of the headphone output buffer amp to a specified dB value 
// passed in as a float argument.
// Valid dB settings are -74.0 up to +6.0
// Note, we are accepting float arguments here, in order to keep it consistent
// with other volume setting functions in this library that can do partial dB
// values (such as the PGA, ADC and DAC gains).
// -74 (or lower) = MUTE
// -73 = -73dB (MIN)
// ... 1dB steps ...
// 0 = 0dB
// ... 1dB steps ...
// 6 = +6dB  (MAX)
esp_err_t wm8960_setHeadphoneVolumeDB(wm8960_t *wm8960, float dB)
{
  // Create an unsigned integer volume setting variable we can send to 
  // setHeadphoneVolume()
  uint8_t volume = wm8960_convertDBtoSetting(dB, WM8960_HP_GAIN_OFFSET, WM8960_HP_GAIN_STEPSIZE, WM8960_HP_GAIN_MIN, WM8960_HP_GAIN_MAX);

  return wm8960_setHeadphoneVolume(wm8960, volume);
}

// Zero Cross prevents zipper sounds on volume changes
// Sets both left and right Headphone outputs
esp_err_t wm8960_enableHeadphoneZeroCross(wm8960_t *wm8960)
{
  // Left
  esp_err_t result1 = wm8960_register_write_bit(wm8960, WM8960_REG_LOUT1_VOLUME, 7, 1);

  // Right
  esp_err_t result2 = wm8960_register_write_bit(wm8960, WM8960_REG_ROUT1_VOLUME, 7, 1);
  return (result1 == ESP_OK && result2 == ESP_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t wm8960_disableHeadphoneZeroCross(wm8960_t *wm8960)
{
  // Left
  esp_err_t result1 = wm8960_register_write_bit(wm8960, WM8960_REG_LOUT1_VOLUME, 7, 0);

  // Right
  esp_err_t result2 = wm8960_register_write_bit(wm8960, WM8960_REG_ROUT1_VOLUME, 7, 0);
  return (result1 == ESP_OK && result2 == ESP_OK) ? ESP_OK : ESP_FAIL;
}

/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// Speakers
/////////////////////////////////////////////////////////

// Enable and disable speakers (mute)
esp_err_t wm8960_enableSpeakers(wm8960_t *wm8960)
{
  return (wm8960_enableRightSpeaker(wm8960) & wm8960_enableLeftSpeaker(wm8960));
}

esp_err_t wm8960_disableSpeakers(wm8960_t *wm8960)
{
  return (wm8960_disableRightHeadphone(wm8960) & wm8960_disableLeftHeadphone(wm8960));
}

esp_err_t wm8960_enableRightSpeaker(wm8960_t *wm8960)
{
  // SPK_OP_EN
  esp_err_t result1 = wm8960_register_write_bit(wm8960, WM8960_REG_CLASS_D_CONTROL_1, 7, 1);

  // SPKR
  esp_err_t result2 = wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_2, 3, 1);
  return (result1 == ESP_OK && result2 == ESP_OK ) ? ESP_OK : ESP_FAIL;
}

esp_err_t wm8960_disableRightSpeaker(wm8960_t *wm8960)
{
  // SPK_OP_EN
  esp_err_t result1 = wm8960_register_write_bit(wm8960, WM8960_REG_CLASS_D_CONTROL_1, 7, 0);

  // SPKR
  esp_err_t result2 = wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_2, 3, 0);
  return (result1 == ESP_OK && result2 == ESP_OK ) ? ESP_OK : ESP_FAIL;
}

esp_err_t wm8960_enableLeftSpeaker(wm8960_t *wm8960)
{
  // SPK_OP_EN
  esp_err_t result1 = wm8960_register_write_bit(wm8960, WM8960_REG_CLASS_D_CONTROL_1, 6, 1);

  // SPKL
  esp_err_t result2 = wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_2, 4, 1);
  return (result1 == ESP_OK && result2 == ESP_OK ) ? ESP_OK : ESP_FAIL;
}

esp_err_t wm8960_disableLeftSpeaker(wm8960_t *wm8960)
{
  // SPK_OP_EN
  esp_err_t result1 = wm8960_register_write_bit(wm8960, WM8960_REG_CLASS_D_CONTROL_1, 6, 0);

  // SPKL
  esp_err_t result2 = wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_2, 4, 0);
  return (result1 == ESP_OK && result2 == ESP_OK) ? ESP_OK : ESP_FAIL;
}

// SetSpeakerVolume
// Sets to volume for both left and right speaker outputs
// 
// Although you can control each Speaker output independently, here we are
// Going to assume you want both left and right to do the same thing.
// 
// Valid inputs are 47-127. 0-47 = mute, 48 = -73dB, ... 1dB steps ... , 127 = +6dB
esp_err_t wm8960_setSpeakerVolume(wm8960_t *wm8960, uint8_t volume)
{		
  // Updates both left and right channels
	// Handles the SPKVU (volume update) bit control, so that it happens at the 
  // same time on both channels. Note, we must also make sure that the outputs 
  // are enabled in the WM8960_REG_PWR_MGMT_2 [4:3], and the class D control 
  // reg WM8960_REG_CLASS_D_CONTROL_1 [7:6]

  // Limit inputs
  if (volume > 127) volume = 127;

  // LEFT
  esp_err_t result1 = wm8960_register_write_multi_bits(wm8960, WM8960_REG_LOUT2_VOLUME,6,0,volume);

  // RIGHT
  esp_err_t result2 = wm8960_register_write_multi_bits(wm8960, WM8960_REG_ROUT2_VOLUME,6,0,volume);

  // SPKVU

  // Updated left channel
  esp_err_t result3 = wm8960_register_write_bit(wm8960, WM8960_REG_LOUT2_VOLUME, 8, 1);

  // Updated right channel
  esp_err_t result4 = wm8960_register_write_bit(wm8960, WM8960_REG_ROUT2_VOLUME, 8, 1);

  return (result1 == ESP_OK && result2 == ESP_OK && result3 == ESP_OK && result4 == ESP_OK) ? ESP_OK : ESP_FAIL; // If all writes ACK'd
}

// Set speaker volume dB
// Sets the volume of the class-d speaker output amp to a specified dB value 
// passed in as a float argument.
// Valid dB settings are -74.0 up to +6.0
// Note, we are accepting float arguments here, in order to keep it consistent
// with other volume setting functions in this library that can do partial dB
// values (such as the PGA, ADC and DAC gains).
// -74 (or lower) = MUTE
// -73 = -73dB (MIN)
// ... 1dB steps ...
// 0 = 0dB
// ... 1dB steps ...
// 6 = +6dB  (MAX)
esp_err_t wm8960_setSpeakerVolumeDB(wm8960_t *wm8960, float dB)
{
  // Create an unsigned integer volume setting variable we can send to 
  // setSpeakerVolume()
  uint8_t volume = wm8960_convertDBtoSetting(dB, WM8960_SPEAKER_GAIN_OFFSET, WM8960_SPEAKER_GAIN_STEPSIZE, WM8960_SPEAKER_GAIN_MIN, WM8960_SPEAKER_GAIN_MAX);

  return wm8960_setSpeakerVolume(wm8960, volume);
}

// Zero Cross prevents zipper sounds on volume changes
// Sets both left and right Speaker outputs
esp_err_t wm8960_enableSpeakerZeroCross(wm8960_t *wm8960)
{
  // Left
  esp_err_t result1 = wm8960_register_write_bit(wm8960, WM8960_REG_LOUT2_VOLUME, 7, 1);

  // Right
  esp_err_t result2 = wm8960_register_write_bit(wm8960, WM8960_REG_ROUT2_VOLUME, 7, 1);
  return (result1 == ESP_OK && result2 == ESP_OK ) == ESP_OK ? ESP_OK : ESP_FAIL;
}

esp_err_t wm8960_disableSpeakerZeroCross(wm8960_t *wm8960)
{
  // Left
  esp_err_t result1 = wm8960_register_write_bit(wm8960, WM8960_REG_LOUT2_VOLUME, 7, 0);

  // Right
  esp_err_t result2 = wm8960_register_write_bit(wm8960, WM8960_REG_ROUT2_VOLUME, 7, 0);
  return (result1 == ESP_OK && result2 == ESP_OK ) == ESP_OK ? ESP_OK : ESP_FAIL;
}

// SetSpeakerDcGain
// DC and AC gain - allows signal to be higher than the DACs swing
// (use only if your SPKVDD is high enough to handle a larger signal)
// Valid inputs are 0-5
// 0 = +0dB (1.0x boost) ... up to ... 5 = +5.1dB (1.8x boost)
esp_err_t wm8960_setSpeakerDcGain(wm8960_t *wm8960, uint8_t gain)
{
  if(gain > 5) gain = 5; // Limit incoming values max
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_CLASS_D_CONTROL_3,5,3,gain);
}

// SetSpeakerAcGain
// DC and AC gain - allows signal to be higher than the DACs swing
// (use only if your SPKVDD is high enough to handle a larger signal)
// Valid inputs are 0-5
// 0 = +0dB (1.0x boost) ... up to ... 5 = +5.1dB (1.8x boost)
esp_err_t wm8960_setSpeakerAcGain(wm8960_t *wm8960, uint8_t gain)
{
  if(gain > 5) gain = 5; // Limit incoming values max
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_CLASS_D_CONTROL_3,2,0,gain);
}

//////////////////////////////////////////////
////////////////////////////////////////////// Digital audio interface control
//////////////////////////////////////////////

// Defaults to I2S, peripheral-mode, 24-bit word length

// Loopback
// When enabled, the output data from the ADC audio interface is fed directly 
// into the DAC data input.
esp_err_t wm8960_enableLoopBack(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_AUDIO_INTERFACE_2, 0, 1);
}

esp_err_t wm8960_disableLoopBack(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_AUDIO_INTERFACE_2, 0, 0);
}

/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// Clock controls
/////////////////////////////////////////////////////////

// Getting the Frequency of SampleRate as we wish
// Our MCLK (an external clock on the SFE breakout board) is 24.0MHz.
// According to table 40 (DS pg 58), we want SYSCLK to be 11.2896 for a SR of 
// 44.1KHz. To get that Desired Output (SYSCLK), we need the following settings 
// on the PLL stuff, as found on table 45 (ds pg 61):
// PRESCALE DIVIDE (PLLPRESCALE): 2
// POSTSCALE DVIDE (SYSCLKDIV[1:0]): 2
// FIXED POST-DIVIDE: 4
// R: 7.5264 
// N: 7h
// K: 86C226h

// Example at bottom of table 46, shows that we should be in fractional mode 
// for a 44.1KHz.

// In terms of registers, this is what we want for 44.1KHz
// PLLEN=1			(PLL enable)
// PLLPRESCALE=1	(divide by 2) *This get's us from MCLK (24MHz) down to 12MHZ 
// for F2.
// PLLN=7h			(PLL N value) *this is "int R"
// PLLK=86C226h		(PLL K value) *this is int ( 2^24 * (R- intR)) 
// SDM=1			(Fractional mode)
// CLKSEL=1			(PLL select) 
// MS=0				(Peripheral mode)
// WL=00			(16 bits)
// SYSCLKDIV=2		(Divide by 2)
// ADCDIV=000		(Divide by 1) = 44.1kHz
// DACDIV=000		(Divide by 1) = 44.1kHz
// BCLKDIV=0100		(Divide by 4) = 64fs
// DCLKDIV=111		(Divide by 16) = 705.6kHz

// And now for the functions that will set these registers...
esp_err_t wm8960_enablePLL(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_2, 0, 1);
}

esp_err_t wm8960_disablePLL(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PWR_MGMT_2, 0, 0);
}

esp_err_t wm8960_setPLLPRESCALE(wm8960_t *wm8960, bool div)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_PLL_N, 4, div);
}

esp_err_t wm8960_setPLLN(wm8960_t *wm8960, uint8_t n)
{
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_PLL_N,3,0,n);
}

// Send each nibble of 24-bit value for value K
esp_err_t wm8960_setPLLK(wm8960_t *wm8960, uint8_t one, uint8_t two, uint8_t three)
{
  esp_err_t result1 = wm8960_register_write_multi_bits(wm8960, WM8960_REG_PLL_K_1,5,0,one);
  esp_err_t result2 = wm8960_register_write_multi_bits(wm8960, WM8960_REG_PLL_K_2,8,0,two);
  esp_err_t result3 = wm8960_register_write_multi_bits(wm8960, WM8960_REG_PLL_K_3,8,0,three);
  return (result1 == ESP_OK && result2 == ESP_OK && result3 == ESP_OK) ? ESP_OK : ESP_FAIL; // If all I2C sommands Ack'd, then...
}

// 0=integer, 1=fractional
esp_err_t wm8960_setSMD(wm8960_t *wm8960, bool mode)
{
  // SDM (Sigma-Delta Modulation) is bit 3 of CLOCKING_1 register (0x04), not PLL_N
  return wm8960_register_write_bit(wm8960, WM8960_REG_CLOCKING_1, 3, mode);
}

 // 0=MCLK, 1=PLL_output
esp_err_t wm8960_setCLKSEL(wm8960_t *wm8960, bool sel)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_CLOCKING_1, 0, sel);
}

// (0=divide by 1), (2=div by 2) *1 and 3 are "reserved"
esp_err_t wm8960_setSYSCLKDIV(wm8960_t *wm8960, uint8_t div)
{
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_CLOCKING_1,2,1,div);
}

// 000 = SYSCLK / (1.0*256). See ds pg 57 for other options
esp_err_t wm8960_setADCDIV(wm8960_t *wm8960, uint8_t div)
{
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_CLOCKING_1,8,6,div);
}

// 000 = SYSCLK / (1.0*256). See ds pg 57 for other options
esp_err_t wm8960_setDACDIV(wm8960_t *wm8960, uint8_t div)
{
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_CLOCKING_1,5,3,div);
}

esp_err_t wm8960_setBCLKDIV(wm8960_t *wm8960, uint8_t div)
{
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_CLOCKING_2,3,0,div);
}

// Class D amp, 111= SYSCLK/16, so 11.2896MHz/16 = 705.6KHz
esp_err_t wm8960_setDCLKDIV(wm8960_t *wm8960, uint8_t div)
{
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_CLOCKING_2,8,6,div);
}

esp_err_t wm8960_setALRCGPIO(wm8960_t *wm8960)
{
  // This setting should not be changed if ADCs are enabled.
  return wm8960_register_write_bit(wm8960, WM8960_REG_AUDIO_INTERFACE_2, 6, 1);
}

esp_err_t wm8960_enableMasterMode(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_AUDIO_INTERFACE_1, 6, 1);
}

esp_err_t wm8960_enablePeripheralMode(wm8960_t *wm8960)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_AUDIO_INTERFACE_1, 6, 0);
}

esp_err_t wm8960_setWL(wm8960_t *wm8960, uint8_t word_length)
{
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_AUDIO_INTERFACE_1,3,2,word_length);
}

esp_err_t wm8960_setLRP(wm8960_t *wm8960, bool polarity)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_AUDIO_INTERFACE_1, 4, polarity);
}

esp_err_t wm8960_setALRSWAP(wm8960_t *wm8960, bool swap)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_AUDIO_INTERFACE_1, 8, swap);
}

esp_err_t wm8960_setVROI(wm8960_t *wm8960, bool setting)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_ADDITIONAL_CONTROL_3, 6, setting);
}

esp_err_t wm8960_setVSEL(wm8960_t *wm8960, uint8_t setting)
{
  return wm8960_register_write_multi_bits(wm8960, WM8960_REG_ADDITIONAL_CONTROL_1,7,6,setting);
}




esp_err_t wm8960_setDacMonoMix(wm8960_t *wm8960, bool setting)
{
  return wm8960_register_write_bit(wm8960, WM8960_REG_ADDITIONAL_CONTROL_1, 4, setting);
}



// convertDBtoSetting
// This function will take in a dB value (as a float), and return the 
// corresponding volume setting necessary.
// For example, Headphone volume control goes from 47-120.
// While PGA gain control is from 0-63.
// The offset values allow for proper conversion.
//
// dB - float value of dB
//
// offset - the differnce from lowest dB value to lowest setting value
//
// stepSize - the dB step for each setting (aka the "resolution" of the setting)
// This is 0.75dB for the PGAs, 0.5 for ADC/DAC, and 1dB for most other amps.
//
// minDB - float of minimum dB setting allowed, note this is not mute on the 
// amp. "True mute" is always one stepSize lower.
//
// maxDB - float of maximum dB setting allowed. If you send anything higher, it
// will be limited to this max value.
uint8_t wm8960_convertDBtoSetting(float dB, float offset, float stepSize, float minDB, float maxDB)
{
  // Limit incoming dB values to acceptable range. Note, the minimum limit we
  // want to limit this too is actually one step lower than the minDB, because
  // that is still an acceptable dB level (it is actually "true mute").
  // Note, the PGA amp does not have a "true mute" setting available, so we 
  // must check for its unique minDB of -17.25.

  // Limit max. This is the same for all amps.
  if (dB > maxDB) dB = maxDB;

  // PGA amp doesn't have mute setting, so minDB should be limited to minDB
  // Let's check for the PGAs unique minDB (-17.25) to know we are currently
  // converting a PGA setting.
  if(minDB == WM8960_PGA_GAIN_MIN) 
  {
    if (dB < minDB) dB = minDB;
  }
  else // Not PGA. All other amps have a mute setting below minDb
  {
    if (dB < (minDB - stepSize)) dB = (minDB - stepSize);
  }

  // Adjust for offset
  // Offset is the number that gets us from the minimum dB option of an amp
  // up to the minimum setting value in the register.
  dB = dB + offset; 

  // Find out how many steps we are above the minimum (at this point, our 
  // minimum is "0". Note, because dB comes in as a float, the result of this 
  // division (volume) can be a partial number. We will round that next.
  float volume = dB / stepSize;

  volume = round(volume); // round to the nearest setting value.

  // Serial debug (optional)
  // Serial.print("\t");
  // Serial.print((uint8_t)volume);

  return (uint8_t)volume; // cast from float to unsigned 8-bit integer.
}