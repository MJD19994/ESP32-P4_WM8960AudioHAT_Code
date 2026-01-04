/*
 * WM8960 Audio HAT Test for ESP32-P4
 * I2S DAC Playback with 2kHz Sine Wave Generator
 * Based on SparkFun WM8960 Arduino Library Examples
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#define _USE_MATH_DEFINES
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "wm8960.h"
#include "driver/gpio.h"

// Extern declaration for WM8960 driver internal function (not in public header)
extern esp_err_t wm8960_register_write_multi_bits(wm8960_t *wm8960, uint8_t register_address, uint8_t setting_msb_num, uint8_t setting_lsb_num, uint8_t setting);

//=============================================================================
// Configuration & Defines
//=============================================================================

static const char *TAG = "WM8960_EXAMPLE";

// I2C Configuration
#define I2C_SDA_PIN             7
#define I2C_SCL_PIN             8
#define I2C_FREQ_HZ             100000
#define WM8960_I2C_ADDR         0x1A

// I2S Configuration
#define I2S_BCLK_PIN            22
#define I2S_LRCLK_PIN           53
#define I2S_DOUT_PIN            6       // ESP32 DOUT → Jumper to GPIO45 header → Waveshare I2S_DAC
#define I2S_DIN_PIN             27     // ESP32 DIN ← Waveshare I2S_ADC (from WM8960 ADCDAT)
#define I2S_SAMPLE_RATE         44100
#define I2S_BUFFER_SIZE         256

// Sine Wave Generator
#define SINE_FREQUENCY          2000.0f          // 2kHz tone
#define SINE_AMPLITUDE          8000.0f          // Volume level
#define SINE_SAMPLES            128              // Samples per buffer

//=============================================================================
// Global Handles
//=============================================================================

static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static wm8960_t wm8960_device;
static i2s_chan_handle_t tx_handle = NULL;
static i2s_chan_handle_t rx_handle = NULL;  // Create RX handle even if unused (like Example_08)

//=============================================================================
// I2C Initialization
//=============================================================================

static esp_err_t i2c_master_init(void)
{
    ESP_LOGI(TAG, "Initializing I2C bus...");
    
    i2c_master_bus_config_t i2c_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    esp_err_t ret = i2c_new_master_bus(&i2c_cfg, &i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus!");
        return ret;
    }
    
    // Add WM8960 device to I2C bus
    i2c_device_config_t dev_cfg = {
        .device_address = WM8960_I2C_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    
    ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &wm8960_device.handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add WM8960 to I2C bus!");
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C bus initialized successfully");
    return ESP_OK;
}

//=============================================================================
// WM8960 Codec Configuration for I2S DAC Output
// (Matches Reference.c wm8960_i2s_dac_setup exactly)
//=============================================================================

static esp_err_t wm8960_codec_init(void)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "=== Configuring WM8960 for I2S DAC Output ===");
    
    // Reset codec for clean slate
    ret = wm8960_reset(&wm8960_device);
    ESP_LOGI(TAG, "  reset: %s", esp_err_to_name(ret));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Re-initialize local register copy after reset
    // After reset, the hardware is in default state, so sync the local copy
    ESP_LOGI(TAG, "  Copying WM8960_REGISTER_DEFAULTS (224 regs x 2 bytes)...");
    memcpy(wm8960_device.register_local_copy, WM8960_REGISTER_DEFAULTS, sizeof(WM8960_REGISTER_DEFAULTS));
    
    // VERIFY the copy worked
    uint16_t reg_0x05_after_copy = wm8960_device.register_local_copy[0x05];
    ESP_LOGI(TAG, "  After defaults copy, REG 0x05 = 0x%04X (should be 0x0008)", reg_0x05_after_copy);
    
    // ===== POWER & REFERENCE =====
    ret = wm8960_enableVREF(&wm8960_device);
    ESP_LOGI(TAG, "  enableVREF: %s", esp_err_to_name(ret));
    ret = wm8960_enableVMID(&wm8960_device);
    ESP_LOGI(TAG, "  enableVMID: %s", esp_err_to_name(ret));
    vTaskDelay(pdMS_TO_TICKS(500));  // Wait for VMID to stabilize
    
    // ===== MICROPHONE BIAS =====
    ret = wm8960_enableMicBias(&wm8960_device);
    ESP_LOGI(TAG, "  enableMicBias: %s", esp_err_to_name(ret));
    
    // ===== Enable Signal Flow to ADC =====
    ret = wm8960_enableLMIC(&wm8960_device);
    ESP_LOGI(TAG, "  enableLMIC: %s", esp_err_to_name(ret));
    ret = wm8960_enableRMIC(&wm8960_device);
    ESP_LOGI(TAG, "  enableRMIC: %s", esp_err_to_name(ret));
    
    // Connect INPUT1 (microphone input) to PGA (Pre-Gain Amplifier)
    ret = wm8960_connectLMN1(&wm8960_device);
    ESP_LOGI(TAG, "  connectLMN1: %s", esp_err_to_name(ret));
    ret = wm8960_connectRMN1(&wm8960_device);
    ESP_LOGI(TAG, "  connectRMN1: %s", esp_err_to_name(ret));
    
    // Disable INPUT1 mutes (unmute inputs)
    ret = wm8960_disableLINMUTE(&wm8960_device);
    ESP_LOGI(TAG, "  disableLINMUTE: %s", esp_err_to_name(ret));
    ret = wm8960_disableRINMUTE(&wm8960_device);
    ESP_LOGI(TAG, "  disableRINMUTE: %s", esp_err_to_name(ret));
    
    // Connect mic PGA output to boost mixer (AINL/AINR)
    ret = wm8960_connectLMIC2B(&wm8960_device);
    ESP_LOGI(TAG, "  connectLMIC2B: %s", esp_err_to_name(ret));
    ret = wm8960_connectRMIC2B(&wm8960_device);
    ESP_LOGI(TAG, "  connectRMIC2B: %s", esp_err_to_name(ret));
    
    // Enable analog input mixers (CRITICAL - these feed the ADC!)
    // Don't confuse with LB2LO/RB2RO which feed output mixers (causes feedback)
    ret = wm8960_enableAINL(&wm8960_device);
    ESP_LOGI(TAG, "  enableAINL (→ADC): %s", esp_err_to_name(ret));
    ret = wm8960_enableAINR(&wm8960_device);
    ESP_LOGI(TAG, "  enableAINR (→ADC): %s", esp_err_to_name(ret));

    // ===== MICROPHONE BOOST GAIN =====
    // Increase mic boost (LB2LO/RB2RO are now properly disabled - no feedback!)
    ret = wm8960_setLMICBOOST(&wm8960_device, 2);  // +20dB (good amplification for quiet voices)
    ESP_LOGI(TAG, "  setLMICBOOST(+20dB): %s", esp_err_to_name(ret));
    ret = wm8960_setRMICBOOST(&wm8960_device, 2);  // +20dB
    ESP_LOGI(TAG, "  setRMICBOOST(+20dB): %s", esp_err_to_name(ret));

    // ===== INPUT PGA VOLUME (Pre-Gain Amplifier) =====
    // Add additional gain via PGA for audible signal
    ret = wm8960_setLINVOLDB(&wm8960_device, 15.0f);  // +15dB (good middle ground)
    ESP_LOGI(TAG, "  setLINVOLDB(+15dB): %s", esp_err_to_name(ret));
    ret = wm8960_setRINVOLDB(&wm8960_device, 15.0f);  // +15dB
    ESP_LOGI(TAG, "  setRINVOLDB(+15dB): %s", esp_err_to_name(ret));

    // ===== DAC → OUTPUT MIXER ROUTING (for I2S loopback) =====
    ret = wm8960_enableLD2LO(&wm8960_device);
    ESP_LOGI(TAG, "  enableLD2LO: %s", esp_err_to_name(ret));
    ret = wm8960_enableRD2RO(&wm8960_device);
    ESP_LOGI(TAG, "  enableRD2RO: %s", esp_err_to_name(ret));
    
    // ===== DISABLE BOOST MIXER → OUTPUT (prevents feedback loop) =====
    // Don't use analog bypass - we're doing digital I2S loopback
    ret = wm8960_disableLB2LO(&wm8960_device);
    ESP_LOGI(TAG, "  disableLB2LO (no analog bypass): %s", esp_err_to_name(ret));
    ret = wm8960_disableRB2RO(&wm8960_device);
    ESP_LOGI(TAG, "  disableRB2RO (no analog bypass): %s", esp_err_to_name(ret));
    
    // Enable output mixers
    ret = wm8960_enableLOMIX(&wm8960_device);
    ESP_LOGI(TAG, "  enableLOMIX: %s", esp_err_to_name(ret));
    ret = wm8960_enableROMIX(&wm8960_device);
    ESP_LOGI(TAG, "  enableROMIX: %s", esp_err_to_name(ret));
    


    // ===== CLOCK CONFIGURATION CORRECTED (Configure BEFORE enabling PLL) =====
    ret = wm8960_setPLLPRESCALE(&wm8960_device, WM8960_PLLPRESCALE_DIV_2);
    ESP_LOGI(TAG, "  setPLLPRESCALE: %s", esp_err_to_name(ret));
    ret = wm8960_setSMD(&wm8960_device, WM8960_PLL_MODE_FRACTIONAL);
    ESP_LOGI(TAG, "  setSMD: %s", esp_err_to_name(ret));
    ret = wm8960_setPLLN(&wm8960_device, 7);
    ESP_LOGI(TAG, "  setPLLN: %s", esp_err_to_name(ret));
    ret = wm8960_setPLLK(&wm8960_device, 0x86, 0xC2, 0x26);
    ESP_LOGI(TAG, "  setPLLK: %s", esp_err_to_name(ret));
    ret = wm8960_enablePLL(&wm8960_device);  // Enable AFTER configuration
    ESP_LOGI(TAG, "  enablePLL: %s", esp_err_to_name(ret));
    ret = wm8960_setCLKSEL(&wm8960_device, WM8960_CLKSEL_PLL);
    ESP_LOGI(TAG, "  setCLKSEL: %s", esp_err_to_name(ret));
    ret = wm8960_setSYSCLKDIV(&wm8960_device, WM8960_SYSCLK_DIV_BY_2);
    ESP_LOGI(TAG, "  setSYSCLKDIV: %s", esp_err_to_name(ret));
    ret = wm8960_setBCLKDIV(&wm8960_device, 4);
    ESP_LOGI(TAG, "  setBCLKDIV: %s", esp_err_to_name(ret));
    ret = wm8960_setDCLKDIV(&wm8960_device, WM8960_DCLKDIV_16);
    ESP_LOGI(TAG, "  setDCLKDIV: %s", esp_err_to_name(ret));
    ret = wm8960_setADCDIV(&wm8960_device, 0);  // ADD THIS
    ESP_LOGI(TAG, "  setADCDIV: %s", esp_err_to_name(ret));
    ret = wm8960_setDACDIV(&wm8960_device, 0);  // ADD THIS
    ESP_LOGI(TAG, "  setDACDIV: %s", esp_err_to_name(ret));
    


    // ===== CLOCK CONFIGURATION (44.1kHz) =====
    //ret = wm8960_enablePLL(&wm8960_device);
    //ESP_LOGI(TAG, "  enablePLL: %s", esp_err_to_name(ret));
    //wm8960_setPLLPRESCALE(&wm8960_device, WM8960_PLLPRESCALE_DIV_2);
    //wm8960_setSMD(&wm8960_device, WM8960_PLL_MODE_FRACTIONAL);
    //wm8960_setCLKSEL(&wm8960_device, WM8960_CLKSEL_PLL);
    //wm8960_setSYSCLKDIV(&wm8960_device, WM8960_SYSCLK_DIV_BY_2);
    //wm8960_setBCLKDIV(&wm8960_device, 4);
    //wm8960_setDCLKDIV(&wm8960_device, WM8960_DCLKDIV_16);
    //wm8960_setPLLN(&wm8960_device, 7);
    //wm8960_setPLLK(&wm8960_device, 0x86, 0xC2, 0x26);

    
    // Set word length to 16-bit (CRITICAL)
    ret = wm8960_setWL(&wm8960_device, WM8960_WL_16BIT);
    ESP_LOGI(TAG, "  setWL(16bit): %s", esp_err_to_name(ret));
    
    // ===== I2S MODE SELECTION =====
    // WM8960 as SLAVE (peripheral), ESP32 as MASTER
    ret = wm8960_enablePeripheralMode(&wm8960_device);
    ESP_LOGI(TAG, "  enablePeripheralMode: %s", esp_err_to_name(ret));
    
    // Set LRCLK polarity - normal
    ret = wm8960_setLRP(&wm8960_device, 0);  // Normal LRCLK polarity
    ESP_LOGI(TAG, "  setLRP(0 - normal): %s", esp_err_to_name(ret));

    // Set WM8960 FORMAT to 10 (I2S/Philips) - this is actually the WM8960 default
    // FORMAT bits are bits 1:0 of register 0x07 (Audio Interface 1)
    // 00 = Right justified, 01 = Left justified, 10 = I2S (Philips), 11 = DSP/PCM
    ret = wm8960_register_write_multi_bits(&wm8960_device, 0x07, 1, 0, 0x02);  // I2S/Philips
    ESP_LOGI(TAG, "  Set FORMAT to I2S (Philips): %s", esp_err_to_name(ret));
    
    // Don't use ALRCGPIO when in peripheral mode - LRCLK comes from ESP32
    ESP_LOGI(TAG, "  (Skipping ALRCGPIO - peripheral mode, LRC from ESP32)");


    // ===== DAC & ADC ENABLE & VOLUME =====
    ret = wm8960_enableDacLeft(&wm8960_device);
    ESP_LOGI(TAG, "  enableDacLeft: %s", esp_err_to_name(ret));
    ret = wm8960_enableDacRight(&wm8960_device);
    ESP_LOGI(TAG, "  enableDacRight: %s", esp_err_to_name(ret));
    ret = wm8960_enableAdcLeft(&wm8960_device);
    ESP_LOGI(TAG, "  enableAdcLeft: %s", esp_err_to_name(ret));
    ret = wm8960_enableAdcRight(&wm8960_device);
    ESP_LOGI(TAG, "  enableAdcRight: %s", esp_err_to_name(ret));
    
    // Set DAC digital volume (boost for better speaker output)
    ret = wm8960_setDacLeftDigitalVolumeDB(&wm8960_device, 6.0f);
    ESP_LOGI(TAG, "  setDacLeftVol(+6dB): %s", esp_err_to_name(ret));
    ret = wm8960_setDacRightDigitalVolumeDB(&wm8960_device, 6.0f);
    ESP_LOGI(TAG, "  setDacRightVol(+6dB): %s", esp_err_to_name(ret));
    
    // Set ADC digital volume (0dB = unity, no need for 30dB)
    ret = wm8960_setAdcLeftDigitalVolumeDB(&wm8960_device, 0.0f);  // 0dB
    ESP_LOGI(TAG, "  setAdcLeftVol(0dB): %s", esp_err_to_name(ret));
    ret = wm8960_setAdcRightDigitalVolumeDB(&wm8960_device, 0.0f);  // 0dB
    ESP_LOGI(TAG, "  setAdcRightVol(0dB): %s", esp_err_to_name(ret));
    
    // Disable loopback (we're using I2S input, not internal loopback)
    wm8960_disableLoopBack(&wm8960_device);
    
    // CRITICAL: Disable DAC mute (default is soft mute ON)
    ret = wm8960_disableDacMute(&wm8960_device);
    ESP_LOGI(TAG, "  disableDacMute: %s", esp_err_to_name(ret));

    // ===== Ensure Stero Mode =====
    ret = wm8960_setDacMonoMix(&wm8960_device, 0);  // Disable mono mix
    ESP_LOGI(TAG, "  setDacMonoMix(0): %s", esp_err_to_name(ret));
    
    // ===== OUTPUT SELECTION (BOTH HEADPHONES AND SPEAKERS) =====
    // Enable headphones for testing (in case speaker amp has issues)
    ret = wm8960_enableHeadphones(&wm8960_device);
    ESP_LOGI(TAG, "  enableHeadphones: %s", esp_err_to_name(ret));
    ret = wm8960_setHeadphoneVolumeDB(&wm8960_device, 0.0);  // 0dB (max without boost)
    ESP_LOGI(TAG, "  setHeadphoneVolumeDB(0): %s", esp_err_to_name(ret));
    
    ret = wm8960_enableSpeakers(&wm8960_device);
    ESP_LOGI(TAG, "  enableSpeakers: %s", esp_err_to_name(ret));
    ret = wm8960_setSpeakerVolume(&wm8960_device, 115);  // Louder speaker output (0-127 range)
    ESP_LOGI(TAG, "  setSpeakerVolume(115): %s", esp_err_to_name(ret));

    // ==== Speaker Boost Gains =====
    ret = wm8960_setSpeakerDcGain(&wm8960_device, 5);  // 0 to 7
    ret = wm8960_setSpeakerAcGain(&wm8960_device, 5);  // 0 to 7
    ESP_LOGI(TAG, "  setSpeakerDc/AcGain(5): %s", esp_err_to_name(ret));


    // Enable OUT3MIX for VMID buffer (ground reference for speakers/headphones) 
    ret = wm8960_enableOUT3MIX(&wm8960_device);  // VMID buffer for speaker/HP ground
    ESP_LOGI(TAG, "  enableOUT3MIX: %s", esp_err_to_name(ret));
    
    ESP_LOGI(TAG, "WM8960 codec setup complete!");
    return ESP_OK;
}

//=============================================================================
// I2S Initialization
// ESP32 as MASTER (generates BCLK/LRCLK for WM8960)
//=============================================================================

static esp_err_t i2s_init(void)
{
    ESP_LOGI(TAG, "Initializing I2S...");
    ESP_LOGI(TAG, "  BCLK: GPIO%d, LRCLK: GPIO%d, DOUT: GPIO%d",
             I2S_BCLK_PIN, I2S_LRCLK_PIN, I2S_DOUT_PIN);
    
    // ===== GPIO45 DIAGNOSTIC TEST =====
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "===== GPIO45 DIAGNOSTIC TEST =====");
    ESP_LOGI(TAG, "");
    
    int pin = I2S_DOUT_PIN;  // GPIO45
    
    // First, try to completely reset the pin and disconnect from any peripheral
    gpio_reset_pin(pin);
    
    // Try using IO MUX directly
    ESP_LOGI(TAG, "Configuring GPIO%d with direct IO MUX control...", pin);
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT_OUTPUT,  // Try input/output mode
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // Set maximum drive strength
    gpio_set_drive_capability(pin, GPIO_DRIVE_CAP_3);
    
    // Read what drive capability was actually set
    gpio_drive_cap_t actual_cap;
    gpio_get_drive_capability(pin, &actual_cap);
    ESP_LOGI(TAG, "  Requested drive: 3, Actual drive: %d", actual_cap);
    
    // Test HIGH
    gpio_set_level(pin, 1);
    ESP_LOGI(TAG, "  GPIO%d set to HIGH - measure voltage (expect 3.3V)", pin);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Read back the pin state
    int level = gpio_get_level(pin);
    ESP_LOGI(TAG, "  GPIO%d read back: %d (should be 1)", pin, level);
    
    // Test with output only mode
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 1);
    ESP_LOGI(TAG, "  GPIO%d OUTPUT mode, set HIGH - measure again", pin);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    gpio_set_level(pin, 0);
    ESP_LOGI(TAG, "  GPIO%d = LOW", pin);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    gpio_reset_pin(pin);
    
    ESP_LOGI(TAG, "GPIO45 diagnostic complete");
    ESP_LOGI(TAG, "If still 1.2V, GPIO45 may be internally connected to something");
    ESP_LOGI(TAG, "=====================================");
    ESP_LOGI(TAG, "");
    
    // Create I2S channel - ESP32 as MASTER
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 6;
    chan_cfg.dma_frame_num = I2S_BUFFER_SIZE;
    
    if (i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channels!");
        return ESP_FAIL;
    }
    
    // Configure I2S in Philips format (standard I2S)
    // This matches WM8960 FORMAT=10 (I2S/Philips)
    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = I2S_SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,  // 256x is max for new API
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT, 
            I2S_SLOT_MODE_STEREO
        ),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_PIN,
            .ws = I2S_LRCLK_PIN,
            .dout = I2S_DOUT_PIN,
            .din = I2S_DIN_PIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    
    if (i2s_channel_init_std_mode(tx_handle, &std_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2S standard mode!");
        return ESP_FAIL;
    }
    
    if (i2s_channel_init_std_mode(rx_handle, &std_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2S RX standard mode!");
        return ESP_FAIL;
    }
    
    if (i2s_channel_enable(tx_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S TX channel!");
        return ESP_FAIL;
    }
    
    if (i2s_channel_enable(rx_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S RX channel!");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "I2S initialized (MASTER mode, %d Hz, RX+TX)", I2S_SAMPLE_RATE);
    return ESP_OK;
}

//=============================================================================
// Audio Loopback Task - Microphone → Speaker (Real-time Echo Test)
//=============================================================================

static void audio_loopback_task(void *args)
{
    ESP_LOGI(TAG, "Audio loopback task started!");
    
    // Allocate DMA-safe buffer for loopback samples
    int16_t *buffer = (int16_t *)malloc(SINE_SAMPLES * 2 * sizeof(int16_t));
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate loopback buffer!");
        vTaskDelete(NULL);
        return;
    }
    
    size_t bytes_read = 0;
    size_t bytes_written = 0;
    uint32_t loopback_count = 0;
    
    // 60-second timeout for loopback
    TickType_t start_time = xTaskGetTickCount();
    const TickType_t LOOPBACK_DURATION = pdMS_TO_TICKS(60000);  // 60 seconds
    
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "LOOPBACK TEST - MIC → SPEAKER (Echo)");
    ESP_LOGI(TAG, "  Real-time microphone input to speakers");
    ESP_LOGI(TAG, "  Duration: 60 seconds");
    ESP_LOGI(TAG, "  Speak into mic to hear echo");
    ESP_LOGI(TAG, "===========================================");
    
    // Log codec status
    ESP_LOGI(TAG, "Codec Register Check:");
    ESP_LOGI(TAG, "  DAC enabled? PWR2[8]=%d", (wm8960_device.register_local_copy[0x1A] >> 8) & 1);
    ESP_LOGI(TAG, "  ADC L enabled? PWR1[3]=%d", (wm8960_device.register_local_copy[0x19] >> 3) & 1);
    ESP_LOGI(TAG, "  ADC R enabled? PWR1[2]=%d", (wm8960_device.register_local_copy[0x19] >> 2) & 1);
    ESP_LOGI(TAG, "  Speaker enabled? PWR2[4]=%d", (wm8960_device.register_local_copy[0x1A] >> 4) & 1);
    ESP_LOGI(TAG, "  DAC Mute? R5[3]=%d (should be 0)", (wm8960_device.register_local_copy[0x05] >> 3) & 1);
    
    while ((xTaskGetTickCount() - start_time) < LOOPBACK_DURATION) {
        // Read from microphone
        esp_err_t ret = i2s_channel_read(rx_handle, buffer, 
                                         SINE_SAMPLES * 2 * sizeof(int16_t),
                                         &bytes_read, 100);
        
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "RX read error: %d", ret);
            continue;
        }
        
        // Check if we got any audio
        if (loopback_count == 0) {
            ESP_LOGI(TAG, "First RX buffer - first 8 samples: %d,%d,%d,%d,%d,%d,%d,%d",
                     buffer[0], buffer[1], buffer[2], buffer[3], 
                     buffer[4], buffer[5], buffer[6], buffer[7]);
        }
        
        // Write immediately to speaker (real-time loopback)
        ret = i2s_channel_write(tx_handle, buffer, 
                                SINE_SAMPLES * 2 * sizeof(int16_t),
                                &bytes_written, 1000);
        
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "TX write error: %s (wrote %zu of %u bytes)", 
                     esp_err_to_name(ret), bytes_written, SINE_SAMPLES * 2 * sizeof(int16_t));
        } else if (bytes_written != (SINE_SAMPLES * 2 * sizeof(int16_t))) {
            ESP_LOGW(TAG, "TX incomplete write: %zu of %u bytes", 
                     bytes_written, SINE_SAMPLES * 2 * sizeof(int16_t));
        }
        
        loopback_count++;
        
        // Log status every 1 second
        if (loopback_count % 44 == 0) {
            // Analyze audio levels
            int16_t min_val = 32767;
            int16_t max_val = -32768;
            int32_t sum = 0;
            
            for (int i = 0; i < SINE_SAMPLES * 2; i++) {
                if (buffer[i] < min_val) min_val = buffer[i];
                if (buffer[i] > max_val) max_val = buffer[i];
                sum += (int32_t)buffer[i] * buffer[i];
            }
            
            float rms = sqrt((float)sum / (SINE_SAMPLES * 2));
            uint32_t elapsed_ms = (xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS;
            
            ESP_LOGI(TAG, "LOOP #%u (%.1fs): Min=%d, Max=%d, RMS=%.1f (you should hear this)", 
                     loopback_count, elapsed_ms/1000.0f, min_val, max_val, rms);
        }
    }
    
    ESP_LOGI(TAG, "Loopback test timeout reached (60 seconds) - test stopped");
    
    // Stop I2S transmission
    i2s_channel_disable(tx_handle);
    ESP_LOGI(TAG, "I2S channel disabled - audio silenced");
    
    vTaskDelay(pdMS_TO_TICKS(100));
    free(buffer);
    vTaskDelete(NULL);
}

//=============================================================================
// Audio Recording Task - Microphone Input Monitoring
//=============================================================================

static void audio_recording_task(void *args)
{
    ESP_LOGI(TAG, "Audio recording task started!");
    
    // Allocate DMA-safe buffer for recording samples
    int16_t *buffer = (int16_t *)malloc(SINE_SAMPLES * 2 * sizeof(int16_t));
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate recording buffer!");
        vTaskDelete(NULL);
        return;
    }
    
    size_t bytes_read = 0;
    uint32_t read_count = 0;
    
    // 60-second timeout for recording
    TickType_t start_time = xTaskGetTickCount();
    const TickType_t RECORDING_DURATION = pdMS_TO_TICKS(60000);  // 60 seconds
    
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "MICROPHONE INPUT TEST - ADC RECORDING");
    ESP_LOGI(TAG, "  Listening for 60 seconds");
    ESP_LOGI(TAG, "  Speak into microphone to test input");
    ESP_LOGI(TAG, "===========================================");
    
    while ((xTaskGetTickCount() - start_time) < RECORDING_DURATION) {
        // Read from microphone
        esp_err_t ret = i2s_channel_read(rx_handle, buffer, 
                                         SINE_SAMPLES * 2 * sizeof(int16_t),
                                         &bytes_read, 100);
        
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "RX read error: %d", ret);
            continue;
        }
        
        read_count++;
        
        // Analyze audio levels every 44 reads (~1 second at 44.1kHz)
        if (read_count % 44 == 0) {
            int16_t min_val = 32767;
            int16_t max_val = -32768;
            int32_t sum = 0;
            
            // Find min/max and calculate RMS
            for (int i = 0; i < SINE_SAMPLES * 2; i++) {
                if (buffer[i] < min_val) min_val = buffer[i];
                if (buffer[i] > max_val) max_val = buffer[i];
                sum += (int32_t)buffer[i] * buffer[i];
            }
            
            float rms = sqrt((float)sum / (SINE_SAMPLES * 2));
            uint32_t elapsed_ms = (xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS;
            
            ESP_LOGI(TAG, "RX #%u (%.1fs): Min=%d, Max=%d, RMS=%.1f, bytes=%u", 
                     read_count, elapsed_ms/1000.0f, min_val, max_val, rms, bytes_read);
        }
    }
    
    ESP_LOGI(TAG, "Audio recording timeout reached (60 seconds) - recording stopped");
    free(buffer);
    vTaskDelete(NULL);
}

//=============================================================================
// Audio Playback Task - Sine Wave Generator
//=============================================================================

static void audio_playback_task(void *args)
{
    ESP_LOGI(TAG, "Audio playback task started!");
    
    // Allocate DMA-safe buffer for audio samples
    int16_t *buffer = (int16_t *)malloc(SINE_SAMPLES * 2 * sizeof(int16_t));
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate audio buffer!");
        vTaskDelete(NULL);
        return;
    }
    
    size_t bytes_written = 0;
    uint32_t write_count = 0;
    
    // 60-second timeout for audio playback
    TickType_t start_time = xTaskGetTickCount();
    const TickType_t PLAYBACK_DURATION = pdMS_TO_TICKS(60000);  // 60 seconds
    
    // Square wave test - LOUD and clear
    const int16_t HIGH_VAL = 25000;   // ~75% of max
    const int16_t LOW_VAL = -25000;
    const int samples_per_half_cycle = I2S_SAMPLE_RATE / (2 * 1000);  // 1kHz
    
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "SQUARE WAVE TEST - DAC ONLY (no ADC)");
    ESP_LOGI(TAG, "  ESP32 MASTER, WM8960 PERIPHERAL");
    ESP_LOGI(TAG, "  ALRCGPIO enabled to output LRC on ADCLRC");
    ESP_LOGI(TAG, "  PLAYBACK DURATION: 60 seconds");
    ESP_LOGI(TAG, "===========================================");
    
    int sample_counter = 0;
    bool high_phase = true;
    
    while ((xTaskGetTickCount() - start_time) < PLAYBACK_DURATION) {
        // Generate square wave
        for (int i = 0; i < SINE_SAMPLES; i++) {
            int16_t val = high_phase ? HIGH_VAL : LOW_VAL;
            buffer[i * 2] = val;      // Left
            buffer[i * 2 + 1] = val;  // Right
            
            sample_counter++;
            if (sample_counter >= samples_per_half_cycle) {
                sample_counter = 0;
                high_phase = !high_phase;
            }
        }
        
        (void)i2s_channel_write(tx_handle, buffer, 
                                SINE_SAMPLES * 2 * sizeof(int16_t),
                                &bytes_written, 1000);
        write_count++;
        
        if (write_count % 200 == 0) {
            uint32_t elapsed_ms = (xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS;
            ESP_LOGI(TAG, "TX #%u (%.1fs): wrote=%u, sample=%d", write_count, elapsed_ms/1000.0f, bytes_written, buffer[0]);
        }
    }
    
    ESP_LOGI(TAG, "Audio playback timeout reached (60 seconds) - stopping playback");
    
    // Stop I2S transmission to flush DMA buffers and silence output
    i2s_channel_disable(tx_handle);
    ESP_LOGI(TAG, "I2S channel disabled - audio silenced");
    
    vTaskDelay(pdMS_TO_TICKS(100));  // Give DMA time to finish
    
    free(buffer);
    vTaskDelete(NULL);
}

//=============================================================================
// Comprehensive Register Dump (Debugging Utility)
//=============================================================================

static void dump_wm8960_registers(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========== WM8960 FULL REGISTER DUMP ==========");
    
    // Power Management
    ESP_LOGI(TAG, "--- POWER MANAGEMENT ---");
    ESP_LOGI(TAG, "  R25 (0x19) PWR_MGMT_1: 0x%03X", wm8960_device.register_local_copy[0x19]);
    ESP_LOGI(TAG, "    VMIDSEL=%d, VREF=%d, AINL=%d, AINR=%d, ADCL=%d, ADCR=%d, MICB=%d, DIGENB=%d",
        (wm8960_device.register_local_copy[0x19] >> 7) & 0x3,
        (wm8960_device.register_local_copy[0x19] >> 6) & 1,
        (wm8960_device.register_local_copy[0x19] >> 5) & 1,
        (wm8960_device.register_local_copy[0x19] >> 4) & 1,
        (wm8960_device.register_local_copy[0x19] >> 3) & 1,
        (wm8960_device.register_local_copy[0x19] >> 2) & 1,
        (wm8960_device.register_local_copy[0x19] >> 1) & 1,
        wm8960_device.register_local_copy[0x19] & 1);
    
    ESP_LOGI(TAG, "  R26 (0x1A) PWR_MGMT_2: 0x%03X", wm8960_device.register_local_copy[0x1A]);
    ESP_LOGI(TAG, "    DACL=%d, DACR=%d, LOUT1=%d, ROUT1=%d, SPKL=%d, SPKR=%d, OUT3=%d, PLL_EN=%d",
        (wm8960_device.register_local_copy[0x1A] >> 8) & 1,
        (wm8960_device.register_local_copy[0x1A] >> 7) & 1,
        (wm8960_device.register_local_copy[0x1A] >> 6) & 1,
        (wm8960_device.register_local_copy[0x1A] >> 5) & 1,
        (wm8960_device.register_local_copy[0x1A] >> 4) & 1,
        (wm8960_device.register_local_copy[0x1A] >> 3) & 1,
        (wm8960_device.register_local_copy[0x1A] >> 1) & 1,
        wm8960_device.register_local_copy[0x1A] & 1);
    
    ESP_LOGI(TAG, "  R47 (0x2F) PWR_MGMT_3: 0x%03X", wm8960_device.register_local_copy[0x2F]);
    ESP_LOGI(TAG, "    LMIC=%d, RMIC=%d, LOMIX=%d, ROMIX=%d",
        (wm8960_device.register_local_copy[0x2F] >> 5) & 1,
        (wm8960_device.register_local_copy[0x2F] >> 4) & 1,
        (wm8960_device.register_local_copy[0x2F] >> 3) & 1,
        (wm8960_device.register_local_copy[0x2F] >> 2) & 1);
    
    // Audio Interface
    ESP_LOGI(TAG, "--- AUDIO INTERFACE ---");
    ESP_LOGI(TAG, "  R7  (0x07) AUDIO_IFACE_1: 0x%03X", wm8960_device.register_local_copy[0x07]);
    ESP_LOGI(TAG, "    BCLKINV=%d, MS=%d, LRSWAP=%d, LRP=%d, WL=%d, FORMAT=%d",
        (wm8960_device.register_local_copy[0x07] >> 7) & 1,
        (wm8960_device.register_local_copy[0x07] >> 6) & 1,
        (wm8960_device.register_local_copy[0x07] >> 5) & 1,
        (wm8960_device.register_local_copy[0x07] >> 4) & 1,
        (wm8960_device.register_local_copy[0x07] >> 2) & 3,
        wm8960_device.register_local_copy[0x07] & 3);
    ESP_LOGI(TAG, "    FORMAT: 00=RightJ, 01=LeftJ, 10=I2S, 11=DSP");
    
    ESP_LOGI(TAG, "  R9  (0x09) AUDIO_IFACE_2: 0x%03X", wm8960_device.register_local_copy[0x09]);
    ESP_LOGI(TAG, "    ALRCGPIO=%d, WL8=%d, DACPOL=%d, ADCPOL=%d, DACSLOPE=%d, LOOPBACK=%d",
        (wm8960_device.register_local_copy[0x09] >> 6) & 1,
        (wm8960_device.register_local_copy[0x09] >> 5) & 1,
        (wm8960_device.register_local_copy[0x09] >> 4) & 1,
        (wm8960_device.register_local_copy[0x09] >> 3) & 1,
        (wm8960_device.register_local_copy[0x09] >> 1) & 1,
        wm8960_device.register_local_copy[0x09] & 1);
    
    // DAC Control
    ESP_LOGI(TAG, "--- DAC CONTROL ---");
    ESP_LOGI(TAG, "  R5  (0x05) ADC_DAC_CTRL_1: 0x%03X", wm8960_device.register_local_copy[0x05]);
    ESP_LOGI(TAG, "    DACDIV2=%d, ADCPOL=%d, DACMU=%d, DEEMPH=%d, ADCHPD=%d",
        (wm8960_device.register_local_copy[0x05] >> 7) & 1,
        (wm8960_device.register_local_copy[0x05] >> 5) & 3,
        (wm8960_device.register_local_copy[0x05] >> 3) & 1,
        (wm8960_device.register_local_copy[0x05] >> 1) & 3,
        wm8960_device.register_local_copy[0x05] & 1);
    ESP_LOGI(TAG, "    *** DACMU (bit3) must be 0 for audio! ***");
    
    ESP_LOGI(TAG, "  R10 (0x0A) L DAC VOL: 0x%03X (0xFF = 0dB)", wm8960_device.register_local_copy[0x0A]);
    ESP_LOGI(TAG, "  R11 (0x0B) R DAC VOL: 0x%03X (0xFF = 0dB)", wm8960_device.register_local_copy[0x0B]);
    
    // Output Mixer
    ESP_LOGI(TAG, "--- OUTPUT MIXER ---");
    ESP_LOGI(TAG, "  R34 (0x22) L OUT MIX: 0x%03X", wm8960_device.register_local_copy[0x22]);
    ESP_LOGI(TAG, "    LD2LO=%d (DAC to output), LI2LO=%d, LB2LO=%d",
        (wm8960_device.register_local_copy[0x22] >> 8) & 1,
        (wm8960_device.register_local_copy[0x22] >> 7) & 1,
        (wm8960_device.register_local_copy[0x22] >> 4) & 1);
    
    ESP_LOGI(TAG, "  R37 (0x25) R OUT MIX: 0x%03X", wm8960_device.register_local_copy[0x25]);
    ESP_LOGI(TAG, "    RD2RO=%d (DAC to output), RI2RO=%d, RB2RO=%d",
        (wm8960_device.register_local_copy[0x25] >> 8) & 1,
        (wm8960_device.register_local_copy[0x25] >> 7) & 1,
        (wm8960_device.register_local_copy[0x25] >> 4) & 1);
    
    // Speaker Control
    ESP_LOGI(TAG, "--- SPEAKER CONTROL ---");
    ESP_LOGI(TAG, "  R40 (0x28) L SPK VOL: 0x%03X (0x7F = max)", wm8960_device.register_local_copy[0x28]);
    ESP_LOGI(TAG, "  R41 (0x29) R SPK VOL: 0x%03X (0x7F = max)", wm8960_device.register_local_copy[0x29]);
    ESP_LOGI(TAG, "  R49 (0x31) CLASS D CTRL 1: 0x%03X", wm8960_device.register_local_copy[0x31]);
    ESP_LOGI(TAG, "    SPK_OP_EN: L=%d, R=%d",
        (wm8960_device.register_local_copy[0x31] >> 7) & 1,
        (wm8960_device.register_local_copy[0x31] >> 6) & 1);
    
    // Clock / PLL
    ESP_LOGI(TAG, "--- CLOCK / PLL ---");
    ESP_LOGI(TAG, "  R4  (0x04) CLOCKING_1: 0x%03X", wm8960_device.register_local_copy[0x04]);
    ESP_LOGI(TAG, "    CLKSEL=%d (0=MCLK, 1=PLL), SYSCLKDIV=%d, DACDIV=%d, ADCDIV=%d",
        wm8960_device.register_local_copy[0x04] & 1,
        (wm8960_device.register_local_copy[0x04] >> 1) & 3,
        (wm8960_device.register_local_copy[0x04] >> 3) & 7,
        (wm8960_device.register_local_copy[0x04] >> 6) & 7);
    
    ESP_LOGI(TAG, "  R8  (0x08) CLOCKING_2: 0x%03X", wm8960_device.register_local_copy[0x08]);
    ESP_LOGI(TAG, "    BCLKDIV=%d, DCLKDIV=%d",
        wm8960_device.register_local_copy[0x08] & 0xF,
        (wm8960_device.register_local_copy[0x08] >> 6) & 7);
    
    ESP_LOGI(TAG, "  R52 (0x34) PLL_N: 0x%03X (PLLPRESCALE=%d, SDM=%d, N=%d)",
        wm8960_device.register_local_copy[0x34],
        (wm8960_device.register_local_copy[0x34] >> 4) & 1,
        (wm8960_device.register_local_copy[0x34] >> 5) & 1,
        wm8960_device.register_local_copy[0x34] & 0xF);
    
    ESP_LOGI(TAG, "  R53-55 PLL_K: 0x%02X 0x%02X 0x%02X",
        wm8960_device.register_local_copy[0x35] & 0xFF,
        wm8960_device.register_local_copy[0x36] & 0xFF,
        wm8960_device.register_local_copy[0x37] & 0xFF);
    
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "");
}

//=============================================================================
// Main Application
//=============================================================================

void app_main(void)
{


        // DIAGNOSTIC: Test GPIO6 manually BEFORE any initialization
    ESP_LOGI(TAG, "=== GPIO6 Diagnostic Test (Before I2C) ===");
    
    // Configure GPIO6 as simple GPIO output (not peripheral function)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << 6),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_drive_capability(6, GPIO_DRIVE_CAP_3);  // Max drive strength
    
    gpio_set_level(6, 1);
    ESP_LOGI(TAG, "GPIO6 set HIGH - measure voltage NOW (should be 3.3V)");
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    gpio_set_level(6, 0);
    ESP_LOGI(TAG, "GPIO6 set LOW - measure voltage NOW (should be 0V)");
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    gpio_set_level(6, 1);
    ESP_LOGI(TAG, "GPIO6 set HIGH again");
    ESP_LOGI(TAG, "=== End GPIO6 Test ===");
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "WM8960 ESP32-P4 I2S DAC Test");
    ESP_LOGI(TAG, "=================================");
    
    
    // Step 1: Initialize I2C (for codec control)
    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed!");
        return;
    }
    
    // Step 2: Initialize WM8960 codec
    if (wm8960_codec_init() != ESP_OK) {
        ESP_LOGE(TAG, "WM8960 initialization failed!");
        return;
    }
    
    // Step 3: Initialize I2S (audio data path)
    if (i2s_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2S initialization failed!");
        return;
    }
    
    // Step 4: Wait for codec to stabilize
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Debug: Dump all WM8960 registers for verification
    dump_wm8960_registers();
    
    // Step 5: Start audio loopback task (microphone → speaker)
    if (xTaskCreate(audio_loopback_task, "audio_loopback", 8192, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio loopback task!");
        return;
    }
    
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "Setup complete!");
    ESP_LOGI(TAG, "Microphone Loopback Test");
    ESP_LOGI(TAG, "Duration: 60 seconds");
    ESP_LOGI(TAG, "=================================");
    
    // Main task keeps running (loopback task handles the work)
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
