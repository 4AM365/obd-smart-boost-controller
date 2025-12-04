/*
 * Modular Boost Controller Firmware for STM32F103C8T6
 * PWM output controls wastegate solenoid based on multiple inputs
 */

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// CONFIGURATION
// ============================================================================
#define MAX_BOOST_PSI           25.0f
#define MIN_BOOST_PSI           3.0f
#define PWM_FREQUENCY_HZ        30
#define CAN_TIMEOUT_MS          500
#define ADC_SAMPLES             10

// Gear ratios for inference (adjust for your vehicle)
const float GEAR_RATIOS[] = {3.5f, 2.0f, 1.4f, 1.0f, 0.8f, 0.7f};
#define NUM_GEARS 6

// ============================================================================
// DATA STRUCTURES
// ============================================================================
typedef struct {
    float boost_psi;
    bool valid;
    uint32_t last_update_ms;
} BoostPressure_t;

typedef struct {
    float max_boost_psi;
    bool valid;
} TrimPot_t;

typedef struct {
    uint16_t vss_kph;
    uint16_t rpm;
    uint8_t tps_percent;
    bool valid;
    uint32_t last_update_ms;
} CANData_t;

typedef struct {
    uint8_t current_gear;
    bool valid;
} GearState_t;

typedef struct {
    float target_boost_psi;
    float duty_cycle_percent;
} BoostTarget_t;

// ============================================================================
// GLOBAL STATE
// ============================================================================
static BoostPressure_t g_boost_pressure;
static TrimPot_t g_trim_pot;
static CANData_t g_can_data;
static GearState_t g_gear;
static BoostTarget_t g_boost_target;

// HAL handles (initialized in main)
static ADC_HandleTypeDef hadc1;
static CAN_HandleTypeDef hcan;
static TIM_HandleTypeDef htim2;

// ============================================================================
// BLOCK 1: INPUT READING
// ============================================================================

void ReadTrimPot(void) {
    uint32_t adc_sum = 0;
    
    // Oversample ADC for stability
    for (int i = 0; i < ADC_SAMPLES; i++) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 10);
        adc_sum += HAL_ADC_GetValue(&hadc1);
    }
    
    uint16_t adc_avg = adc_sum / ADC_SAMPLES;
    
    // Map 0-4095 ADC to MIN_BOOST_PSI to MAX_BOOST_PSI
    g_trim_pot.max_boost_psi = MIN_BOOST_PSI + 
        ((float)adc_avg / 4095.0f) * (MAX_BOOST_PSI - MIN_BOOST_PSI);
    g_trim_pot.valid = true;
}

void ReadPressureSensor(void) {
    uint32_t adc_sum = 0;
    
    // Select pressure sensor ADC channel (configure in CubeMX)
    for (int i = 0; i < ADC_SAMPLES; i++) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 10);
        adc_sum += HAL_ADC_GetValue(&hadc1);
    }
    
    uint16_t adc_avg = adc_sum / ADC_SAMPLES;
    
    // Convert to PSI (assuming 0.5-4.5V = 0-30 PSI sensor)
    float voltage = (adc_avg / 4095.0f) * 3.3f;
    g_pressure.boost_psi = ((voltage - 0.5f) / 4.0f) * 30.0f;
    g_pressure.valid = true;
}

void ReadCANData(void) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    
    // Check if CAN data is stale
    if ((HAL_GetTick() - g_can_data.last_update_ms) > CAN_TIMEOUT_MS) {
        g_can_data.valid = false;
    }
    
    // Check if boost pressure data is stale
    if ((HAL_GetTick() - g_boost_pressure.last_update_ms) > CAN_TIMEOUT_MS) {
        g_boost_pressure.valid = false;
    }
    
    // Parse J1939 messages (adjust PGNs for your ECU)
    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        uint32_t pgn = (rx_header.ExtId >> 8) & 0x3FFFF;
        
        // PGN 61444: Electronic Engine Controller 1 (RPM, TPS)
        if (pgn == 61444) {
            g_can_data.rpm = (rx_data[4] << 8 | rx_data[3]) * 0.125f;
            g_can_data.tps_percent = rx_data[6] * 0.4f;
            g_can_data.last_update_ms = HAL_GetTick();
            g_can_data.valid = true;
        }
        
        // PGN 65265: Cruise Control/Vehicle Speed
        if (pgn == 65265) {
            g_can_data.vss_kph = (rx_data[2] << 8 | rx_data[1]) * 0.00390625f;
            g_can_data.last_update_ms = HAL_GetTick();
            g_can_data.valid = true;
        }
        
        // PGN 65270: Intake Manifold 1 (Boost Pressure)
        // Byte 2: Intake manifold 1 pressure (0-250 kPa at 2 kPa/bit)
        if (pgn == 65270) {
            float boost_kpa = rx_data[2] * 2.0f;
            
            // Convert kPa to PSI (subtract atmospheric ~101 kPa for gauge pressure)
            g_boost_pressure.boost_psi = (boost_kpa - 101.325f) * 0.145038f;
            
            g_boost_pressure.last_update_ms = HAL_GetTick();
            g_boost_pressure.valid = true;
        }
    }
}

// ============================================================================
// BLOCK 2: GEAR INFERENCE
// ============================================================================

void InferGear(void) {
    if (!g_can_data.valid || g_can_data.vss_kph < 5 || g_can_data.rpm < 500) {
        g_gear.valid = false;
        return;
    }
    
    // Calculate speed/RPM ratio
    float ratio = (float)g_can_data.vss_kph / (float)g_can_data.rpm;
    
    // Find closest gear ratio
    float min_diff = 999.9f;
    uint8_t best_gear = 1;
    
    for (int i = 0; i < NUM_GEARS; i++) {
        float diff = fabsf(ratio - GEAR_RATIOS[i]);
        if (diff < min_diff) {
            min_diff = diff;
            best_gear = i + 1;
        }
    }
    
    g_gear.current_gear = best_gear;
    g_gear.valid = true;
}

// ============================================================================
// BLOCK 3: BOOST TARGET CALCULATION
// ============================================================================

void CalculateBoostTarget(void) {
    if (!g_trim_pot.valid || !g_can_data.valid || !g_gear.valid) {
        g_boost_target.target_boost_psi = MIN_BOOST_PSI;
        return;
    }
    
    // Start with max boost from trim pot
    float target = g_trim_pot.max_boost_psi;
    
    // Scale by TPS (0-100%)
    float tps_scale = g_can_data.tps_percent / 100.0f;
    target *= tps_scale;
    
    // Scale by gear (lower gears = reduced boost)
    float gear_scale = 1.0f;
    if (g_gear.current_gear == 1) gear_scale = 0.6f;
    else if (g_gear.current_gear == 2) gear_scale = 0.8f;
    else gear_scale = 1.0f;
    
    target *= gear_scale;
    
    // Clamp to valid range
    if (target < MIN_BOOST_PSI) target = MIN_BOOST_PSI;
    if (target > MAX_BOOST_PSI) target = MAX_BOOST_PSI;
    
    g_boost_target.target_boost_psi = target;
}

// ============================================================================
// BLOCK 4: PWM DUTY CYCLE CONTROL
// ============================================================================

void CalculatePWMDutyCycle(void) {
    if (!g_pressure.valid) {
        g_boost_target.duty_cycle_percent = 0.0f;
        return;
    }
    
    // Simple proportional control
    float error = g_boost_target.target_boost_psi - g_pressure.boost_psi;
    float gain = 5.0f; // Adjust for your system
    
    float duty = 50.0f + (error * gain); // Start at 50% baseline
    
    // Clamp duty cycle to 0-100%
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 100.0f) duty = 100.0f;
    
    g_boost_target.duty_cycle_percent = duty;
}

void SetPWMOutput(void) {
    // Calculate CCR value for duty cycle
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
    uint32_t ccr = (uint32_t)((g_boost_target.duty_cycle_percent / 100.0f) * arr);
    
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);
}

// ============================================================================
// MAIN CONTROL LOOP
// ============================================================================

void BoostControlLoop(void) {
    // Block 1: Read all inputs
    ReadTrimPot();
    ReadPressureSensor();
    ReadCANData();
    
    // Block 2: Infer current gear
    InferGear();
    
    // Block 3: Calculate target boost
    CalculateBoostTarget();
    
    // Block 4: Calculate and set PWM output
    CalculatePWMDutyCycle();
    SetPWMOutput();
}

// ============================================================================
// INITIALIZATION AND MAIN
// ============================================================================

void SystemInit_BoostController(void) {
    // Initialize peripherals (CAN, ADC, PWM Timer)
    // This would be done in main() with CubeMX-generated code
    
    // Start PWM output
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    
    // Configure CAN filters for J1939
    // Start CAN
    HAL_CAN_Start(&hcan);
    
    // Initialize state
    g_can_data.valid = false;
    g_gear.valid = false;
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    
    SystemInit_BoostController();
    
    while (1) {
        BoostControlLoop();
        HAL_Delay(20); // 50Hz control loop
    }
}