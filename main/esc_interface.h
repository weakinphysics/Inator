#pragma once

#include <array>
#include <cstdint>
#include "driver/mcpwm_prelude.h"
#include "esp_err.h"
#include "driver/gpio.h"

class ESCInterface {
public:
    static constexpr int NUM_ESCS = 4;
    static constexpr uint32_t RESOLUTION_HZ = 1000000; // 1 tick = 1 us
    static constexpr uint32_t PERIOD_US = 20000;       // 50 Hz
    static constexpr uint32_t MIN_US = 1000;
    static constexpr uint32_t MAX_US = 2000;
	static constexpr uint32_t SPIN_US = 1500;

    esp_err_t init(const std::array<gpio_num_t, NUM_ESCS>& pins);
    esp_err_t write_us(int idx, uint32_t pulse_us);
    esp_err_t write_all_us(const std::array<uint32_t, NUM_ESCS>& pulse_us);
    esp_err_t write_throttle(int idx, float t);
    esp_err_t arm_all();
    esp_err_t disarm_all();

private:
    mcpwm_timer_handle_t timer_ = nullptr;
    std::array<mcpwm_oper_handle_t, NUM_ESCS> opers_{};
    std::array<mcpwm_cmpr_handle_t, NUM_ESCS> cmprs_{};
    std::array<mcpwm_gen_handle_t, NUM_ESCS> gens_{};
    bool initialized_ = false;

    static uint32_t clamp_us(uint32_t us) {
        if (us < MIN_US) return MIN_US;
        if (us > MAX_US) return MAX_US;
        return us;
    }
};