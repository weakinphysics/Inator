#include "esc_interface.h"
#include "esp_check.h"

esp_err_t ESCInterface::init(const std::array<gpio_num_t, NUM_ESCS>& pins)
{
    esp_err_t err;

    mcpwm_timer_config_t timer_cfg = {};
    timer_cfg.group_id = 0;
    timer_cfg.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_cfg.resolution_hz = RESOLUTION_HZ;
    timer_cfg.period_ticks = PERIOD_US;
    timer_cfg.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&timer_cfg, &timer_), "ESC", "new timer failed");

    for (int i = 0; i < NUM_ESCS; ++i) {
        mcpwm_operator_config_t oper_cfg = {};
        oper_cfg.group_id = 0;
        ESP_RETURN_ON_ERROR(mcpwm_new_operator(&oper_cfg, &opers_[i]), "ESC", "new operator failed");

        ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(opers_[i], timer_), "ESC", "connect timer failed");

        mcpwm_comparator_config_t cmpr_cfg = {};
        cmpr_cfg.flags.update_cmp_on_tez = true; // latch cleanly at start of next period
        ESP_RETURN_ON_ERROR(mcpwm_new_comparator(opers_[i], &cmpr_cfg, &cmprs_[i]), "ESC", "new comparator failed");

        mcpwm_generator_config_t gen_cfg = {};
        gen_cfg.gen_gpio_num = pins[i];
        ESP_RETURN_ON_ERROR(mcpwm_new_generator(opers_[i], &gen_cfg, &gens_[i]), "ESC", "new generator failed");

        ESP_RETURN_ON_ERROR(
            mcpwm_comparator_set_compare_value(cmprs_[i], MIN_US),
            "ESC", "set initial compare failed"
        );

        // HIGH at start of period
        ESP_RETURN_ON_ERROR(
            mcpwm_generator_set_action_on_timer_event(
                gens_[i],
                MCPWM_GEN_TIMER_EVENT_ACTION(
                    MCPWM_TIMER_DIRECTION_UP,
                    MCPWM_TIMER_EVENT_EMPTY,
                    MCPWM_GEN_ACTION_HIGH
                )
            ),
            "ESC", "set timer action failed"
        );

        // LOW when timer hits compare
        ESP_RETURN_ON_ERROR(
            mcpwm_generator_set_action_on_compare_event(
                gens_[i],
                MCPWM_GEN_COMPARE_EVENT_ACTION(
                    MCPWM_TIMER_DIRECTION_UP,
                    cmprs_[i],
                    MCPWM_GEN_ACTION_LOW
                )
            ),
            "ESC", "set compare action failed"
        );
    }

    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(timer_), "ESC", "timer enable failed");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(timer_, MCPWM_TIMER_START_NO_STOP), "ESC", "timer start failed");

    initialized_ = true;
    return ESP_OK;
}

esp_err_t ESCInterface::write_us(int idx, uint32_t pulse_us)
{
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    if (idx < 0 || idx >= NUM_ESCS) return ESP_ERR_INVALID_ARG;
    return mcpwm_comparator_set_compare_value(cmprs_[idx], clamp_us(pulse_us));
}

esp_err_t ESCInterface::write_all_us(const std::array<uint32_t, NUM_ESCS>& pulse_us)
{
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    for (int i = 0; i < NUM_ESCS; ++i) {
        ESP_RETURN_ON_ERROR(
            mcpwm_comparator_set_compare_value(cmprs_[i], clamp_us(pulse_us[i])),
            "ESC", "write_all failed"
        );
    }
    return ESP_OK;
}

esp_err_t ESCInterface::write_throttle(int idx, float t)
{
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    uint32_t us = MIN_US + static_cast<uint32_t>(t * float(MAX_US - MIN_US));
    return write_us(idx, us);
}

esp_err_t ESCInterface::arm_all()
{
    return write_all_us({SPIN_US, SPIN_US, SPIN_US, SPIN_US});
}

esp_err_t ESCInterface::disarm_all()
{
    return write_all_us({MIN_US, MIN_US, MIN_US, MIN_US});
}