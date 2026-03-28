#pragma once

#include <cstdint>
#include <cstring>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_err.h"

struct PIDGains {
    float kp;
    float ki;
    float kd;
};

struct RcChannelCalib {
    uint16_t minPWM;
    uint16_t maxPWM;
    bool reversed;
};

struct ParameterBlock {
    uint32_t version;
    bool valid;

    PIDGains roll_rate;
    PIDGains pitch_rate;
    PIDGains yaw_rate;

    RcChannelCalib rc[5];

    float accel_bias1[3];
	float accel_bias2[3];
	float accel_scales1[3];
	float accel_scales2[3];
    float gyro_bias1[3];
	float gyro_bias2[3];
	
	float mag_bias[3];
	float mag_scales[3];
	

    uint16_t esc_min_us;
    uint16_t esc_max_us;
    uint16_t esc_spin_min_us;
};

class ParameterStore {
public:
    static constexpr const char* NAMESPACE_NAME = "autopilot";
    static constexpr const char* BLOB_KEY = "params";
    static constexpr uint32_t PARAM_VERSION = 1;

    ParameterStore() = default;

    esp_err_t init()
    {
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }
        return err;
    }

    esp_err_t save(const ParameterBlock& params)
    {
        nvs_handle_t handle;
        esp_err_t err = nvs_open(NAMESPACE_NAME, NVS_READWRITE, &handle);
        if (err != ESP_OK) return err;

        err = nvs_set_blob(handle, BLOB_KEY, &params, sizeof(params));
        if (err == ESP_OK) {
            err = nvs_commit(handle);
        }

        nvs_close(handle);
        return err;
    }

    esp_err_t load(ParameterBlock& params)
    {
        nvs_handle_t handle;
        esp_err_t err = nvs_open(NAMESPACE_NAME, NVS_READONLY, &handle);
        if (err != ESP_OK) return err;

        size_t required_size = sizeof(params);
        err = nvs_get_blob(handle, BLOB_KEY, &params, &required_size);
        nvs_close(handle);

        if (err != ESP_OK) return err;
        if (required_size != sizeof(params)) return ESP_ERR_INVALID_SIZE;
        if (!params.valid) return ESP_ERR_NOT_FOUND;
        if (params.version != PARAM_VERSION) return ESP_ERR_INVALID_VERSION;

        return ESP_OK;
    }

    esp_err_t erase()
    {
        nvs_handle_t handle;
        esp_err_t err = nvs_open(NAMESPACE_NAME, NVS_READWRITE, &handle);
        if (err != ESP_OK) return err;

        err = nvs_erase_key(handle, BLOB_KEY);
        if (err == ESP_OK) {
            err = nvs_commit(handle);
        }

        nvs_close(handle);
        return err;
    }

    static ParameterBlock make_defaults()
    {
        ParameterBlock p{};
        p.version = PARAM_VERSION;
        p.valid = true;

        p.roll_rate  = {0.10f, 0.00f, 0.002f};
        p.pitch_rate = {0.10f, 0.00f, 0.002f};
        p.yaw_rate   = {0.10f, 0.00f, 0.000f};

        for (int i = 0; i < 5; ++i) {
            p.rc[i].minPWM = 1000;
            p.rc[i].maxPWM = 2000;
            p.rc[i].reversed = false;
        }

        p.accel_bias1[0] = p.accel_bias1[1] = p.accel_bias1[2] = 0.0f;
		p.accel_bias2[0] = p.accel_bias2[1] = p.accel_bias2[2] = 0.0f;
		p.accel_scales1[0] = p.accel_scales1[1] = p.accel_scales1[2] = 1.0f;
		p.accel_scales2[0] = p.accel_scales2[1] = p.accel_scales2[2] = 1.0f;
		p.gyro_bias1[0]  = p.gyro_bias1[1]  = p.gyro_bias1[2]  = 0.0f;
		p.gyro_bias2[0]  = p.gyro_bias2[1]  = p.gyro_bias2[2]  = 0.0f;
		
		p.mag_bias[0]  = p.mag_bias[1]  = p.mag_bias[2]  = 0.0f;
		p.mag_scales[0] = p.mag_scales[1] = p.mag_scales[2] = 1.0f;
		
        p.esc_min_us = 1000;
        p.esc_max_us = 2000;
        p.esc_spin_min_us = 1070;

        return p;
    }
};