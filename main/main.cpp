#include <cstdint>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

#include "hal/i2c_types.h"
#include "hal/ledc_types.h"
//#include "icm20948_enumerations.h"
#include "esp_check.h"
#include <math.h>
#include <array>
#include <algorithm>
#include <cmath>
// CRSF 

#include "crsf_driver.h"


// ESC MOTOR COMMS 

#include "esc_interface.h"


// FLASH INTERFACE
#include "soc/gpio_num.h"
#include "store_config.h"


#include <system_error>

#include <functional>



#define I2C_MASTER_SCL_IO   22
#define I2C_MASTER_SDA_IO   21
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000	



static constexpr auto i2c_port = I2C_NUM_0;
static constexpr auto i2c_clock_speed = I2C_MASTER_FREQ_HZ;
static constexpr gpio_num_t i2c_scl = (gpio_num_t)I2C_MASTER_SCL_IO;
static constexpr gpio_num_t i2c_sda = (gpio_num_t)I2C_MASTER_SDA_IO;
static constexpr uint8_t imu_addr = 0x69; 


///// SCALES FOR ACCELS AND GYROS, COMMON TO BOTH IMUs

float a2 = 16384.0f;
float a4 = 8192.0f;
float a8 = 4096.0f;
float a16 = 2048.0f;
		
float g250 = 131.0f;
float g500 = 65.5f;
float g1000 = 32.8f;
float g2000 = 16.4f;

double PI = 3.1415926535897932384626433832;

//////////////////////////////////////////////////////


//
static esp_err_t icm20948_whoami_test_newapi() {
    const uint8_t addr = 0x69;
    const uint8_t reg  = 0x00;   // WHO_AM_I, bank 0
    uint8_t who = 0;

    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = GPIO_NUM_21;
    bus_cfg.scl_io_num = GPIO_NUM_22;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t bus_handle = nullptr;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = addr;
    dev_cfg.scl_speed_hz = 100000;

    i2c_master_dev_handle_t dev_handle = nullptr;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    esp_err_t err = i2c_master_transmit_receive(
        dev_handle,
        &reg, 1,
        &who, 1,
        100 / portTICK_PERIOD_MS
    );

    printf("combined read err=%s, WHO_AM_I=0x%02X\n", esp_err_to_name(err), who);

    return err;
}

//static void i2c_master_init()
//{
//    i2c_config_t conf = {};
//    conf.mode = I2C_MODE_MASTER;
//    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
//    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
//    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
//    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
//    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
//	conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
//    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
//    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
//}

double clip(double input, double left,  double right){
	input = std::max(input, left);
	input = std::min(input, right);
	return input;
}




class ICM20948 {
public:
    struct ImuData {
        float ax_g, ay_g, az_g;
        float gx_dps, gy_dps, gz_dps;
        float mx_uT, my_uT, mz_uT;
        bool mag_valid;
    };

    explicit ICM20948(i2c_master_dev_handle_t dev) : dev_(dev) {}

    esp_err_t init() {
        uint8_t who = 0;
        ESP_RETURN_ON_ERROR(read_reg(REG_BANK0, REG_WHO_AM_I, &who, 1), TAG, "WHO_AM_I read failed");
        ESP_LOGI(TAG, "ICM20948 WHO_AM_I = 0x%02X", who);
        if (who != 0xEA) {
            ESP_LOGE(TAG, "Unexpected WHO_AM_I");
            return ESP_FAIL;
        }

        // device reset
        ESP_RETURN_ON_ERROR(write_reg(REG_BANK0, REG_PWR_MGMT_1, 0x80), TAG, "reset failed");
        vTaskDelay(pdMS_TO_TICKS(100));

        // clock auto select, wake up
        ESP_RETURN_ON_ERROR(write_reg(REG_BANK0, REG_PWR_MGMT_1, 0x01), TAG, "wake failed");
        vTaskDelay(pdMS_TO_TICKS(10));

        // enable accel + gyro
        ESP_RETURN_ON_ERROR(write_reg(REG_BANK0, REG_PWR_MGMT_2, 0x00), TAG, "pwr mgmt2 failed");
        vTaskDelay(pdMS_TO_TICKS(10));

        // gyro ±250 dps
        ESP_RETURN_ON_ERROR(write_reg(REG_BANK2, REG_GYRO_CONFIG_1, 0x00), TAG, "gyro cfg failed");

        // accel ±2g
        ESP_RETURN_ON_ERROR(write_reg(REG_BANK2, REG_ACCEL_CONFIG, 0x00), TAG, "accel cfg failed");
        vTaskDelay(pdMS_TO_TICKS(10));

        // enable internal i2c master
        ESP_RETURN_ON_ERROR(write_reg(REG_BANK0, REG_USER_CTRL, 0x20), TAG, "enable i2c master failed");
        vTaskDelay(pdMS_TO_TICKS(10));

        // pulse internal i2c master reset
        ESP_RETURN_ON_ERROR(write_reg(REG_BANK0, REG_USER_CTRL, 0x22), TAG, "i2c master reset failed");
        vTaskDelay(pdMS_TO_TICKS(10));

        // leave only i2c master enabled
        ESP_RETURN_ON_ERROR(write_reg(REG_BANK0, REG_USER_CTRL, 0x20), TAG, "re-enable i2c master failed");
        vTaskDelay(pdMS_TO_TICKS(10));

        // internal master clock
        ESP_RETURN_ON_ERROR(write_reg(REG_BANK3, REG_I2C_MST_CTRL, 0x07), TAG, "mst ctrl failed");
        vTaskDelay(pdMS_TO_TICKS(10));

        // reset AK09916
        ESP_RETURN_ON_ERROR(ak_write(AK09916_REG_CNTL3, 0x01), TAG, "mag reset failed");
        vTaskDelay(pdMS_TO_TICKS(100));

        // identity read
        uint8_t wia1 = 0, wia2 = 0;
        ESP_RETURN_ON_ERROR(ak_read_byte(AK09916_REG_WIA1, wia1), TAG, "mag WIA1 failed");
        ESP_RETURN_ON_ERROR(ak_read_byte(AK09916_REG_WIA2, wia2), TAG, "mag WIA2 failed");
        ESP_LOGI(TAG, "AK09916 WIA1=0x%02X WIA2=0x%02X", wia1, wia2);

        if (wia1 != 0x48 || wia2 != 0x09) {
            ESP_LOGE(TAG, "Unexpected AK09916 identity");
            return ESP_FAIL;
        }

        // continuous mode 100 Hz
        ESP_RETURN_ON_ERROR(ak_write(AK09916_REG_CNTL2, 0x08), TAG, "mag mode set failed");
        vTaskDelay(pdMS_TO_TICKS(20));

        // configure SLV0 auto-fetch into EXT_SENS_DATA
        ESP_RETURN_ON_ERROR(configure_mag_continuous_read(), TAG, "mag continuous read config failed");
        vTaskDelay(pdMS_TO_TICKS(20));

        initialized_ = true;
        return ESP_OK;
    }

    esp_err_t read_all(ImuData &out) {
        if (!initialized_) return ESP_ERR_INVALID_STATE;

        // accel + gyro burst
        uint8_t buf[12] = {0};
        ESP_RETURN_ON_ERROR(read_reg(REG_BANK0, REG_ACCEL_XOUT_H, buf, 12), TAG, "accel/gyro burst failed");

        int16_t ax = (int16_t)((buf[0]  << 8) | buf[1]);
        int16_t ay = (int16_t)((buf[2]  << 8) | buf[3]);
        int16_t az = (int16_t)((buf[4]  << 8) | buf[5]);
        int16_t gx = (int16_t)((buf[6]  << 8) | buf[7]);
        int16_t gy = (int16_t)((buf[8]  << 8) | buf[9]);
        int16_t gz = (int16_t)((buf[10] << 8) | buf[11]);

        out.ax_g   = (float)ax / 16384.0f;
        out.ay_g   = (float)ay / 16384.0f;
        out.az_g   = (float)az / 16384.0f;
        out.gx_dps = (float)gx / 131.0f;
        out.gy_dps = (float)gy / 131.0f;
        out.gz_dps = (float)gz / 131.0f;

		// First debug path: read EXT_SENS_DATA
	    uint8_t mbuf[9] = {0};
	    ESP_RETURN_ON_ERROR(read_reg(REG_BANK0, REG_EXT_SENS_DATA_00, mbuf, 9), TAG, "mag ext sens read failed");

//	    printf("EXT:");
//	    for (int i = 0; i < 9; i++) printf(" %02X", mbuf[i]);
//	    printf("\n");
//
//	    // Second debug path: direct SLV4 read
//	    int16_t mx_dbg = 0, my_dbg = 0, mz_dbg = 0;
//	    uint8_t st1_dbg = 0, st2_dbg = 0;
//	    esp_err_t dbg_err = ak_read_mag_once(mx_dbg, my_dbg, mz_dbg, st1_dbg, st2_dbg);
//	    if (dbg_err == ESP_OK) {
//	        printf("SLV4 MAG: st1=0x%02X mx=%d my=%d mz=%d st2=0x%02X\n",
//	               st1_dbg, mx_dbg, my_dbg, mz_dbg, st2_dbg);
//	    } else {
//	        printf("SLV4 MAG read failed: %s\n", esp_err_to_name(dbg_err));
//	    }

        // layout: ST1, HXL, HXH, HYL, HYH, HZL, HZH, TMPS, ST2
		uint8_t st1 = mbuf[0];
		uint8_t st2 = mbuf[8];

		int16_t mx = (int16_t)((mbuf[2] << 8) | mbuf[1]);
		int16_t my = (int16_t)((mbuf[4] << 8) | mbuf[3]);
		int16_t mz = (int16_t)((mbuf[6] << 8) | mbuf[5]);

		out.mx_uT = (float)mx * 0.15f;
		out.my_uT = (float)my * 0.15f;
		out.mz_uT = (float)mz * 0.15f;

		// overflow bit only
		out.mag_valid = ((st2 & 0x08) == 0);
        return ESP_OK;
    }
	
	
	esp_err_t ak_read_mag_once(int16_t &mx, int16_t &my, int16_t &mz, uint8_t &st1, uint8_t &st2) {
	    uint8_t b[8] = {0};

	    ESP_RETURN_ON_ERROR(ak_read_byte(AK09916_REG_ST1, st1), TAG, "read ST1 failed");
	    if (!(st1 & 0x01)) {
	        mx = my = mz = 0;
	        st2 = 0;
	        return ESP_OK;
	    }

	    for (int i = 0; i < 6; i++) {
	        ESP_RETURN_ON_ERROR(ak_read_byte(AK09916_REG_HXL + i, b[i]), TAG, "read mag byte failed");
	    }
	    ESP_RETURN_ON_ERROR(ak_read_byte(0x18, st2), TAG, "read ST2 failed");

	    mx = (int16_t)((b[1] << 8) | b[0]);
	    my = (int16_t)((b[3] << 8) | b[2]);
	    mz = (int16_t)((b[5] << 8) | b[4]);
	    return ESP_OK;
	}
	
private:
    static constexpr const char *TAG = "ICM20948";
    static constexpr int TIMEOUT_MS = 100;

    // bank select
    static constexpr uint8_t REG_BANK_SEL = 0x7F;
    static constexpr uint8_t REG_BANK0 = 0x00;
    static constexpr uint8_t REG_BANK1 = 0x10;
    static constexpr uint8_t REG_BANK2 = 0x20;
    static constexpr uint8_t REG_BANK3 = 0x30;

	
    // bank 0
    static constexpr uint8_t REG_WHO_AM_I         = 0x00;
    static constexpr uint8_t REG_USER_CTRL        = 0x03;
    static constexpr uint8_t REG_PWR_MGMT_1       = 0x06;
    static constexpr uint8_t REG_PWR_MGMT_2       = 0x07;
    static constexpr uint8_t REG_ACCEL_XOUT_H     = 0x2D;
    static constexpr uint8_t REG_EXT_SENS_DATA_00 = 0x3B;

    // bank 2
    static constexpr uint8_t REG_GYRO_CONFIG_1    = 0x01;
    static constexpr uint8_t REG_ACCEL_CONFIG     = 0x14;

    // bank 3
    static constexpr uint8_t REG_I2C_MST_CTRL     = 0x01;
    static constexpr uint8_t REG_I2C_SLV0_ADDR    = 0x03;
    static constexpr uint8_t REG_I2C_SLV0_REG     = 0x04;
    static constexpr uint8_t REG_I2C_SLV0_CTRL    = 0x05;

    static constexpr uint8_t REG_I2C_SLV4_ADDR    = 0x13;
    static constexpr uint8_t REG_I2C_SLV4_REG     = 0x14;
    static constexpr uint8_t REG_I2C_SLV4_CTRL    = 0x15;
    static constexpr uint8_t REG_I2C_SLV4_DO      = 0x16;
    static constexpr uint8_t REG_I2C_SLV4_DI      = 0x17;

    // AK09916
    static constexpr uint8_t AK09916_I2C_ADDR     = 0x0C;
    static constexpr uint8_t AK09916_REG_WIA1     = 0x00;
    static constexpr uint8_t AK09916_REG_WIA2     = 0x01;
    static constexpr uint8_t AK09916_REG_ST1      = 0x10;
    static constexpr uint8_t AK09916_REG_CNTL2    = 0x31;
    static constexpr uint8_t AK09916_REG_CNTL3    = 0x32;
	static constexpr uint8_t AK09916_REG_HXL = 0x11;
	
    i2c_master_dev_handle_t dev_ = nullptr;
    bool initialized_ = false;
    uint8_t current_bank_ = 0xFF;

    esp_err_t set_bank(uint8_t bank) {
        if (current_bank_ == bank) return ESP_OK;
        uint8_t payload[2] = {REG_BANK_SEL, bank};
        esp_err_t err = i2c_master_transmit(dev_, payload, sizeof(payload), TIMEOUT_MS);
        if (err == ESP_OK) current_bank_ = bank;
        return err;
    }

    esp_err_t write_reg(uint8_t bank, uint8_t reg, uint8_t value) {
        ESP_RETURN_ON_ERROR(set_bank(bank), TAG, "set bank failed");
        uint8_t payload[2] = {reg, value};
        return i2c_master_transmit(dev_, payload, sizeof(payload), TIMEOUT_MS);
    }

    esp_err_t read_reg(uint8_t bank, uint8_t reg, uint8_t *data, size_t len) {
        ESP_RETURN_ON_ERROR(set_bank(bank), TAG, "set bank failed");
        return i2c_master_transmit_receive(dev_, &reg, 1, data, len, TIMEOUT_MS);
    }

    esp_err_t ak_write(uint8_t reg, uint8_t value) {
        ESP_RETURN_ON_ERROR(write_reg(REG_BANK3, REG_I2C_SLV4_ADDR, AK09916_I2C_ADDR), TAG, "slv4 addr write failed");
        ESP_RETURN_ON_ERROR(write_reg(REG_BANK3, REG_I2C_SLV4_REG, reg), TAG, "slv4 reg write failed");
        ESP_RETURN_ON_ERROR(write_reg(REG_BANK3, REG_I2C_SLV4_DO, value), TAG, "slv4 do write failed");
        ESP_RETURN_ON_ERROR(write_reg(REG_BANK3, REG_I2C_SLV4_CTRL, 0x80), TAG, "slv4 ctrl write failed");

        // crude but robust enough for now
        vTaskDelay(pdMS_TO_TICKS(20));
        return ESP_OK;
    }

    esp_err_t ak_read_byte(uint8_t reg, uint8_t &value) {
        ESP_RETURN_ON_ERROR(write_reg(REG_BANK3, REG_I2C_SLV4_ADDR, 0x80 | AK09916_I2C_ADDR), TAG, "slv4 addr read failed");
        ESP_RETURN_ON_ERROR(write_reg(REG_BANK3, REG_I2C_SLV4_REG, reg), TAG, "slv4 reg set failed");
        ESP_RETURN_ON_ERROR(write_reg(REG_BANK3, REG_I2C_SLV4_CTRL, 0x80), TAG, "slv4 ctrl start failed");

        // crude but robust enough for now
        vTaskDelay(pdMS_TO_TICKS(20));

        ESP_RETURN_ON_ERROR(read_reg(REG_BANK3, REG_I2C_SLV4_DI, &value, 1), TAG, "slv4 di read failed");
        return ESP_OK;
    }

    esp_err_t configure_mag_continuous_read() {
        // fetch ST1 + HXL..HZH + TMPS + ST2 into EXT_SENS_DATA_00..08
        ESP_RETURN_ON_ERROR(write_reg(REG_BANK3, REG_I2C_SLV0_ADDR, 0x80 | AK09916_I2C_ADDR), TAG, "slv0 addr failed");
        ESP_RETURN_ON_ERROR(write_reg(REG_BANK3, REG_I2C_SLV0_REG, AK09916_REG_ST1), TAG, "slv0 reg failed");
        ESP_RETURN_ON_ERROR(write_reg(REG_BANK3, REG_I2C_SLV0_CTRL, 0x80 | 0x09), TAG, "slv0 ctrl failed");
        return ESP_OK;
    }
};


class NineAxis {
public:
    ICM20948 imu;
    bool initialized = false;

    explicit NineAxis(i2c_master_dev_handle_t dev)
        : imu(dev)
    {
        esp_err_t err = imu.init();
        if (err != ESP_OK) {
            printf("ICM20948 init failed: %s\n", esp_err_to_name(err));
            initialized = false;
        } else {
            printf("ICM20948 initialized successfully\n");
            initialized = true;
        }
    }

    esp_err_t fetchAll(float &ax, float &ay, float &az,
                       float &gx, float &gy, float &gz,
                       float &mx, float &my, float &mz,
                       bool getMags = false)
    {
        if (!initialized) return ESP_ERR_INVALID_STATE;

        ICM20948::ImuData data;
        esp_err_t err = imu.read_all(data);
        if (err != ESP_OK) {
            printf("ICM20948 read failed: %s\n", esp_err_to_name(err));
            return err;
        }

        // keep your old sign convention
        ax = -data.ax_g;
        ay = -data.ay_g;
        az =  data.az_g;

        gx = -data.gx_dps;
        gy = -data.gy_dps;
        gz =  data.gz_dps;

        if (getMags) {
            mx =  data.mx_uT;
            my = -data.my_uT;
            mz =  data.mz_uT;

            if (!data.mag_valid) {
                return ESP_ERR_INVALID_RESPONSE;
            }
        }

        return ESP_OK;
    }
};



class MPU6500 {
public:
    static constexpr uint8_t ADDR_LOW  = 0x68;
    static constexpr uint8_t ADDR_HIGH = 0x69;

    static constexpr uint8_t REG_SMPLRT_DIV    = 0x19;
    static constexpr uint8_t REG_CONFIG        = 0x1A;
    static constexpr uint8_t REG_GYRO_CONFIG   = 0x1B;
    static constexpr uint8_t REG_ACCEL_CONFIG  = 0x1C;
    static constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D;
    static constexpr uint8_t REG_ACCEL_XOUT_H  = 0x3B;
    static constexpr uint8_t REG_PWR_MGMT1     = 0x6B;
    static constexpr uint8_t REG_PWR_MGMT2     = 0x6C;
    static constexpr uint8_t REG_WHO_AM_I      = 0x75;

    explicit MPU6500(i2c_master_dev_handle_t dev) : dev_(dev) {}

    esp_err_t init() {
        uint8_t who = 0;
        ESP_RETURN_ON_ERROR(read_reg(REG_WHO_AM_I, &who, 1), TAG, "WHO_AM_I read failed");
        printf("MPU6500 WHO_AM_I = 0x%02X\n", who);

        if (who != 0x70) {
            printf("Unexpected WHO_AM_I\n");
            return ESP_FAIL;
        }

        ESP_RETURN_ON_ERROR(write_reg(REG_PWR_MGMT1, 0x01), TAG, "wake failed");
        vTaskDelay(pdMS_TO_TICKS(50));

        ESP_RETURN_ON_ERROR(write_reg(REG_PWR_MGMT2, 0x00), TAG, "PWR_MGMT2 failed");
        ESP_RETURN_ON_ERROR(write_reg(REG_CONFIG, 0x03), TAG, "CONFIG failed");

        // ±250 dps
        ESP_RETURN_ON_ERROR(write_reg(REG_GYRO_CONFIG, 0x00), TAG, "GYRO_CONFIG failed");

        // ±2 g
        ESP_RETURN_ON_ERROR(write_reg(REG_ACCEL_CONFIG, 0x00), TAG, "ACCEL_CONFIG failed");

        // accel DLPF config
        ESP_RETURN_ON_ERROR(write_reg(REG_ACCEL_CONFIG2, 0x03), TAG, "ACCEL_CONFIG2 failed");

        // sample divider
        ESP_RETURN_ON_ERROR(write_reg(REG_SMPLRT_DIV, 0x04), TAG, "SMPLRT_DIV failed");

        vTaskDelay(pdMS_TO_TICKS(50));
        initialized_ = true;
        return ESP_OK;
    }

    esp_err_t read_accel_gyro_raw(int16_t &ax, int16_t &ay, int16_t &az,
                                  int16_t &gx, int16_t &gy, int16_t &gz) {
        if (!initialized_) return ESP_ERR_INVALID_STATE;

        uint8_t buf[14] = {0};
        ESP_RETURN_ON_ERROR(read_reg(REG_ACCEL_XOUT_H, buf, 14), TAG, "burst read failed");

        ax = (int16_t)((buf[0]  << 8) | buf[1]);
        ay = (int16_t)((buf[2]  << 8) | buf[3]);
        az = (int16_t)((buf[4]  << 8) | buf[5]);

        gx = (int16_t)((buf[8]  << 8) | buf[9]);
        gy = (int16_t)((buf[10] << 8) | buf[11]);
        gz = (int16_t)((buf[12] << 8) | buf[13]);

        return ESP_OK;
    }

    esp_err_t fetchAll(float &ax_g, float &ay_g, float &az_g,
                       float &gx_dps, float &gy_dps, float &gz_dps) {
        int16_t ax, ay, az, gx, gy, gz;
        ESP_RETURN_ON_ERROR(read_accel_gyro_raw(ax, ay, az, gx, gy, gz), TAG, "raw read failed");

        ax_g   = static_cast<float>(ax) / 16384.0f; // ±2g
        ay_g   = static_cast<float>(ay) / 16384.0f;
        az_g   = static_cast<float>(az) / 16384.0f;

        gx_dps = static_cast<float>(gx) / 131.0f;   // ±250 dps
        gy_dps = static_cast<float>(gy) / 131.0f;
        gz_dps = static_cast<float>(gz) / 131.0f;

        return ESP_OK;
    }

private:
    static constexpr const char *TAG = "MPU6500";
    static constexpr int TIMEOUT_MS = 100;

    i2c_master_dev_handle_t dev_ = nullptr;
    bool initialized_ = false;

    esp_err_t write_reg(uint8_t reg, uint8_t value) {
        uint8_t payload[2] = {reg, value};
        return i2c_master_transmit(dev_, payload, sizeof(payload), TIMEOUT_MS);
    }

    esp_err_t read_reg(uint8_t reg, uint8_t *data, size_t len) {
        return i2c_master_transmit_receive(dev_, &reg, 1, data, len, TIMEOUT_MS);
    }
};




class IMUInterface{
	
public:
	std::vector <double> accels;
	std::vector <double> gyros;
	std::vector <double> mags;
	std::vector <std::vector<double>> accelBias;
	std::vector <std::vector<double>> gyroBias;
	std::vector <std::vector<double>> accelScale;
	std::vector <double> magBias2;
	std::vector <double> magScale2;
	
	
	MPU6500 imu1;
	NineAxis imu2;
	IMUInterface(i2c_master_dev_handle_t  i1, i2c_master_dev_handle_t  i2): imu1(i1), imu2(i2)
	{
		// this is a constructor 
		printf("break bread with the enemy?\n");
		this->accels = {0.0, 0.0, 0.0};
		this->gyros = {0.0, 0.0, 0.0};
		this->mags = {0.0, 0.0, 0.0};
		this->accelBias.resize(2, std::vector <double> (3, 0.0));
		this->gyroBias.resize(2, std::vector<double>(3, 0.0));
		this->accelScale.resize(2, std::vector<double>(3, 1.0));
		this->magScale2.resize(3, 1.0);
		this->magBias2.resize(3, 0);		
		imu1.init();
		
	}
	
	void calibrateAccelGyro(int which_imu){
		// for now this will print to the console at the beginning of every phase
		printf("Starting Accel Calibration, place the device z up\n");
		vTaskDelay(pdMS_TO_TICKS(8000));
		float ax, ay, az, gx, gy, gz, mx, my, mz;
		float accelsd1[3] = {0.0, 0.0, 0.0};
		float accelsd2[3] = {0.0, 0.0, 0.0};
		for(int i = 0; i < 1000; i++){	
			esp_err_t e;
			if(which_imu == 0) e = imu1.fetchAll(ax, ay, az, gx, gy, gz);
			else e = imu2.fetchAll(ax, ay, az, gx, gy, gz, mx, my, mz, false);
			accelsd1[0] += az;
		}
		accelsd1[0] /= 1000.0;
		printf("Place the device z down\n");
		vTaskDelay(pdMS_TO_TICKS(8000));
		for(int i = 0; i < 1000; i++){
			esp_err_t e;
			if(which_imu == 0) e = imu1.fetchAll(ax, ay, az, gx, gy, gz);
			else e = imu2.fetchAll(ax, ay, az, gx, gy, gz, mx, my, mz, false);
			accelsd2[0] += az;
		}
		accelsd2[0] /= 1000.0; 
		this->accelBias[which_imu][0]= (accelsd1[0] + accelsd2[0])/2.0;
		this->accelScale[which_imu][0]= (accelsd1[0] - accelsd2[0])/2.0;
			
		//////
		printf("Place the device y up\n");
		vTaskDelay(pdMS_TO_TICKS(8000));
		for(int i = 0; i < 1000; i++){	
			esp_err_t e;
			if(which_imu == 0) e = imu1.fetchAll(ax, ay, az, gx, gy, gz);
			else e = imu2.fetchAll(ax, ay, az, gx, gy, gz, mx, my, mz, false);
			accelsd1[1] += ay;
		}
		accelsd1[1] /= 1000.0;
		printf("Place the device y down\n");
		vTaskDelay(pdMS_TO_TICKS(8000));
		for(int i = 0; i < 1000; i++){
			esp_err_t e;
			if(which_imu == 0) e = imu1.fetchAll(ax, ay, az, gx, gy, gz);
			else e = imu2.fetchAll(ax, ay, az, gx, gy, gz, mx, my, mz, false);
			accelsd2[1] += ay;
		}
		accelsd2[1] /= 1000.0; 
		this->accelBias[which_imu][1]= (accelsd1[1] + accelsd2[1])/2.0;
		this->accelScale[which_imu][1]= (accelsd1[1] - accelsd2[1])/2.0;
		
		//////
		
		printf("Place the device x up\n");
		vTaskDelay(pdMS_TO_TICKS(8000));
		for(int i = 0; i < 1000; i++){	
			esp_err_t e;
			if(which_imu == 0) e = imu1.fetchAll(ax, ay, az, gx, gy, gz);
			else e = imu2.fetchAll(ax, ay, az, gx, gy, gz, mx, my, mz, false);
			accelsd1[2] += ax;
		}
		accelsd1[2] /= 1000.0;
		printf("Place the device x down\n");
		vTaskDelay(pdMS_TO_TICKS(8000));
		for(int i = 0; i < 1000; i++){
			esp_err_t e;
			if(which_imu == 0) e = imu1.fetchAll(ax, ay, az, gx, gy, gz);
			else e = imu2.fetchAll(ax, ay, az, gx, gy, gz, mx, my, mz, false);
			accelsd2[2] += ax;
		}
		accelsd2[2] /= 1000.0; 
		this->accelBias[which_imu][2]= (accelsd1[2] + accelsd2[2])/2.0;
		this->accelScale[which_imu][2]= (accelsd1[2] - accelsd2[2])/2.0;
		
		
		// gyro cal 
		printf("Place the device level and z up to calibrate Gyros\n");
		vTaskDelay(pdMS_TO_TICKS(8000));
		float g1 = 0.0; float g2 = 0.0; float g3 = 0.0;
		for(int i = 0; i < 1000; i++){
			esp_err_t e;
			if(which_imu == 0) e = imu1.fetchAll(ax, ay, az, gx, gy, gz);
			else e = imu2.fetchAll(ax, ay, az, gx, gy, gz, mx, my, mz, false);
			g1 += gx; g2 += gy; g3 += gz;
		}
		g1 /= 1000.0; g2 /= 1000.0; g3 /= 1000.0;
		this->gyroBias[which_imu][0] = g1; this->gyroBias[which_imu][1] = g2; this->gyroBias[which_imu][2] = g3;
		return;
		
	}
	
	void calibrateMag(){
		// this has to be imu 2
		printf("Calibrating mags, keep moving !\n");
		int64_t start_time_us = esp_timer_get_time();
		float ax, ay, az, gx, gy, gz, mx, my, mz;
		float max_x = -1e6, min_x = 1e6, max_y = -1e6, min_y = 1e6, max_z = -1e6, min_z = 1e6;
		while((esp_timer_get_time() - start_time_us) < 60LL*1000000LL){
			esp_err_t e = imu2.fetchAll(ax, ay, az, gx, gy, gz, mx, my, mz, true);
			max_x = std::max(mx, max_x); min_x = std::min(mx, min_x);
			max_y = std::max(my, max_y); min_y = std::min(my, min_y);
			max_z = std::max(mz, max_z); min_z = std::min(mz, min_z);
		}
		float bx = (max_x  + min_x)/2.0; float by = (max_y + min_y)/2.0; float bz = (max_z + min_z)/2.0;
		float rx = (max_x  - min_x)/2.0; float ry = (max_y - min_y)/2.0; float rz = (max_z - min_z)/2.0;
		float r = (rx + ry + rz)/3.0;
		this->magScale2[0] = r/rx; this->magScale2[1] = r/ry; this->magScale2[2] = r/rz;
		this->magBias2[0] = bx; this->magBias2[1] = by; this->magBias2[2] = bz;
		return;
	}
	
	void calibrateAll(){
		this->calibrateAccelGyro(0);
		this->calibrateAccelGyro(1);
		this->calibrateMag();
	}
	

	void testComms(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &mx, float &my, float &mz, int which_imu = 1){
		if(which_imu == 0){
			esp_err_t e = imu1.fetchAll(ax, ay, az, gx, gy, gz);
		}
		else{
			esp_err_t e = imu2.fetchAll(ax, ay, az, gx, gy, gz, mx, my, mz, true);
		}
		return;
	}
	
	
	void fetchData(bool accel = true, bool gyro = true, bool mags = false){
		float ax, ay, az, gx, gy, gz, mx, my, mz;
			// code to fetch data from MPU
		esp_err_t e = imu1.fetchAll(ax, ay, az, gx, gy, gz);
		float accels1[3] = {ax, ay, az};
		float gyros1[3] = {gx, gy, gz};
		
		for(int i = 0; i < 3; i++){
			accels1[i] = (accels1[i] - this->accelBias[0][i])/this->accelScale[0][i];
			gyros1[i] = gyros1[i] - this->gyroBias[0][i];
		}
		
		// code to fetch data from ICM
		esp_err_t f = imu2.fetchAll(ax, ay, az, gx, gy, gz, mx, my, mz, true);
		float accels2[3] = {ax, ay, az};
		float gyros2[3] = {gx, gy, gz};
		float mags2[3] = {mx, my, mz};
		
		for(int i = 0; i < 3; i++){
			accels2[i] = (accels2[i] - this->accelBias[1][i])/this->accelScale[1][i];
			gyros2[i] = gyros2[i] - this->gyroBias[1][i];
			mags2[i] = (mags2[i] - this->magBias2[i])*(this->magScale2[i]);
		}
		
		for(int i = 0; i < 3; i++){
			this->accels[i] = (accels1[i] + accels2[i])*0.5;
			this->gyros[i] = (gyros1[i] + gyros2[i])*0.5;
			this->mags[i] = mags2[i];
		}
		// update complete
		return;
	}
	
	
	
};


class AHRS{
	
	// there probably is a way to implement this in a cleaner, more efficient manner! 
	
	
	// fix the accel reliability when the vehicle is undergoing acceleration
	
	public:
	IMUInterface imu;
	std::vector <std::vector<double>> rotationMatrix;
	std::vector <double> quaternion;
	std::vector <double> rotvec;
	std::vector <double> initialMagneticVector;
	
	std::vector <std::vector<double>> hatTarget;
	std::vector <double> hatInput;
	std::vector <double> eulerAngles;
	std::vector <double> cache;
	std::vector <double> rotVecStorage;
	
	int cachePointer = 0;
	double refresh_rate = 100.0;
	bool history = false; 
	bool imus_need_calibration = false;
	// data 

	AHRS(i2c_master_dev_handle_t  i1, i2c_master_dev_handle_t  i2):
	imu(i1, i2)
	{
		this->rotationMatrix = {{1.0, 0.0, 0.0},{0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
		this->quaternion = {0.0, 0.0, 0.0, 0.0};
		this->rotvec = {0.0, 0.0, 0.0};
		this->initialMagneticVector = {0.0, 0.0, 0.0};
		(this->hatTarget).resize(3, std::vector<double> (3, 0.0));
		(this->hatInput).resize(3, 0.0);
		(this->eulerAngles).resize(3, 0.0);
		(this->rotVecStorage).resize(3, 0.0);
		if(imus_need_calibration){
			(this->imu).calibrateAll();
		}
	}
	
	void printCalibration(){
		printf("Printing Accel Scales: \n");
		for(auto it: (this->imu).accelScale){
			printf("%f %f %f\n", it[0], it[1], it[2]);
		}
		printf("Printing Accel Bias: \n");
		for(auto it: (this->imu).accelBias){
			printf("%f %f %f\n", it[0], it[1], it[2]);
		}
		printf("Printing Gyro Bias: \n");
		for(auto it: (this->imu).gyroBias){
			printf("%f %f %f\n", it[0], it[1], it[2]);
		}
		printf("Printing Mag Bias and then Scale:\n");
		printf("%f %f %f\n", (this->imu).magBias2[0], (this->imu).magBias2[1], (this->imu).magBias2[2]);
		printf("%f %f %f\n", (this->imu).magScale2[0], (this->imu).magScale2[1], (this->imu).magScale2[2]);
	}
	
	void rotvecFromMatrix(){
		// this assumes the rotation matrix is populated with the correct entries
		double trace = rotationMatrix[0][0] + rotationMatrix[1][1] + rotationMatrix[2][2];
		double acostarget = clip(0.5*(trace - 1), -1.0, 1.0);
		double angle = acos(acostarget);
		double sinangle = sin(angle);
		if(std::abs(sinangle) < 1e-6){
			this->rotvec[0] = 0.0;
			this->rotvec[1] = 0.0;
			this->rotvec[2] = 0.0;
			return;
		}
		double a1 = ((this->rotationMatrix[0][1] - this->rotationMatrix[1][0])*0.5)/sinangle;
		double a2 = ((this->rotationMatrix[0][2] - this->rotationMatrix[2][0])*0.5)/sinangle;
		double a3 = ((this->rotationMatrix[1][2] - this->rotationMatrix[2][1])*0.5)/sinangle;
		
		this->rotvec[0] = -a3*angle;
		this->rotvec[1] = a2*angle;
		this->rotvec[2] = -a1*angle;
		return;
		
	}
	
	
	void logMap(std::vector<std::vector<double>> &rotation){
		double trace = rotation[0][0] + rotation[1][1] + rotation[2][2];
		double acostarget = clip(0.5*(trace - 1), -1.0, 1.0);
		double angle = acos(acostarget);
		double sinangle = sin(angle);
		if(std::abs(sinangle) < 1e-7){
			this->rotVecStorage[0] = 0.0;
			this->rotVecStorage[1] = 0.0;
			this->rotVecStorage[2] = 0.0;
			return;
		}
		double a1 = ((rotation[0][1] - rotation[1][0])*0.5)/sinangle;
		double a2 = ((rotation[0][2] - rotation[2][0])*0.5)/sinangle;
		double a3 = ((rotation[1][2] - rotation[2][1])*0.5)/sinangle;
		
		this->rotVecStorage[0] = -a3*angle;
		this->rotVecStorage[1] = a2*angle;
		this->rotVecStorage[2] = -a1*angle;
		return;
	}
	
	
 
	double norm(std::vector<double> &v){
		double val = 0.0; 
		for(int i = 0; i < v.size(); i++) val += v[i]*v[i];
		return sqrt(val);
	}
	
	void matmul(std::vector <std::vector<double>> &a, std::vector<std::vector<double>> &b, std::vector<std::vector<double>> &target){
		int m = a.size();
		int n = b.size();
		int l = b[0].size();
		for(int i = 0; i < m; i++){
			std::fill(target[i].begin(), target[i].end(), 0.0);
		}
		for(int k = 0; k < n; k++){
			for(int i = 0; i < m; i++){
				for(int j = 0; j < l; j++){
					target[i][j] += a[i][k]*b[k][j];
				}
			}
		}
		return;
	}
	
	
	void matrixVectorProduct(std::vector<std::vector<double>> &m, std::vector<double> &v, std::vector<double> &output){
		std::fill(output.begin(), output.end(), 0.0);
		for(int i = 0; i < m.size(); i++){
			for(int j = 0; j < m[0].size(); j++){
				output[i] += m[i][j]*v[j];
			}
		}
		return;
		
	}
	
	void crossProduct(std::vector<double> &v1, std::vector<double> &v2, std::vector<double> &output){
		output[0] = v1[1]*v2[2] - v2[1]*v1[2];
		output[1] = v1[2]*v2[0] - v2[2]*v1[0];
		output[2] = v1[0]*v2[1] - v2[0]*v1[1];
	}
	
	
	
	void interpolate_rotations(double alpha, std::vector <std::vector<double>> &r1, std::vector<std::vector<double>> &r2, std::vector<std::vector<double>> &target){
		// sketchy code but it works 
		this->logMap(r1);
		std::vector<double> log_r1(this->rotVecStorage);
		this->logMap(r2);
		std::vector<double> log_r2(this->rotVecStorage);
		log_r1[0] = alpha*log_r1[0];
		log_r1[1] = alpha*log_r1[1];
		log_r1[2] = alpha*log_r1[2];
		std::vector<std::vector<double>> alphaR1(this->hatTarget);
		this->exponential_map(log_r1, alphaR1);
		log_r2[0] = -alpha*log_r2[0];
		log_r2[1] = -alpha*log_r2[1];
		log_r2[2] = -alpha*log_r2[2];
		std::vector<std::vector<double>> minusAlphaR2(this->hatTarget);
		this->exponential_map(log_r2, minusAlphaR2);
		std::vector<std::vector<double>> intermediary(3, std::vector<double>(3, 0.0));
		this->matmul(minusAlphaR2, r2, intermediary);
		this->matmul(alphaR1, intermediary, target);
		
	}
	
	void exponential_map(std::vector<double> &source, std::vector<std::vector<double>> &target){
		// this performs the exponential operation on the contents of the exponential map and stores them in the cache
		double angle = sqrt(source[0]*source[0] + source[1]*source[1] + source[2]*source[2]);
		if(angle < 1e-4){
			for(int i = 0; i < target.size(); i++) for(int j = 0; j < target[0].size(); j++) target[i][j] = 0.0;
			target[0][0] = 1.0; target[1][1] = 1.0; target[2][2] = 1.0;
		}
		std::vector <double> axis = {source[0]/angle, source[1]/angle, source[2]/angle};
		this->hatInput[0] = axis[0]; this->hatInput[1] = axis[1]; this->hatInput[2] = axis[2];
		this->hat();
		std::vector <std::vector<double>> identity = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
		std::vector <std::vector<double>> outer = {
												{axis[0]*axis[0], axis[0]*axis[1], axis[0]*axis[2]},
												{axis[1]*axis[0], axis[1]*axis[1], axis[1]*axis[2]},
												{axis[2]*axis[0], axis[2]*axis[1], axis[2]*axis[2]}
												};
		for(int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) identity[i][j] *= cos(angle);
		for(int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) outer[i][j] *= (1 - cos(angle));
		for(int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) this->hatTarget[i][j] *= sin(angle);
		
		for(int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) target[i][j] += identity[i][j] + outer[i][j] + this->hatTarget[i][j];
		return;
	}
		
	void hat(){
		// this places a hat on whatever vector is stored in the hatInput
		// lets try to implement our operations without using matrix operations, since they are costly
		for(int i = 0; i < 3; i++) std::fill(this->hatTarget[i].begin(), this->hatTarget[i].end(), 0.0);
		this->hatTarget[0][1] = -this->hatInput[2];
		this->hatTarget[0][2] = this->hatInput[1];
		this->hatTarget[1][0] = this->hatInput[2];
		this->hatTarget[1][2] = -this->hatInput[0];
		this->hatTarget[2][1] = this->hatInput[0];
		this->hatTarget[2][0] = -this->hatInput[1];
	}
	
	void transpose(std::vector<std::vector<double>> &source, std::vector<std::vector<double>> &target){
		for(int i = 0; i < source.size(); i++){
			for(int j = 0; j < source[0].size(); j++) target[j][i] = source[i][j];
		}
		return;
	}
	
	void updateRotMat(){
		// we normalize the accels first 
		double normVal = norm((this->imu).accels);
		for(int i = 0; i < 3; i++) (this->imu).accels[i] /= normVal;
		double delta_t = 1/refresh_rate;
		for(int i = 0; i < 3; i++) (this->imu).gyros[i] *= delta_t;
		std::vector<std::vector<double>> targetForExponentiation(3, std::vector<double> (3, 0.0));
		this->exponential_map((this->imu).gyros, targetForExponentiation);
		std::vector<std::vector<double>> rotationPrior(targetForExponentiation);
		this->matmul(this->rotationMatrix, targetForExponentiation, rotationPrior);
		std::vector <double> gravity = {0.0, 0.0, 1.0};
		std::vector <double> targetDirection(gravity);
		std::vector <std::vector<double>> rotationTranspose(rotationPrior);
		this->transpose(rotationPrior, rotationTranspose);
		this->matrixVectorProduct(rotationTranspose, gravity, targetDirection);
		std::vector <double> correction(gravity);
		this->crossProduct((this->imu).accels, targetDirection, correction);
		double dotProduct = 0.0; for(int i = 0; i < 3; i++) dotProduct += targetDirection[i]*((this->imu).accels[i]);
		dotProduct = clip(dotProduct, -1, 1);
		double angle = acos(dotProduct);
		normVal = norm(correction);
		if(normVal > 1e-5){
			for(int i = 0; i < 3; i++) correction[i] = (correction[i]/normVal)*angle;
			std::vector<std::vector<double>> finalCorrection(rotationPrior);
			this->exponential_map(correction, finalCorrection);
			this->interpolate_rotations(0.9, rotationPrior, finalCorrection, this->rotationMatrix);
		} 
		else{
			for(int i = 0; i < 3; i++) for(int j = 0; j < 3; j++) this->rotationMatrix[i][j] = rotationPrior[i][j];
		}		
		return;
	}
	
	void updateRotMatWithMags(){
		double normVal = norm((this->imu).accels);
		for(int i = 0; i < 3; i++) (this->imu).accels[i] /= normVal;
		double delta_t = 1/refresh_rate;
		for(int i = 0; i < 3; i++) (this->imu).gyros[i] *= delta_t;
		std::vector<std::vector<double>> targetForExponentiation(3, std::vector<double> (3, 0.0));
		this->exponential_map((this->imu).gyros, targetForExponentiation);
		std::vector<std::vector<double>> rotationPrior(targetForExponentiation);
		this->matmul(this->rotationMatrix, targetForExponentiation, rotationPrior);
		std::vector <double> gravity = {0.0, 0.0, 1.0};
		std::vector <double> targetDirection(gravity);
		std::vector <std::vector<double>> rotationTranspose(rotationPrior);
		this->transpose(rotationPrior, rotationTranspose);
		this->matrixVectorProduct(rotationTranspose, gravity, targetDirection);
		std::vector <double> correction(gravity);
		this->crossProduct((this->imu).accels, targetDirection, correction);
		double dotProduct = 0.0; for(int i = 0; i < 3; i++) dotProduct += targetDirection[i]*((this->imu).accels[i]);
		dotProduct = clip(dotProduct, -1, 1);
		double angle = acos(dotProduct);
		normVal = norm(correction);
		if(normVal > 1e-5){
			for(int i = 0; i < 3; i++) correction[i] = (correction[i]/normVal)*angle;
			std::vector<std::vector<double>> finalCorrection(rotationPrior);
			this->exponential_map(correction, finalCorrection);
			this->interpolate_rotations(0.9, rotationPrior, finalCorrection, this->rotationMatrix);
		} 
		else{
			
			for(int i = 0; i < 3; i++) for(int j = 0; j < 3; j++) this->rotationMatrix[i][j] = rotationPrior[i][j];
		}		
		
		// mags
		std::vector<double> finalGravityVector(3, 0.0);
		std::vector<double> finalMagVector(3, 0.0);
		this->transpose(this->rotationMatrix, rotationTranspose);
		this->matrixVectorProduct(rotationTranspose, gravity, finalGravityVector);
		this->matrixVectorProduct(rotationTranspose, this->initialMagneticVector, finalMagVector);
		
		std::vector<double> measuredMagProjection(3, 0.0);
		double dotProd1 = 0.0;
		double dotProd2 = 0.0;
		for(int i = 0; i < 3; i++){
			dotProd1 += finalGravityVector[i]*finalMagVector[i];
			dotProd2 += finalGravityVector[i]*((this->imu).mags[i]);
			measuredMagProjection[i] = (this->imu).mags[i];
		}
		double norm1 = 0.0; double norm2 = 0.0; double dotProduct3 = 0.0;
		for(int i = 0; i < 3; i++){
			finalMagVector[i] -= dotProd1*finalGravityVector[i];
			measuredMagProjection[i] -= dotProd2*finalGravityVector[i];
			norm1 += (finalMagVector[i]*finalMagVector[i]);
			norm2 += (measuredMagProjection[i]*measuredMagProjection[i]);
			dotProduct3 += measuredMagProjection[i]*finalMagVector[i];
		}
		// normalized vectors for stability
		norm1 = sqrt(norm1); norm2 = sqrt(norm2); 
		for(int i = 0; i < 3; i++) finalMagVector[i] /= norm1;
		for(int i = 0; i < 3; i++) measuredMagProjection[i] /= norm2;
		
		std::vector<double> errorAxis(3, 0.0);
		this->crossProduct(measuredMagProjection, finalMagVector, errorAxis);
		dotProduct3 = clip(dotProduct3, -1.0, 1.0);
		double theta = acos(dotProduct3);
		if(norm(errorAxis)){
			double normie = norm(errorAxis);
			std::vector<double> scaledErrorAxis(3, 0.0);
			for(int i = 0; i < 3; i++) scaledErrorAxis[i] = (errorAxis[i]/normie)*theta;
			std::vector<std::vector<double>> rotationUpdate(3, std::vector<double>(3, 0.0));
			std::vector<std::vector<double>> finalRotationForUpdate(3, std::vector<double>(3, 0.0));
			this->exponential_map(scaledErrorAxis, rotationUpdate);
			this->matmul(this->rotationMatrix, rotationUpdate, finalRotationForUpdate);
			std::vector<std::vector<double>> rotationPrior2(this->rotationMatrix);
			this->interpolate_rotations(0.9, rotationPrior2, finalRotationForUpdate, this->rotationMatrix);
		}
		return;
	}
	
	void updatePipeline(){
		this->imu.fetchData(true, true, true);
		if(!history){
			history = true;
			this->initialMagneticVector[0] = (this->imu).mags[0];
			this->initialMagneticVector[1] = (this->imu).mags[1];
			this->initialMagneticVector[2] = (this->imu).mags[2];
		}
		this->updateRotMatWithMags();
		this->generateEulerAngles();
	}
	
	int generateEulerAngles(){
		// First we need to decide on an Euler Convention
		// This means 1. A rotation order, 2. Rotation type: intrinsic or extrinsic.
		// lets choose Intrinsic ZYX i.e. YPR, which means the outermost rotation is Yaw. This is equivalent to extrinsic XYZ. 
		double theta = asin(-this->rotationMatrix[2][0]); 
		double delta = theta - PI/2.0;
		if(std::abs(delta) < 1e-4){
			printf("GIMBAL LOCKKKKK");
		}
		double costheta = cos(theta);
		double phi = asin(this->rotationMatrix[2][1]/costheta);
		double psi = asin(this->rotationMatrix[1][0]/costheta);
		this->eulerAngles[0] = psi; this->eulerAngles[1] = theta; this->eulerAngles[2] = phi;
		return -1;
	}
	
	void generateControlAttitude(double roll, double pitch, double yaw, double &w_x, double &w_y, double &w_z){
		// rpy are extrinsic XYZ, intrinsic YZX 
		// we construct a rotation vector using the supplied attitude:
		// roll phi pitch theta yaw psi 
		double phi = roll; double theta = pitch; double psi = yaw;
		std::vector<std::vector<double>> commandedAttitude = {{cos(psi)*cos(theta), cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi), cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi)},
		{sin(psi)*cos(theta), sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi), sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi)},
		{-sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)}};
		
		std::vector<std::vector<double>> transposed(commandedAttitude);
		this->transpose(commandedAttitude, transposed);
		std::vector<std::vector<double>> rotationDelta(this->rotationMatrix);
		this->matmul(transposed, this->rotationMatrix, rotationDelta);
		this->logMap(rotationDelta);
		w_x = this->rotVecStorage[0];
		w_y = this->rotVecStorage[1];
		w_z = this->rotVecStorage[2];
	}
	
};





class StabilizeController{
	// very low level controller, PID based
	public:
	
	AHRS* ahorse;
	
	std::vector <double> state;
	std::vector <double> prevs;
	std::vector <double> kps;
	std::vector <double> kis;
	std::vector <double> kds;
	std::vector <double> accError;
	double integralWindupClamp = 2.0;
	std::vector <double> issuedControl;
	double delta_t = 1/100.0;
	bool history = false;
	// this merely tries to maintain and match angular setpoints
	
	
	
	StabilizeController(AHRS* a){
		(this->kps).resize(3, 1.0);
		(this->kds).resize(3, 2.0);
		(this->kis).resize(3, 0.01);
		(this->accError).resize(3, 0.0);
		(this->prevs).resize(3, 0.0);
		(this->issuedControl).resize(4, 0.0);
		(this->ahorse) = a;
	}
	
	
	
	void clipError(){
		for(int i = 0; i < 3; i++){
			(this->accError)[i] = std::max((this->accError)[i], -integralWindupClamp);
			(this->accError)[i] = std::min((this->accError[i]), integralWindupClamp);
		}
	}
	
	void generateCommandsPrincipledWay(double thrust, double roll, double pitch, double yaw){
		// this is da wae
		double currentYaw = this->ahorse->eulerAngles[2];
		double yaw_update = currentYaw + delta_t*yaw;
		double omega_x, omega_y, omega_z; // errors 
		this->ahorse->generateControlAttitude(roll, pitch, yaw_update, omega_x, omega_y, omega_z);
		this->accError[0] += omega_x*delta_t; this->accError[1] += omega_y*delta_t; this->accError[2] += omega_z*delta_t;
		
		this->clipError();
		if(!history){
			history = true;
			this->prevs[0] = omega_x; this->prevs[1] = omega_y; this->prevs[2] = omega_z;
		}
		double commanded_roll = omega_x*this->kps[0] + this->accError[0]*this->kis[0] + this->kds[0]*(omega_x - this->prevs[0])*delta_t;
		double commanded_pitch = omega_y*this->kps[1] + this->accError[1]*this->kis[1] + this->kds[1]*(omega_y - this->prevs[1])*delta_t;
		double commanded_yaw = omega_z*this->kps[2] + this->accError[2]*this->kis[2] + this->kds[2]*(omega_z - this->prevs[2])*delta_t;
		
		
		this->prevs[0] = omega_x; this->prevs[1] = omega_y; this->prevs[2] = omega_z;
		
		this->issuedControl[0] = thrust + commanded_pitch + commanded_roll - commanded_yaw;
		this->issuedControl[1] = thrust - commanded_pitch + commanded_roll + commanded_yaw;
		this->issuedControl[2] = thrust - commanded_pitch - commanded_roll - commanded_yaw;
		this->issuedControl[3] = thrust + commanded_pitch - commanded_roll + commanded_yaw;	
	}
	
//	void command(double thrust, double roll, double pitch, double yaw){
//		// expects data in radians
//		// also expects the external managing loop to maintain state. 
//		// for updating using stabilize, we only need to use the accel based tilt, since yaw is rate yaw anyway...
//		// but during periods of no RC Input, do we try to maintain the current pose by attending to readings of the latest rpy? 
//		this->ahorse->generateEulerAngles();
//		// this generates the current euler angles, minus the yaws....
//		// lets not handle the yaws right now....
//		// the extrinsic angles need to be converted into intrinsic angles....
//		// remap the extrinsic RPY set into an intrinsic one for the drone to realize. 
//		// as for the yaw, handle that separately 
//		// yaw alignment first, this requires the angular velocity in the z axis, brought to the world frame 
//		double error_yaw = 0.0;
//		// yaw fix generated
//		double current_roll= this->ahorse->eulerAngles[0];
//		double current_pitch = this->ahorse->eulerAngles[1];
//		double commanded_roll = this->ahorse->eulerInputConverted[0];
//		double commanded_pitch = this->ahorse->eulerInputConverted[1];
//		double error_roll = commanded_roll - current_roll;
//		double error_pitch = commanded_pitch - current_pitch;
//		this->accError[0] += error_roll;
//		this->accError[1] += error_pitch;
//		this->accError[2] += error_yaw;
//		double final_roll_command = this->kps[0]*error_roll + this->kis[0]*(this->accError[0]) + this->kds[0]*(error_roll - this->prevs[0]);
//		double final_pitch_command = this->kps[1]*error_pitch + this->kis[1]*(this->accError[1]) + this->kds[1]*(error_pitch - this->prevs[1]);
//		double final_yaw_command = this->kps[2]*error_yaw + this->kis[2]*(this->accError[2]) + this->kds[2]*(error_yaw - this->prevs[2]);
//		// motor mixing baby! 
//		this->issuedControl[0] = thrust + final_pitch_command + final_roll_command - final_yaw_command;
//		this->issuedControl[1] = thrust - final_pitch_command + final_roll_command + final_yaw_command;
//		this->issuedControl[2] = thrust - final_pitch_command - final_roll_command - final_yaw_command;
//		this->issuedControl[3] = thrust + final_pitch_command - final_roll_command + final_yaw_command;
//		return;
//	}
};


class RCInterface{
	public:
	crsf_channels_t rc{};
	int minPwms[5] = {10000, 10000, 10000, 10000, 10000};
	int maxPwms[5] = {0,0,0,0,0};
	int rcCenters[5] = {1500, 1500, 1500, 1500, 1500};
	
	
	RCInterface(){
		// binding complete!
		crsf_init();
		
	}
	
	void poll(int &c1v, int &c2v, int &c3v, int &c4v, int &c5v){
		if(crsf_poll(&rc)){
//			ESP_LOGI("CHANNEL VALUES READ!",
//                     "CH1=%u CH2=%u CH3=%u CH4=%u AUX1=%u AUX2=%u AUX3=%u AUX4=%u",
//                     rc.ch[0], rc.ch[1], rc.ch[2], rc.ch[3], rc.ch[4]);
			c1v = rc.ch[0]; c2v = rc.ch[1]; c3v = rc.ch[2]; c4v = rc.ch[3]; c5v = rc.ch[4];
		}
		uint64_t now = esp_timer_get_time();
		if(rc.last_frame_us != 0 && (now - rc.last_frame_us) > 500000){
			// we need to disarm, fill c5v with disarmed state! 
			c5v = -1;
		}
	}
	void calibrate(){
		printf("Set the thrust to min\n");
		int minima = 1000, maxima=2000;
		vTaskDelay(pdMS_TO_TICKS(5000));
		int a=0,b=0,c=0,d=0,e=0;
		int64_t start_time = esp_timer_get_time();
		while(esp_timer_get_time() - start_time < 7000000){
			this->poll(b,c,a,d,e);
			minima = std::min(minima, a);
			vTaskDelay(pdMS_TO_TICKS(50));
		}
		this->minPwms[2] = minima;
		printf("Min thrust %d\n", minima);
		printf("Set the thrust to max\n");
		start_time = esp_timer_get_time();
		while(esp_timer_get_time() - start_time < 7000000){
			this->poll(b,c,a,d,e);
			maxima= std::max(maxima, a);
			vTaskDelay(pdMS_TO_TICKS(50));
		}
		this->maxPwms[2] = maxima;
		printf("Max thrust %d\n", maxima);
		
		minima = 2000; maxima=1000;
		
		printf("Set the yaw to min\n");
		start_time = esp_timer_get_time();
		while(esp_timer_get_time() - start_time < 7000000){
			this->poll(d,c,a,b,e);
			minima = std::min(minima, b);
			vTaskDelay(pdMS_TO_TICKS(50));
		}
		this->minPwms[3] = minima;
		printf("Min yaw %d\n", minima);
		printf("Set the yaw to max\n");
		start_time = esp_timer_get_time();
		while(esp_timer_get_time() - start_time < 7000000){
			this->poll(d,c,a,b,e);
			maxima= std::max(maxima, b);
			vTaskDelay(pdMS_TO_TICKS(50));
		}
		this->maxPwms[3] = maxima;
		printf("Max Yaw %d\n", maxima);
		
		maxima = 1000; minima = 2000;
		
		printf("Set the pitch to min\n");
		start_time = esp_timer_get_time();
		while(esp_timer_get_time() - start_time < 7000000){
			this->poll(a,c,d,b,e);
			minima = std::min(minima, c);
			vTaskDelay(pdMS_TO_TICKS(50));
		}
		this->minPwms[1] = minima;
		printf("Pitch min %d\n", minima);
		
		printf("Set the pitch to max\n");
		start_time = esp_timer_get_time();
		while(esp_timer_get_time() - start_time < 7000000){
			this->poll(a,c,d,b,e);
			maxima= std::max(maxima, c);
			vTaskDelay(pdMS_TO_TICKS(50));
		}
		this->maxPwms[1] = maxima;
		printf("Max Pitch %d\n", maxima);
		
		maxima = 1000; minima = 2000;
		
		printf("Set the roll to min\n");
		start_time = esp_timer_get_time();
		while(esp_timer_get_time() - start_time < 7000000){
			this->poll(d,c,a,b,e);
			minima = std::min(minima, d);
			vTaskDelay(pdMS_TO_TICKS(50));
		}
		this->minPwms[0] = minima;
		printf("Roll min %d\n", minima);
		
		printf("Set the roll to max\n");
		start_time = esp_timer_get_time();
		while(esp_timer_get_time() - start_time < 7000000){
			this->poll(d,c,a,b,e);
			maxima= std::max(maxima, d);
			vTaskDelay(pdMS_TO_TICKS(50));
		}
		this->maxPwms[0] = maxima;
		printf("Max Roll %d\n", maxima);
		
		
		minima = 2000; maxima = 1000;
		printf("Set the arming switch to min\n");
		start_time = esp_timer_get_time();
		while(esp_timer_get_time() - start_time < 7000000){
			this->poll(a,c,d,b,e);
			minima = std::min(minima, e);
			vTaskDelay(pdMS_TO_TICKS(50));
		}
		this->minPwms[4] = minima;
		printf("Disarmed pwm %d\n", minima);
		
		printf("Set the arming switch to max\n");
		start_time = esp_timer_get_time();
		while(esp_timer_get_time() - start_time < 7000000){
			this->poll(a,c,d,b,e);
			maxima= std::max(maxima, e);
			vTaskDelay(pdMS_TO_TICKS(50));
		}
		this->maxPwms[4] = maxima;
		printf("Arming pwm %d\n", maxima);
		
		for(int i = 0; i < 5; i++){
			this->rcCenters[i] = (this->minPwms[i] + this->maxPwms[i])/2;
		}
		
	}
};

// the scaling factor for the yaw rate is in the Stabilize loop

uint32_t clipInt(uint32_t x, uint32_t left, uint32_t right){
	x = std::min(x, right);
	x = std::max(x, left);
	return x;
}


class Drone{
	public:
	AHRS AHorse;
	AHRS* pointer = &AHorse;
	StabilizeController Commander;
	RCInterface Radio;
	ESCInterface escs;
	bool armed = false;
	uint32_t escmin = 1000;
	uint32_t escmax = 2000;
	double maxRoll = PI/4.0;
	double maxPitch = PI/4.0;
	double maxYawRate = PI/4.0;
	
	bool haveParams = true;	
	
	
	Drone(i2c_master_dev_handle_t i1, i2c_master_dev_handle_t  i2):
		AHorse(i1, i2),
		Commander(pointer),
		Radio(),
		escs()
	{
		ParameterStore store;
		ESP_ERROR_CHECK(store.init());
		
		ParameterBlock params{};
		esp_err_t e = store.load(params);
		if(e != ESP_OK){
			haveParams = false;
			AHorse.imu.calibrateAll();
			for (int i = 0; i < 3; ++i) {
		        params.accel_bias1[i]   = AHorse.imu.accelBias[0][i];
		        params.accel_bias2[i]   = AHorse.imu.accelBias[1][i];
		        params.accel_scales1[i] = AHorse.imu.accelScale[0][i];
		        params.accel_scales2[i] = AHorse.imu.accelScale[1][i];
		        params.gyro_bias1[i]    = AHorse.imu.gyroBias[0][i];
		        params.gyro_bias2[i]    = AHorse.imu.gyroBias[1][i];
		        params.mag_bias[i]      = AHorse.imu.magBias2[i];
		        params.mag_scales[i]    = AHorse.imu.magScale2[i];
		    }
		    ESP_ERROR_CHECK(store.save(params));
			ESP_LOGW("Taginator", "No valid params in NVS, writing defaults");
			params = ParameterStore::make_defaults();
		}
		
		if(haveParams){
			AHorse.imu.accelScale[0] = {params.accel_scales1[0], params.accel_scales1[1], params.accel_scales1[2]};
			AHorse.imu.accelScale[1] = {params.accel_scales2[0], params.accel_scales2[1], params.accel_scales2[2]};
			AHorse.imu.accelBias[0] =  {params.accel_bias1[0], params.accel_bias1[1], params.accel_bias1[2]};
			AHorse.imu.accelBias[1] = {params.accel_bias2[0], params.accel_bias2[1], params.accel_bias2[2]};
			AHorse.imu.gyroBias[0] = {params.gyro_bias1[0], params.gyro_bias1[1], params.gyro_bias1[2]};
			AHorse.imu.gyroBias[1] = {params.gyro_bias2[0], params.gyro_bias2[1], params.gyro_bias2[2]};
			AHorse.imu.magBias2 = {params.mag_bias[0], params.mag_bias[1], params.mag_bias[2]};
			AHorse.imu.magScale2 = {params.mag_scales[0], params.mag_scales[1], params.mag_scales[2]};
			
			for(int i = 0; i < 5; i++){
				Radio.maxPwms[i] = params.rc[i].maxPWM;
				Radio.minPwms[i] = params.rc[i].minPWM;
				Radio.rcCenters[i] = (Radio.maxPwms[i] + Radio.minPwms[i])/2.0;
			}
			
			Commander.kps[0] = params.roll_rate.kp; Commander.kps[1] = params.pitch_rate.kp; Commander.kps[2] = params.yaw_rate.kp;
			Commander.kis[0] = params.roll_rate.ki; Commander.kis[1] = params.pitch_rate.ki; Commander.kis[2] = params.yaw_rate.ki;
			Commander.kds[0] = params.roll_rate.kd; Commander.kds[1] = params.pitch_rate.kd; Commander.kds[2] = params.yaw_rate.kd;
		}
		
		std::array<gpio_num_t, ESCInterface::NUM_ESCS> pins = {GPIO_NUM_11, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14};
		escs.init(pins);
		
		if(AHorse.initialMagneticVector[0] == 0.0 && AHorse.initialMagneticVector[1] == 0.0 && AHorse.initialMagneticVector[2] == 0.0){
			printf("Place the drone perfectly level and in the expected initial heading direction. The Vehicle will wait 8 seconds for you to do this.\n");
			vTaskDelay(pdMS_TO_TICKS(8000));
			AHorse.updatePipeline();
		}
	}

	
	// need to program a more reliable "LAND" mode 
	
	
	void run_loop(){
		int c1, c2, c3, c4, c5;
		if(!armed){
			Radio.poll(c1, c2, c3, c4, c5);
			if(c5 == -1 || c5 == this->Radio.minPwms[4]){
				armed = false;
				escs.disarm_all();
				return;
			}
			armed = true;
			escs.arm_all();
		}
		
		AHorse.updatePipeline();
		Radio.poll(c1, c2, c3, c4, c5);
		if(c5 == this->Radio.minPwms[4] || c5 == -1){
			armed = false;
			escs.disarm_all();
			return;
		}
		double range0 = (double)(Radio.maxPwms[0] - Radio.minPwms[0]);
		double range1 = (double)(Radio.maxPwms[1] - Radio.minPwms[1]);
		double range2 = (double)(Radio.maxPwms[2] - Radio.minPwms[2]);
		double range3 = (double)(Radio.maxPwms[3] - Radio.minPwms[3]);
		double thrust = ((double)(c3 - Radio.minPwms[2]))/(range2);
		double yaw = ((double)(c4 - Radio.rcCenters[3]))/(0.5*range3);
		double pitch = ((double)(c2 - Radio.rcCenters[1]))/(0.5*range1);
		double roll = ((double)(c1 - Radio.rcCenters[0]))/(0.5*range0);
		yaw *= maxYawRate; roll *= maxRoll; pitch *= maxPitch;
		Commander.generateCommandsPrincipledWay(thrust, roll, pitch, yaw);
		uint32_t motor1 = (uint32_t)(escmin + Commander.issuedControl[0]*((double)(escmax - escmin)));
		uint32_t motor2 = (uint32_t)(escmin + Commander.issuedControl[1]*((double)(escmax - escmin)));
		uint32_t motor3 = (uint32_t)(escmin + Commander.issuedControl[2]*((double)(escmax - escmin)));
		uint32_t motor4 = (uint32_t)(escmin + Commander.issuedControl[3]*((double)(escmax - escmin)));
		
		motor1 = clipInt(motor1, escmin, escmax);
		motor2 = clipInt(motor2, escmin, escmax);
		motor3 = clipInt(motor3, escmin, escmax);
		motor4 = clipInt(motor4, escmin, escmax);
		
		escs.write_all_us({motor1, motor2, motor3, motor4});
		
	}
	
};



// we gon test components one by one. Lets start with the IMUs

extern "C" void app_main(void){
	
//	i2c_master_bus_config_t bus_cfg = {};
//    bus_cfg.i2c_port = I2C_NUM_0;
//    bus_cfg.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
//    bus_cfg.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
//    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
//    bus_cfg.glitch_ignore_cnt = 7;
//    bus_cfg.flags.enable_internal_pullup = true;
//
//    i2c_master_bus_handle_t bus_handle = nullptr;
//    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));
////
    // 2) add MPU6500 device
//    i2c_device_config_t mpu_cfg = {};
//    mpu_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
//    mpu_cfg.device_address = 0x68;
//    mpu_cfg.scl_speed_hz = I2C_MASTER_FREQ_HZ;
//
//    i2c_master_dev_handle_t mpu_dev = nullptr;
//    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mpu_cfg, &mpu_dev));
//
//    // 3) add ICM20948 device
//    i2c_device_config_t icm_cfg = {};
//    icm_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
//    icm_cfg.device_address = 0x69;
//    icm_cfg.scl_speed_hz = I2C_MASTER_FREQ_HZ;
//
//    i2c_master_dev_handle_t icm_dev = nullptr;
//    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &icm_cfg, &icm_dev));
//
//    vTaskDelay(pdMS_TO_TICKS(200)); // sensor settle time
//
//    // 4) pass device handles into your top-level class
//    AHRS horsey(mpu_dev, icm_dev);
//	vTaskDelay(pdMS_TO_TICKS(1000));
//	float ax = 0.0,ay = 0.0,az = 0.0 ,gx = 0.0 ,gy = 0.0,gz = 0.0 ,mx = 0.0 ,my = 0.0 ,mz = 0.0;
//	horsey.imu.testComms(ax, ay, az, gx, gy, gz, mx, my, mz, 1);
//	printf("Datastic %f %f %f %f %f %f %f %f %f\n", ax, ay, az, gx, gy, gz, mx, my, mz);
////	vTaskDelay(pdMS_TO_TICKS(10000));
//	horsey.imu.calibrateAll();
//	horsey.printCalibration();
//	horsey.updatePipeline();
//	horsey.rotvecFromMatrix();

	// Radio calibration and testing 
	RCInterface Radio;
	Radio.calibrate();
	
	while(true){
		int c1,c2,c3,c4,c5;
		// c1 is roll
		// c2 is pitch 
		// c3 is thrust
		// c4 is yaw
		// c5 is arming
				
		Radio.poll(c1,c2,c3,c4,c5);
		printf("Thrust: %d Roll %d Pitch %d Yaw %d", c3, c1, c2, c4);
		printf("HELPPPPP\n");
//		printf("%f %f %f\n", horsey.rotvec[0], horsey.rotvec[1], horsey.rotvec[2]);
		vTaskDelay(pdMS_TO_TICKS(50));
	}
	vTaskDelay(pdMS_TO_TICKS(200));


//
//	Drone Dronacharya;
//	printf("Wireframe Complete!");
//	vTaskDelay(pdMS_TO_TICKS(1000));	
//	while(1){	
//		Dronacharya.run_loop();
//		vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
//	}
//	
}
