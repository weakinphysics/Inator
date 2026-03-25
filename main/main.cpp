#include <cstdint>
#include <stdio.h>
#include <stdbool.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>
#include <string>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "hal/i2c_types.h"
#include "icm20948_enumerations.h"
#include "esp_check.h"
extern "C" {
	#include "icm20948.h"
	#include "icm20948_i2c.h"
}

#define I2C_MASTER_SCL_IO   22
#define I2C_MASTER_SDA_IO   21
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000	


///// SCALES FOR ACCELS AND GYROS, COMMON TO BOTH IMUs

float a2 = 16384.0f;
float a4 = 8192.0f;
float a8 = 4096.0f;
float a16 = 2048.0f;
		
float g250 = 131.0f;
float g500 = 65.5f;
float g1000 = 32.8f;
float g2000 = 16.4f;

//////////////////////////////////////////////////////

static void i2c_master_init()
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}



class NineAxis{
	public:
	icm20948_device_t imu;
	icm0948_config_i2c_t imu_config;
	icm20948_agmt_t agmt;
	
	NineAxis(){
		imu_config.i2c_port = I2C_MASTER_NUM;
		imu_config.i2c_addr = 0x69;
		icm20948_status_e status;
	    icm20948_init_struct(&imu);
		icm20948_init_i2c(&imu, &imu_config);
		status = icm20948_check_id(&imu);
		printf("check_id status = %d\n", status);
		if (status != ICM_20948_STAT_OK) {
	        printf("ICM20948 not found\n");
	        return;
	    }
		// IMPORTANT: wake device
	    status = icm20948_sleep(&imu, false);
	    printf("sleep(false) status = %d\n", status);
	    vTaskDelay(pdMS_TO_TICKS(50));

	    // optional but good
	    status = icm20948_set_clock_source(&imu, CLOCK_AUTO);
	    printf("set_clock_source status = %d\n", status);
	    status = icm20948_set_sample_mode(
	        &imu,
	        (icm20948_internal_sensor_id_bm)(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR),
	        SAMPLE_MODE_CONTINUOUS
	    );
	    printf("set_sample_mode status = %d\n", status);
	    icm20948_fss_t fss = {};
	    fss.a = GPM_2;
	    fss.g = DPS_250;
	    status = icm20948_set_full_scale(
	        &imu,
	        (icm20948_internal_sensor_id_bm)(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR),
	        fss
	    );
	    printf("set_full_scale status = %d\n", status);
	}
	

	esp_err_t fetchAll(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &mx, float &my, float &mz, bool getMags=false){
		icm20948_status_e status = icm20948_data_ready(&imu);
		if (status == ICM_20948_STAT_OK){
			status = icm20948_get_agmt(&imu, &agmt);
			printf("Get AGMT status %d\n", status);
			ax = agmt.acc.axes.x / a2;
			ay = agmt.acc.axes.y / a2;
			az = agmt.acc.axes.z / a2;
			gx = agmt.gyr.axes.x / g250;
			gy = agmt.gyr.axes.y / g250;
			gz = agmt.gyr.axes.z / g250;
			if(getMags){
				mx = agmt.mag.axes.x*0.15f;
				my = agmt.mag.axes.y*0.15f;
				mz = agmt.mag.axes.z*0.15f;
			}
		}
		return ESP_OK;
	}    

};







class MPU6500{
	public:
	static constexpr uint8_t ADDR_LOW = 0x68;
	static constexpr uint8_t ADDR_HIGH = 0x69;
	static constexpr uint8_t REG_SMPLRT_DIV = 0x19;
	static constexpr uint8_t REG_CONFIG = 0x1A;
	static constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
	static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
	static constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D;
	static constexpr uint8_t REG_ACCEL_XOUT_H= 0x3B;
	static constexpr uint8_t REG_ACCEL_XOUT_L = 0x43;
	static constexpr uint8_t REG_PWR_MGMT1 = 0x6B;
	static constexpr uint8_t REG_PWR_MGMT2 = 0x6C;
	static constexpr uint8_t REG_WHO_AM_I= 0x70;
	
	i2c_port_t port{};
	uint8_t addr{ADDR_LOW};
	
	
	
	esp_err_t init(i2c_port_t i2c_port, uint8_t i2c_addr) {
        port = i2c_port;
        addr = i2c_addr;

        uint8_t who = 0;
        ESP_RETURN_ON_ERROR(read_reg(REG_WHO_AM_I, &who, 1), "MPU6500", "WHO_AM_I read failed");
        printf("MPU6500 WHO_AM_I = 0x%02X\n", who);

        if (who != 0x70) {
            printf("Unexpected WHO_AM_I\n");
        }

        // Wake up, use PLL with X gyro as clock source.
        ESP_RETURN_ON_ERROR(write_reg(this->REG_PWR_MGMT1, 0x01), "MPU6500", "wake failed");
        vTaskDelay(pdMS_TO_TICKS(50));

        // Enable accel + gyro on all axes.
        ESP_RETURN_ON_ERROR(write_reg(this->REG_PWR_MGMT2, 0x00), "MPU6500", "PWR_MGMT_2 failed");

        // DLPF config. Mild filtering.
        ESP_RETURN_ON_ERROR(write_reg(this->REG_CONFIG, 0x03), "MPU6500", "CONFIG failed");

        // Gyro full scale = ±250 dps
        ESP_RETURN_ON_ERROR(write_reg(this->REG_GYRO_CONFIG, 0x00), "MPU6500", "GYRO_CONFIG failed");

        // Accel full scale = ±2 g
        ESP_RETURN_ON_ERROR(write_reg(this->REG_ACCEL_CONFIG, 0x00), "MPU6500", "ACCEL_CONFIG failed");

        // Accel DLPF on, moderate bandwidth
        ESP_RETURN_ON_ERROR(write_reg(this->REG_ACCEL_CONFIG2, 0x03), "MPU6500", "ACCEL_CONFIG2 failed");

        // Sample rate divider
        ESP_RETURN_ON_ERROR(write_reg(this->REG_SMPLRT_DIV, 0x04), "MPU6500", "SMPLRT_DIV failed");

        vTaskDelay(pdMS_TO_TICKS(50));
        return ESP_OK;
    }

    esp_err_t read_accel_gyro_raw(int16_t& ax, int16_t& ay, int16_t& az,
                                  int16_t& gx, int16_t& gy, int16_t& gz) {
        uint8_t buf[14] = {0};
        ESP_RETURN_ON_ERROR(read_reg(REG_ACCEL_XOUT_H, buf, 14), "MPU6500", "burst read failed");

        ax = (int16_t)((buf[0]  << 8) | buf[1]);
        ay = (int16_t)((buf[2]  << 8) | buf[3]);
        az = (int16_t)((buf[4]  << 8) | buf[5]);

        // buf[6], buf[7] are temperature

        gx = (int16_t)((buf[8]  << 8) | buf[9]);
        gy = (int16_t)((buf[10] << 8) | buf[11]);
        gz = (int16_t)((buf[12] << 8) | buf[13]);

        return ESP_OK;
    }

    esp_err_t fetchAll(float& ax_g, float& ay_g, float& az_g,
                              float& gx_dps, float& gy_dps, float& gz_dps) {
        int16_t ax, ay, az, gx, gy, gz;
        ESP_RETURN_ON_ERROR(read_accel_gyro_raw(ax, ay, az, gx, gy, gz), "MPU6500", "raw read failed");
        // for ±2g and ±250 dps
        ax_g   = ax / a2;
        ay_g   = ay / a2;
        az_g   = az / a2;
        gx_dps = gx / g250;
        gy_dps = gy / g250;
        gz_dps = gz / g250;
		
        return ESP_OK;
    }

private:
    esp_err_t write_reg(uint8_t reg, uint8_t value) {
        uint8_t data[2] = {reg, value};
        return i2c_master_write_to_device(port, addr, data, 2, pdMS_TO_TICKS(100));
    }

    esp_err_t read_reg(uint8_t reg, uint8_t* data, size_t len) {
        return i2c_master_write_read_device(port, addr, &reg, 1, data, len, pdMS_TO_TICKS(100));
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
	IMUInterface(){
		// this is a constructor 
		printf("break bread with the enemy?");
		this->accels = {0.0, 0.0, 0.0};
		this->gyros = {0.0, 0.0, 0.0};
		this->mags = {0.0, 0.0, 0.0};
		this->accelBias.resize(2, std::vector <double> (3, 0.0));
		this->gyroBias.resize(2, std::vector<double>(3, 0.0));
		this->accelScale.resize(2, std::vector<double>(3, 0.0));
		this->magScale2.resize(3, 0);
		this->magBias2.resize(3, 0);		
		imu1.init(I2C_MASTER_NUM, 0x68);
		this->calibrateAll();
	}
	
	void calibrateAccelGyro(int which_imu){
		// for now this will print to the console at the beginning of every phase
		printf("Starting Accel Calibration, place the device z up");
		vTaskDelay(pdMS_TO_TICKS(5000));
		float ax, ay, az, gx, gy, gz, mx, my, mz;
		float accelsd1[3] = {0.0, 0.0, 0.0};
		float accelsd2[3] = {0.0, 0.0, 0.0};
		for(int i = 0; i < 1000; i++){	
			esp_err_t e;
			if(which_imu == 0) e = imu1.fetchAll(ax, ay, az, gx, gy, gz);
			else e = imu2.fetchAll(ax, ay, az, gx, gy, gz, mx, my, mz, false);
			accelsd1[0] += ax;
		}
		accelsd1[0] /= 1000.0;
		printf("Place the device z down");
		vTaskDelay(pdMS_TO_TICKS(5000));
		for(int i = 0; i < 1000; i++){
			esp_err_t e;
			if(which_imu == 0) e = imu1.fetchAll(ax, ay, az, gx, gy, gz);
			else e = imu2.fetchAll(ax, ay, az, gx, gy, gz, mx, my, mz, false);
			accelsd2[0] += ax;
		}
		accelsd2[0] /= 1000.0; 
		this->accelBias[which_imu][0]= (accelsd1[0] + accelsd2[0])/2.0;
		this->accelScale[which_imu][0]= (accelsd1[0] - accelsd2[0])/2.0;
			
		//////
		printf("Place the device y up");
		for(int i = 0; i < 1000; i++){	
			esp_err_t e;
			if(which_imu == 0) e = imu1.fetchAll(ax, ay, az, gx, gy, gz);
			else e = imu2.fetchAll(ax, ay, az, gx, gy, gz, mx, my, mz, false);
			accelsd1[1] += ay;
		}
		accelsd1[1] /= 1000.0;
		printf("Place the device y down");
		vTaskDelay(pdMS_TO_TICKS(5000));
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
		
		printf("Place the device x up");
		for(int i = 0; i < 1000; i++){	
			esp_err_t e;
			if(which_imu == 0) e = imu1.fetchAll(ax, ay, az, gx, gy, gz);
			else e = imu2.fetchAll(ax, ay, az, gx, gy, gz, mx, my, mz, false);
			accelsd1[2] += az;
		}
		accelsd1[2] /= 1000.0;
		printf("Place the device x down");
		vTaskDelay(pdMS_TO_TICKS(5000));
		for(int i = 0; i < 1000; i++){
			esp_err_t e;
			if(which_imu == 0) e = imu1.fetchAll(ax, ay, az, gx, gy, gz);
			else e = imu2.fetchAll(ax, ay, az, gx, gy, gz, mx, my, mz, false);
			accelsd2[2] += az;
		}
		accelsd2[2] /= 1000.0; 
		this->accelBias[which_imu][2]= (accelsd1[2] + accelsd2[2])/2.0;
		this->accelScale[which_imu][2]= (accelsd1[2] - accelsd2[2])/2.0;
		
		
		// gyro cal 
		printf("Place the device level and z up to calibrate Gyros");
		vTaskDelay(pdMS_TO_TICKS(5000));
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
		printf("Calibrating mags, keep moving !");
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
	public:
	
	IMUInterface imu;
	std::vector <std::vector<double>> rotationMatrix;
	std::vector <double> quaternion;
	std::vector <double> rotvec;
	std::vector <std::vector<double>> hatTarget;
	AHRS(){
		
	}
	
	void hat(std::vector <double> &v){
		
	}
	
};


class Controller{
	// very low level controller, PID based
	Controller(){
		
	}
};


class Drone{
	Drone(){
		
	}
};

extern "C" void app_main(void){
    i2c_master_init();
    vTaskDelay(pdMS_TO_TICKS(100));

    while(1){
		printf("AHRS To be Implemented next!");
		vTaskDelay(pdMS_TO_TICKS(1000));	
		
	}
}
