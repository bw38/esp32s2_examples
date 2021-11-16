/*
 * bme280_i2c_if.c
 *
 *  Created on: 2021-11-14
 *  Author    : Joerg / DL7VMD
 *	License   : MIT
 *
 *  use actual driver from:
 *  https://github.com/BoschSensortec/BMP2-Sensor-API/
 *
*/

//relative Paths not found in Eclipse / IDF is ok
#include "ulp_riscv/ulp_riscv.h"
#include "ulp_riscv/ulp_riscv_utils.h"
#include "ulp_riscv/ulp_riscv_gpio.h"

#include "BMP2-Sensor-API-master/bmp2.h"
#include "BMP2-Sensor-API-master/bmp2_defs.h"

/*
 * #define BME280_32BIT_ENABLE	!!! set in bme280_defs.h !!!
 */


//internal varibales
static uint8_t dev_addr;
static uint8_t first_run = 0;

//cache wakeup check
static uint32_t mcycle = 0;
static uint32_t mtemp = 0;
static uint32_t mhumi = 0;
static uint32_t mpres = 0;


//external variables, access from main => ulp_xxxxx
//results to main
uint32_t bmp2_temperature;
uint32_t bmp2_pressure;
uint32_t bmp2_status;
uint32_t bmp2_chip_id = 0;
uint32_t bmp2_acquisition_time_ms;
uint32_t bmp2_cycles = 0;
//set individual from main
uint32_t bmp2_sda = 0;
uint32_t bmp2_scl = 0;
uint32_t set_bmp2_force_wake = 0;
uint32_t set_bmp2_thres_temp = 0;
uint32_t set_bmp2_thres_pres = 0;


// HW - Initialisierung --------------------------------------------

static void init_gpio() {
	// Setup GPIO für bitweise I2C
    ulp_riscv_gpio_init(bmp2_sda);
    ulp_riscv_gpio_input_enable(bmp2_sda);
    ulp_riscv_gpio_set_output_mode(bmp2_sda, RTCIO_MODE_OUTPUT_OD);
    ulp_riscv_gpio_pullup_disable(bmp2_sda);
    ulp_riscv_gpio_pulldown_disable(bmp2_sda);
    ulp_riscv_gpio_output_level(bmp2_sda, 0);

    ulp_riscv_gpio_init(bmp2_scl);
    ulp_riscv_gpio_input_enable(bmp2_scl);
    ulp_riscv_gpio_set_output_mode(bmp2_scl, RTCIO_MODE_OUTPUT_OD);
    ulp_riscv_gpio_pullup_disable(bmp2_scl);
    ulp_riscv_gpio_pulldown_disable(bmp2_scl);
    ulp_riscv_gpio_output_level(bmp2_scl, 0);
}


// I2C - Bit & Byte - level ----------------------------------------
//Grundstellung High -> Input (ext. PullUp) / aktiv Low => Input & Output (fix low)
#define SCL_L		ulp_riscv_gpio_output_enable(bmp2_scl)
#define SCL_H		ulp_riscv_gpio_output_disable(bmp2_scl)
#define X_SCL		ulp_riscv_gpio_get_level(bmp2_scl)
#define SDA_L		ulp_riscv_gpio_output_enable(bmp2_sda)
#define SDA_H		ulp_riscv_gpio_output_disable(bmp2_sda)
#define X_SDA		ulp_riscv_gpio_get_level(bmp2_sda)

#define CLK			20 	//ca. 20kHz Takt (measured)
#define T25			ulp_riscv_delay_cycles(CLK / 4)

static void tx_start_bit() {
	SDA_H; T25; SCL_H; T25; SDA_L; T25;	// -> SDA_L
}

static void tx_stop_bit() {	//SCL_H
	T25; SCL_L; SDA_L; T25; SCL_H; T25; SDA_H;
}

static void tx_1_bit() {	//SCL H
	T25; SCL_L; T25; SDA_H; T25; SCL_H; T25;
}

static void tx_0_bit() {	//SCL H
	T25; SCL_L; T25; SDA_L; T25; SCL_H; T25;	// -> SDA_L
}

static uint8_t rx_bit() {
	uint8_t x;
	T25;  SCL_L; SDA_H; T25; x = X_SDA; SCL_H;  T25;
	return x;
}

static void tx_ack_bit() {
	T25; SCL_L; T25; SDA_L; T25; SCL_H; T25;	// ->SDA_L
}

static void tx_nack_bit() {
	T25; SCL_L; T25; SDA_H; T25; SCL_H; T25;
}

static void tx_byte (uint8_t x) {
	for (int i = 0; i < 8; i++) {
		if ((x & 0x80) == 0) tx_0_bit();
		else tx_1_bit();
		x = x << 1;
	}
}


// I2C - adjustment to BME280.h --------------------------------
void user_delay_us(uint32_t period, void *intf_ptr) {
	ulp_riscv_delay_cycles(period * ULP_RISCV_CYCLES_PER_US);
}

int8_t bmp2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *((uint8_t *)intf_ptr);

    tx_start_bit();
    tx_byte((dev_addr << 1) & 0xFE);
    uint8_t x = rx_bit();
    tx_byte(reg_addr);
    x |= rx_bit();
    for (int i = 0; i < len; i++) {
    	tx_byte(reg_data[i]);
    	x |= rx_bit();
    }
    tx_stop_bit();
    return x;
}


int8_t bmp2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *((uint8_t *)intf_ptr);
    //send source-addr to slave, without data
    uint8_t x = bmp2_i2c_write(reg_addr, NULL, 0, &dev_addr);
    //sens slave-adr senden for preface reading process
    tx_start_bit();
    tx_byte((dev_addr << 1) | 0x01);
    x |= rx_bit();
    //read data sequentially
	uint8_t y;
	for (int j = 0; j < len; j++) {
		for (int i = 0; i < 8; i++) {
			y <<= 1;
			y |= rx_bit();
		}
		reg_data[j] = y;
		if (j < (len-1)) tx_ack_bit();
	}
	tx_nack_bit();	//last readed byte -> acknowledge with NACK
    tx_stop_bit();
    return x;
}


// --------------------------------------------------------------------------------------------

//initialise chip after coldstart or device error
int8_t bmp2_i2c_init(struct bmp2_dev *dev) {
	dev_addr = BMP2_I2C_ADDR_PRIM;	//SDO 10k Pulldown on Sensorboard => addr = 0x76
	dev->intf_ptr = &dev_addr;
	dev->intf  = BMP2_I2C_INTF;
	dev->read  = bmp2_i2c_read;
	dev->write = bmp2_i2c_write;
	dev->delay_us = user_delay_us;
	//safe side -> SW-Reset
//	const uint8_t com_res = BMP2_SOFT_RESET_CMD;
//	bmp2_i2c_write(BMP2_REG_SOFT_RESET, &com_res, 1, dev);
//	user_delay_us(5000, NULL); // > 2ms PowerOn-Reset

	return bmp2_init(dev);
}




//perform measurement an readinfg sensor-data
//duration about 70ms
int8_t stream_sensor_data_forced_mode(struct bmp2_dev *dev) {
	int8_t rslt;
	struct bmp2_config conf;
	rslt = bmp2_get_config(&conf, dev);
	conf.filter = BMP2_FILTER_OFF;
	conf.os_mode = BMP2_OS_MODE_HIGH_RESOLUTION;
	conf.odr = BMP2_ODR_250_MS;
	rslt = bmp2_set_config(&conf, dev);
	rslt = bmp2_set_power_mode(BMP2_POWERMODE_FORCED, &conf, dev);
	uint32_t t_sampling;
	bmp2_compute_meas_time(&t_sampling, &conf, dev);

    int8_t idx = 1;
    struct bmp2_status status;
    struct bmp2_data comp_data;

    dev->delay_us(t_sampling, NULL);

    rslt = bmp2_get_status(&status, dev);
    if (status.measuring == BMP2_MEAS_DONE) {
    	rslt = bmp2_get_sensor_data(&comp_data, dev);
        bmp2_temperature = comp_data.temperature,
        bmp2_pressure = comp_data.pressure;
    }
	return (rslt << 2);
}


int main (void) {
	bmp2_cycles++;
	init_gpio();
	int8_t rslt = 0;
	struct bmp2_dev dev;

	if (first_run == 0) {
		rslt = bmp2_i2c_init(&dev);
		first_run = 1;
	}
	//Chip-ID zur Info
	bmp2_chip_id =  dev.chip_id;

//	rslt |= stream_sensor_data_forced_mode(&dev);

	if (bmp2_pressure == 0) rslt |= 1 << 3;				// check plausibility
	bmp2_status = rslt;									// 0 == Ok
	if (rslt != 0) first_run = 0; //force reset

    //test wakeup conditions
    if ((++mcycle >= set_bmp2_force_wake) ||							//max cycles since last main-wakeup
    	(abs(mtemp - bmp2_temperature) >= set_bmp2_thres_temp) ||	//delta temp
		(abs(mpres - bmp2_pressure) >= set_bmp2_thres_pres)) {		//delta pres
    		mtemp = bmp2_temperature;
    		mpres = bmp2_pressure;
    		mcycle = 0;
    	ulp_riscv_wakeup_main_processor();
    }

	ulp_riscv_wakeup_main_processor();
}
