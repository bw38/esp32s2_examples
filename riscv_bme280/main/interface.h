/*
 * interface.h
 *
 * Geräte spezifikation der Sensoren
 */

#ifndef MAIN_INTERFACE_H_
#define MAIN_INTERFACE_H_

// Testboard ESP32S2 Plug - RISC-V BME280 Main ----------------

#define INTERVAL_MS			5000

#define TEMP_THRESHOLD		0.25	//delta °C
#define HUMI_THRESHOLD		2		//delta %
#define PRES_THRESHOLD		2		//delta hPa
#define MAX_FORCE_REPORT	0		//nach x Messung main spätestens wecken

#define I2C_MASTER_SDA_IO 	8
#define I2C_MASTER_SCL_IO 	9



#endif /* MAIN_INTERFACE_H_ */
