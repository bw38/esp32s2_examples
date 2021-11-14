# esp32s2 examples
## Examples for ULP RISC-V processor 

Handle some sensorboards for weather oberservation while deepsleep of main processor

 - bme280
 - sht30/31 (soon)

The rtc-i2c-api is not implemented in IDF v4.4, so a bit by bit communication is used.<br>
The mainfocus ist set to RISC-V process. the main-CPU only initialises the ULP-Process und prints the results.
