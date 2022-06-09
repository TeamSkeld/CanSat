# CanSat
Arduino code for the ESP-32 in the can. 

Pinouts:

| BME-280 |  |  |  |  |  
| :---------:| :---: | :---: | :---: | :---: |                   
| Module Pin | VIN   | GND   | SCL  | SDA  |                   
| ESP-32 Pin | 3V3    | GND   | G22   | G21   |                  
                                                                
| MPU-6050 |  |  |  |  |                                        
| :---------:| :---: | :---: | :---: | :---: |                 
| Module Pin | VCC   | GND   | SCL  | SDA  |                     
| ESP-32 Pin | 3V3    | GND   | G22   | G21   |                  
                                                                
| GPS Module |  |  |  |  |                                        
| :---------:| :---: | :---: | :---: | :---: |                  
| Module Pin | 3.3V   | GND   | TX  | RX  |                     
| ESP-32 Pin | 3V3   | GND   | G16   | G17   |

| SD Card Adapter|  |  |  |  |  |  |
| :---------:| :---: | :---: | :---: | :---: | :---: | :---: |
| Module Pin | VCC   | GND   | MOSI  | MISO  | SCK   | CS    |
| ESP-32 Pin | 5V    | GND   | G23   | G19   | G18   | G5    |

| APC-220 |  |  |  |  | 
| :---------:| :---: | :---: | :---: | :---: |
| Module Pin | VCC   | GND   | TX  | RX  |
| ESP-32 Pin | 5V    | GND   | G25   | G26   |


| Libraries used  |
| :-------------: |
| Wire            |
| Adafruit_Sensor |
| Adafruit_BME280 |
| Adafruit_MPU6050 |
| SPI  |
| SD |  
| EEPROM |
| Adafruit_GPS | 
| HardwareSerial |
    
