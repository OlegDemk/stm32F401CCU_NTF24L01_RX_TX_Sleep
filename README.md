The project base on STM32F401CCU microcontroller and NRF24L01 radion module. Project was create on STM32 Cube IDE. 
The RX project: TR all time listen TX modules(up to 6), mark from witch PIPE was(from what TX module data transmeeted), and transmeet reseived data into UART. 

The TX project: Transmeet this data: (32 bytes/one packet)
1. Temperature, Humidity from BME280.
2. Battery voltage. 
3. Retransmitted data packets counter and lost data packets counter. (need for detect lost packets).
After data transmeeting, TX devise go into Stenby mode.
The power consumption(STM32+NRF24l01+BME280) into this mode around 37uA, in condition when device woke up every 5 seconds, and transmeet data every 60 seconds.
