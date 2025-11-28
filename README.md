# Proyecto DHT22 con LoRaWAN y Node-RED

Este proyecto lee datos de temperatura y humedad usando un sensor DHT22 y un ESP32.
Los datos se envían a través de LoRaWAN a The Things Network (TTN) y se visualizan en Node-RED.

## Hardware utilizado
* Placa: TTGO LoRa32 (ESP32)
* Sensor: DHT22 / AM2302

## Carpetas del proyecto
* `src`: Código fuente del firmware (C++).
* `docs`: Manuales y datasheets en PDF.
* `ttn_uplink`: Decodificador Javascript para la consola de TTN.
* `node_red`: Flujos para importar en Node-RED.

## Autor
Subido por xonar7