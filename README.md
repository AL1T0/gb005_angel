| Supported Targets | ESP32 | ESP32-C3 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- |

# GB005 Firmware 

This application connects to the broker api.allthingstalk.io (AllThingsTalk IoT Platform) using ssl transport and subscribes to the compressor command topic, then reads temperature, CO2, pH and light sensors every 30 seconds and publishes the values to specific topics for each measurment.

This application is based on the ESP-MQTT SSL Sample application. It uses ESP-MQTT library which implements mqtt client to connect to mqtt broker.

## How to use

### Hardware Required

This program was designed to run on a WeMos D1 Mini ESP32 board, but can be adapted to any other ESP32 board, changing the used pins to connect the sensors.

### Configure the project

* Open the project configuration menu (`idf.py menuconfig`)
* Configure Wi-Fi or Ethernet under "Example Connection Configuration" menu. See "Establishing Wi-Fi or Ethernet Connection" section in [examples/protocols/README.md](../../README.md) for more details.

CA certificate for this application have to be requested to AllThingsTalk support and should be saved in the project directory as "att.pem".

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

```
I (3714) event: sta ip: 192.168.0.139, mask: 255.255.255.0, gw: 192.168.0.2
I (3714) system_api: Base MAC address is not set, read default base MAC address from BLK0 of EFUSE
I (3964) MQTT_CLIENT: Sending MQTT CONNECT message, type: 1, id: 0000
I (4164) MQTT_TEST: MQTT_EVENT_CONNECTED
I (4174) MQTT_TEST: sent publish successful, msg_id=41464
I (4174) MQTT_TEST: sent subscribe successful, msg_id=17886
I (4174) MQTT_TEST: sent subscribe successful, msg_id=42970
I (4184) MQTT_TEST: sent unsubscribe successful, msg_id=50241
I (4314) MQTT_TEST: MQTT_EVENT_PUBLISHED, msg_id=41464
I (4484) MQTT_TEST: MQTT_EVENT_SUBSCRIBED, msg_id=17886
I (4484) MQTT_TEST: sent publish successful, msg_id=0
I (4684) MQTT_TEST: MQTT_EVENT_SUBSCRIBED, msg_id=42970
I (4684) MQTT_TEST: sent publish successful, msg_id=0
I (4884) MQTT_TEST: deliver_publish, message_length_read=19, message_length=19
I (4884) MQTT_TEST: MQTT_EVENT_DATA
I (5194) MQTT_CLIENT: deliver_publish, message_length_read=19, message_length=19
I (5194) MQTT_TEST: MQTT_EVENT_DATA
  T: 22.938 degC
I (6496463) MQTT_TEST: sent publish successful, msg_id=0
I (6558273) MQTT_TEST: MQTT_EVENT_DATA
TOPIC=device/XXX/asset/actcompr/command
DATA={"at":"2023-09-16T23:29:50.084014Z","value":false,"meta":null}
I (6558283) I/O TEST: Outputs set to OFF
```

