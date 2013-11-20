renesas_mqtt
============

1) Start application for auto provisioning by holding switch 1 while
the device start.  Then follow the instructions on the LCD.  This will
involve connecting to the AP created by the device, searching for the
SSID you want to connect to, and then entering the passkey for that
WiFi network.  This is currently configured for WPA/WPA2 PSK networks
only.

2) Restart the program without holding the switch.  Now the device
will connect to the MQTT broker specified in the code and subscribed
to the specified topic
(M2MIO_DOMAIN/M2MIO_DEVICE_TYPE/M2MIO_SUBSCRIBE_TOPIC).  The payload
of received messages are displayed to the LCD.
