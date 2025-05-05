# Wifi

Esse exemplo é baseado no oficial do mqtt da pico-examples, que indica como fazer envio e recebimento de dados via MQTT para um broker local.

Primeiro, você vai ter que subir um "broker" mqtt na sua máquina, e depois programar a pico (veja a seguir como fazer isso)

Passos:

1. Conecte o seu computador em uma rede local (não pode ser o wifi do insper)
1. Inicialize o broker mosquitto.
3. Agora modifique o `mqtt/mqtt_client.c`:

``` c
#define WIFI_SSID "your-wifi"
#define WIFI_PASSWORD "your-password"
#define MQTT_SERVER "your-ip"
```
4.  Agora execute o código na rasp, você deve observar o envio dos dados.

## MQTT Broker


To use this example you will need to install an MQTT server on your network.
To install the Mosquitto MQTT client on a Raspberry Pi Linux device run the following...

```
sudo apt install mosquitto
sudo apt install mosquitto-clients
```

Check it works...

```
mosquitto_pub -t test_topic -m "Yes it works" -r
mosquitto_sub -t test_topic -C 1
```

To allow an external client to connect to the server you will have to change the configuration. Add the following to /etc/mosquitto/conf.d/mosquitto.conf

```
allow_anonymous true
listener 1883 0.0.0.0
```

Then restart the service.

```
sudo service mosquitto restart
```

The example should publish its core temperature to the /temperature topic. You can subscribe to this topic from another machine.

```
mosquitto_sub -h $MQTT_SERVER -t '/temperature'
```

You can turn the led on and off by publishing messages.

```
mosquitto_pub -h $MQTT_SERVER -t '/led' -m on
mosquitto_pub -h $MQTT_SERVER -t '/led' -m off
```

