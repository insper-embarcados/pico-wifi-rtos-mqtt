## Comunicação Wi-Fi com MQTT

Este exemplo é baseado no repositório oficial [`pico-examples`](https://github.com/raspberrypi/pico-examples) e demonstra como enviar e receber dados utilizando o protocolo MQTT via Wi-Fi com a Raspberry Pi Pico `W`.  

### Objetivo

Você vai aprender como:  

- Configurar um broker MQTT local (Mosquitto);  
- Conectar a Pico W à rede Wi-Fi;  
- Enviar dados (ex: temperatura);
- Receber comandos (ex: ligar/desligar LED).  
  
### Step-by-Step

#### 1. Conecte-se à rede local  

> ⚠️ Não pode ser o wifi do insper


#### 2. Instale o Mosquitto

```bash
sudo apt update
sudo apt install mosquitto mosquitto-clients
```
no MAC 

```bash
brew install mosquitto
brew services start mosquitto                    
```
no windowns

- https://mosquitto.org/download/


### 3. Teste o Mosquitto 

Em um terminal digite: 

```bash
mosquitto_pub -t test_topic -m "Yes it works" -r
mosquitto_sub -t test_topic -C 1
```

Se funcionar, está tudo certo!

### 4. Permita conexões externas 

Edite o arquivo de configuração:  

no linux: 

```bash
sudo nano /etc/mosquitto/conf.d/mosquitto.conf
```

no MAC:

```bash
code /opt/homebrew/etc/mosquitto/mosquitto.conf
```

Adicione:  

```
allow_anonymous true
listener 1883 0.0.0.0
```

Reinicie o serviço:  

no Linux:
```bash
sudo service mosquitto restart
```
no MAC:
```bash
brew services restart mosquitto    
```

### 5. Ajuste o código 

No arquivo `mqtt/mqtt_client.c`, atualize:

```c
#define WIFI_SSID "your-wifi"  // sua rede wifi
#define WIFI_PASSWORD "your-password"  // sua senha
#define MQTT_SERVER "your-ip"  // ip wifi do seu PC (para descobrir ipconfig) 
```

Compile e carregue o código para a Pico.  

### 6. Teste com tópicos MQTT 

A Pico irá publicar temperatura no tópico `/temperature`:  

```bash
mosquitto_sub -h [IP-DO-BROKER] -t '/temperature'
```

Envie comandos para o LED:  

```bash
# Para ligar o LED
mosquitto_pub -h [IP-DO-BROKER] -t '/led' -m on

# Para desligar o LED
mosquitto_pub -h lIP-DO-BROKER] -t '/led' -m off
```
Observando o status do LED:

```bash
mosquitto_sub -h localhost -t "/led/status"
```

## 7. Utilizando com o HiveMQ (broker online gratuito)

Caso prefira usar um broker público como o [HiveMQ](https://www.hivemq.com/public-mqtt-broker/), será necessário alterar o código da Pico para conectar ao servidor do HiveMQ:

No arquivo `mqtt/mqtt_client.c`, substitua:

```c
#define MQTT_SERVER "your-ip"
```
por:

```c
 #define MQTT_SERVER "broker.hivemq.com"  
```

> ⚠️ Como o HiveMQ é um broker público, evite compartilhar dados sensíveis.
> Use tópicos únicos (ex: com seu nome ou ID) para evitar conflitos com outros usuários.
> troque por exemplo /temperature por `/insper/seu_nome/temperature`

acesse o websocket [https://www.hivemq.com/demos/websocket-client/](https://www.hivemq.com/demos/websocket-client/) e faça a conexão para publicar e subscrever os tópicos.


