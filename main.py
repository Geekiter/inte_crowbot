import network
import utime
from micropyserver import MicroPyServer
import utils
from umqtt.simple import MQTTClient
import machine
from utils import log
import json
from machine import Pin, ADC, PWM
import time
import neopixel


def release_wifi():
    sta_if = network.WLAN(network.STA_IF)

    if sta_if.isconnected():
        sta_if.disconnect()
        log("Disconnected from Wi-Fi network.", "INFO")

    sta_if.active(False)
    log("STA interface disabled. Wi-Fi cache released.", "INFO")


def connect_wifi(wifi_name, wifi_password):
    global wifi_wait_time
    wifi_connect = False
    sta_if = network.WLAN(network.STA_IF)

    release_wifi()
    if not sta_if.isconnected():
        try:
            sta_if.active(True)
            sta_if.connect(wifi_name, wifi_password)
            while not sta_if.isconnected():
                utime.sleep(1)

                wifi_wait_time += 1
                if wifi_wait_time >= 10:
                    raise Exception("Connection timeout")

            wifi_connect = True

            log("Connected to Wi-Fi network.", "INFO")
            log("Network config: {}".format(sta_if.ifconfig()), "INFO")
        except Exception as e:
            log("Failed to connect to Wi-Fi network: {}".format(e), "ERROR")
            log("Network config: {}".format(sta_if.ifconfig()), "INFO")
            wifi_connect = False
    return wifi_connect


def restart_and_reconnect():
    print("Failed to connect to MQTT broker. Reconnecting...")
    utime.sleep(10)
    machine.reset()


def connect_and_subscribe(clientID, myTopic):
    try:
        client = MQTTClient(
            client_id=clientID, server=serverIP, port=port, keepalive=6000
        )
        client.set_callback(MsgOK)
        client.connect()
        client.subscribe(myTopic)
        log("Connected to MQTT server at {}:{}".format(serverIP, port), "INFO")
        return client
    except Exception as e:
        log("Failed to connect to MQTT server: " + str(e), "ERROR")
        restart_and_reconnect()


def connect_show_params(client, request):
    global mqtt_client
    global run
    global serverIP
    global port
    global machineId
    global server
    global clientID
    global myTopic

    """ request handler """
    params = utils.get_request_query_params(request)
    log("connect show params: {}".format(params))
    ips = params["mqtt_ip"].split(":")
    serverIP = ips[0]
    port = ips[1]
    """ will return {"param_one": "one", "param_two": "two"} """
    server.send(client, "HTTP/1.0 200 OK\r\n")
    server.send(client, "Content-Type: text/html\r\n\r\n")
    if machineId != params["machineid"]:
        return server.send(client, "Not this car")
    if run == True:
        return server.send(client, "mqtt is connected!")
    try:
        mqtt_client = connect_and_subscribe(clientID, myTopic)
        server.send(client, "ok")
        run = True
        # server.stop()
    except OSError as e:
        server.send(client, "failed")


def stop_show_params(client, request):
    global mqtt_client
    global run
    global machineId
    """ request handler """
    params = utils.get_request_query_params(request)
    print(params)
    server.send(client, "HTTP/1.0 200 OK\r\n")
    server.send(client, "Content-Type: text/html\r\n\r\n")
    if machineId != params["machineid"]:
        return server.send(client, "Not this car")
    if run != True:
        return server.send(client, "No mqtt connected!")
    try:
        mqtt_client.disconnect()
        server.send(client, "ok")
        run = False
    except OSError as e:
        server.send(client, "failed")


def status_show_params(client, request):
    global run, serverIP, port, machineId, server
    """ request handler """
    params = utils.get_request_query_params(request)
    print(params)
    if machineId != params["machineid"]:
        server.send(client, "HTTP/1.0 200 OK\r\n")
        server.send(client, "Content-Type: text/html\r\n\r\n")
        return server.send(client, "Not this car")
    json_str = json.dumps({"run": run, "mqtt_ip": "{}:{}".format(serverIP, port)})
    server.send(client, "HTTP/1.0 200 OK\r\n")
    server.send(client, "Content-Type: application/json\r\n\r\n")
    server.send(client, json_str)


def motor(A1, A2, B1, B2):
    global motor1, motor2, motor3, motor4
    motor1.duty(A1)
    motor2.duty(A2)
    motor3.duty(B1)
    motor4.duty(B2)


def action_forward():
    motor(0, 800, 0, 800)
    utime.sleep(0.5)
    motor(0, 0, 0, 0)


def action_backward():
    motor(800, 0, 800, 0)
    utime.sleep(0.5)
    motor(0, 0, 0, 0)


def action_left():
    motor(800, 0, 0, 0)
    utime.sleep(0.2)
    motor(0, 0, 0, 0)


def action_right():
    motor(0, 0, 800, 0)
    utime.sleep(0.2)
    motor(0, 0, 0, 0)

def action_greeting():
    global buzzer
    global Song2
    global Beat2
    for i in range(0, len(Song2)):
      buzzer.duty_u16(2000)
      buzzer.freq(Song2[i])
      for j in range(1, Beat2[i] / 0.1):
        time.sleep_ms(25)
      buzzer.duty_u16(0)
      time.sleep(0.01)

def action_turn_red():
    global np
    for i in range(0, 4):
      np[i]=(255, 0, 0)
    np.write()

def action_turn_off():
    for i in range(0, 4):
      np[i] = (0, 0, 0)
    np.write()

def MsgOK(topic, msg_c):
    global mqtt_client
    msg_dict = {
        "forward": action_forward,
        "backward": action_backward,
        "left": action_left,
        "right": action_right,
        "greeting": action_greeting,
        "turnred": action_turn_red,
        "turnoff": action_turn_off,
    }
    log("Received message: {} on topic: {}".format(msg_c, topic), "INFO")
    if topic == myTopic.encode():
        msg = msg_c.decode()
        if msg in msg_dict:
            msg_dict[msg]()
        else:
            log("Unknown message: {}".format(msg), "ERROR")


if __name__ == "__main__":

    config_filename = "config.txt"
    config = {}
    with open(config_filename, "r") as f:
        # config
        # key=value
        for line in f.readlines():
            key, value = line.strip().split("=")
            config[key] = value

    wifi_wait_time = 0

    # MQTT setting
    myTopic = config['myTopic']
    clientID = config['clientID']
    machineId = config['machineId']

    run = False
    port = 1883

    

    mqtt_client = False

    serverIP = config['serverIp']
    wifiName = config['wifiName']
    wifiPassword = config['wifiPassword']

    wifi_connect = connect_wifi(wifiName, wifiPassword)

    server = MicroPyServer()

    server.add_route("/connect", connect_show_params)
    server.add_route("/stop", stop_show_params)
    server.add_route("/status", status_show_params)
    server.stop()
    server.start()

    motor1 = PWM(Pin(12))
    motor2 = PWM(Pin(13))
    motor3 = PWM(Pin(14))
    motor4 = PWM(Pin(15))
    buzzer = PWM(Pin(33))

    Tone = [0, 392, 440, 494, 523, 587, 659, 698, 784]
    Song2 = [Tone[3], Tone[3]]
    Beat2 = [1, 1]

    pin = Pin(25, Pin.OUT)
    np = neopixel.NeoPixel(pin, n=4, bpp=3, timing=1)
    
    motor(0,0,0,0)
    buzzer.duty_u16(0)
    while True:
        server.loop()
        if mqtt_client:
            mqtt_client.check_msg()

        utime.sleep(0.5)
    server.stop()
