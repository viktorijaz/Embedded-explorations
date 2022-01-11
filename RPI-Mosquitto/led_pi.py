import paho.mqtt.client as mqtt
from gpiozero import LED
from time import sleep

led = LED(17)

def on_connect(client, userdata, flags, rc):
	print("Connected to local mqtt server")
	client.subscribe("/pi/led")

def on_message(client, userdata, msg):
	print(msg.topic)
	print(msg.payload.decode("utf-8"))
	command = msg.payload.decode("utf-8");
	
	if command == "on":
		led.on()
	if command == "off":
		led.off()

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("localhost", 1883, 60)
client.loop_forever()