import json
import paho.mqtt.client as mqtt

start_all_message = False
stop_all_message = False

def on_connect(client, userdata, flags, reason_code):

    print(f"Connected with result code {reason_code}")

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("vandalrobot")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global start_all_message
    global stop_all_message
    print(msg.topic+" "+str(msg.payload))
    payload= json.loads(msg.payload)
    if "requestMessage" in payload and  "brokerManager" in payload and payload['brokerManager']:
        if payload["requestMessage"] =='registration' :
            message = {
                "messageType": "registration",
                "node":"crx10", # e.g. "roomba", "crx10"
                "nodeId":"bryancrx10_1", # Your nodeID goes here
                "productLine":"moscow", # e.g. moscow, cda
                }
            mqttc.publish(topic, json.dumps(message))

    if "status" in payload and  "brokerManager" in payload and payload['brokerManager']:
        if payload["status"] =='start.all.node' :
            start_all_message=True
        if payload["status"] =='stop.all.node' :
            stop_all_message=True

mqttc = mqtt.Client()
mqttc.on_connect = on_connect
mqttc.on_message = on_message

mqttc.connect("mqtt.eclipseprojects.io", 1883, 60)

topic = "vandalrobot"

