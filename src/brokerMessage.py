from brokerSender import mqttc
# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    payload= json.loads(msg.payload)
    if "requestMessage" in payload and  "brokerManager" in payload and payload['brokerManager']:
        if payload["requestMessage"] =='registration' :
            message = {
                "messageType": "registration",
                "node":"roomba", # e.g. "roomba", "crx10"
                "nodeId":"gatesroomba12", # Your nodeID goes here
                "productLine":"moscow", # e.g. moscow, cda
                }
            mqttc.publish(topic, json.dumps(message))
    print()
    if "requestMessage" in payload and  "brokerManager" in payload and payload['brokerManager']:
        if payload["status"] =='start.all.node' :
            start_all_message=True