import paho.mqtt.client as mqtt
from motion_execution import execute_motion 
#### MQTT callback functions ####

# Called when client attempts to connect to broker 
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc)) # Connection succeful for result code = 0
    client.subscribe("test")

# Called when a message is published over the subscribed topic
def on_message(client, userdata, msg):
    global moveUR10

    if msg.payload.decode() != "":
        print("message received")

        # Decode the bytes receive in the mqtt message payload
        mqtt_msg = msg.payload.decode()
        # extract the center point coordinates from the message payload 
        cor_list = [float(x) for x in mqtt_msg.split()]

        # Execute the soldering motions
        moveit_interface = userdata[0]
        tf_buffer = userdata[1]
        execute_motion(cor_list, moveit_interface,tf_buffer)


