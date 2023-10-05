import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from conceptio_interfaces.msg import ArenaHeartbeat
from conceptio_interfaces.msg import ArenaKinematics

from rclpy import qos
from rclpy.qos import QoSProfile
import paho.mqtt.client as mqtt
import json 

class MqttMirror(Node):

    def __init__(self):
        super().__init__('mqtt_mirror')
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_message = self.on_message
        self.declare_parameter('mqtt_host', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.mqtt_client.connect(self.get_parameter('mqtt_host').get_parameter_value().string_value, 
           self.get_parameter('mqtt_port').get_parameter_value().integer_value)
        self.mqtt_client.subscribe("conceptio/unit/+/+/+/#", qos = 0)
        self.publishers_ = {}
        self.mqtt_client.loop_forever()

    def on_message(self, client, userdata, msg : mqtt.MQTTMessage):
        topic = msg.topic
        last_subtopic = topic.split('/')[-2]

        message = json.loads(msg.payload.decode("utf-8"))
        send = None

        if last_subtopic == "heartbeat":
            print("Received message in lastsubtopic heartbeat")
            send = ArenaHeartbeat()
            send.entity_type = message['entity_type']
            send.entity_uuid = message['entity_uuid']
            send.entity_name = message['entity_name']
            send.heartbeat_rate = message['heartbeat_rate']
            send.stamp = message['stamp']
            send.type = message['type']
        elif last_subtopic == "kinematics":
            print("Received message in lastsubtopic kinematics")
            send = ArenaKinematics()
            send.geo_point = message['geopoint']
            send.yaw = message['yaw']
            send.pitch = message['pitch']
            send.roll = message['roll']
        else:
            print(f"Incorrect lastsubtopic? {last_subtopic}")
            return
        
        if topic not in self.publishers_:
                # https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html#
                _qos = QoSProfile(durability=qos.QoSDurabilityPolicy.VOLATILE,
                           reliability=qos.QoSReliabilityPolicy.BEST_EFFORT, history=qos.QoSHistoryPolicy.KEEP_LAST, depth=1)

                self.publishers_[topic] = self.create_publisher(type(send), topic, 0)
        self.publishers_[topic].publish(send)
        
def main(args=None):
    rclpy.init(args=args)
    mqtt_mirror = MqttMirror()


    rclpy.spin(mqtt_mirror)
    mqtt_mirror.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    