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
        self.mqtt_client : mqtt.Client = mqtt.Client()
        self.mqtt_client.on_message = self.on_message
        self.declare_parameter('mqtt_host', 'emqx')
        self.declare_parameter('mqtt_port',  1883)
        self.get_logger().log("Connecting to MQTT broker with host: " + self.get_parameter('mqtt_host').get_parameter_value().string_value + " and port: " + str(self.get_parameter('mqtt_port').get_parameter_value().integer_value), 1)
        self.mqtt_client.connect(self.get_parameter('mqtt_host').get_parameter_value().string_value, 
           self.get_parameter('mqtt_port').get_parameter_value().integer_value)
        self.mqtt_client.subscribe("conceptio/unit/#", qos = 0)
        self.publishers_ = {}
        self.mqtt_client.loop_forever()

    def on_message(self, client, userdata, msg : mqtt.MQTTMessage):
        topic = msg.topic
        topic_no_whitespace_lowercase = topic.replace(" ", "-").lower()
        first_subtopic = topic.split('/')[1]
        last_subtopic = topic.split('/')[-1]
        if last_subtopic != "kinematics":
            return
        

        message = json.loads(msg.payload.decode("utf-8"))
        send = None

        if last_subtopic == "heartbeat":
            #print("Received message in heartbeat")
            send = ArenaHeartbeat()
            send.entity_type = message['entity_type']
            send.entity_uuid = message['entity_uuid']
            send.entity_name = message['entity_name']
            send.heartbeat_rate = message['heartbeat_rate']
            send.stamp = message['timestamp']
            send.type = message['type']
        elif last_subtopic == "kinematics":
            #print("Received message in kinematics")
            send = ArenaKinematics()
            send.geo_point.latitude = message['geo_point']['latitude']
            send.geo_point.altitude = message['geo_point']['altitude']
            send.geo_point.longitude = message['geo_point']['longitude']
            send.yaw = message['yaw']
            send.pitch = message['pitch']
            send.roll = message['roll']
        else:
            self.get_logger().info(f"[MQTT-Mirror] Received message in unknown topic {topic}")
            return
        
        if topic not in self.publishers_:
                # https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html#
                _qos = QoSProfile(durability=qos.QoSDurabilityPolicy.VOLATILE,
                           reliability=qos.QoSReliabilityPolicy.BEST_EFFORT, history=qos.QoSHistoryPolicy.KEEP_LAST, depth=1)

                self.publishers_[topic] = self.create_publisher(type(send), topic_no_whitespace_lowercase, 0)
                self.get_logger().info(f"[MQTT-Mirror] Created publisher for {topic}") 
        self.publishers_[topic].publish(send)

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"[MQTT-Mirror] Connected to MQTT broker with result code {rc}")

    def on_disconnect(self, client, userdata, rc):
        self.get_logger().info(f"[MQTT-Mirror] Disconnected from MQTT broker with result code {rc}")


def main(args=None):
    rclpy.init(args=args)
    mqtt_mirror = MqttMirror()


    rclpy.spin(mqtt_mirror)
    mqtt_mirror.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    