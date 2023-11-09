import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from conceptio_interfaces.msg import ArenaHeartbeat
from conceptio_interfaces.msg import ArenaKinematics

from rclpy import qos
from rclpy.qos import QoSProfile
import paho.mqtt.client as mqtt
import json 
from rclpy.exceptions import InvalidTopicNameException
from functools import partial

class MqttMirror(Node):

    def __init__(self):
        super().__init__('mqtt_mirror')
        self.mqtt_client : mqtt.Client = mqtt.Client()
        self.mqtt_client.on_message = self.on_message
        self.declare_parameter('mqtt_host', 'emqx')
        self.declare_parameter('mqtt_port',  1883)
        self.get_logger().info("Connecting to MQTT broker with host: " + 
                              self.get_parameter('mqtt_host').get_parameter_value().string_value + 
                              " and port: " + str(self.get_parameter('mqtt_port').get_parameter_value().integer_value))
        self.mqtt_client.connect(self.get_parameter('mqtt_host').get_parameter_value().string_value, 
           self.get_parameter('mqtt_port').get_parameter_value().integer_value)
        self.mqtt_client.subscribe("conceptio/unit/#", qos = 0)
        self.create_timer(1.0, self.fetch_new_topics)
        

        self.publishers_ = {}
        self.mqtt_client.loop_forever()

    def fetch_new_topics(self):
        for subscription in self.subscriptions:
            subscription.destroy()

        topic_names_and_types = self.get_topic_names_and_types()
        for name, topic_type in topic_names_and_types:
            self.get_logger().info(f"[MQTT-Mirror] Found new topic {name}")
            self.create_subscription(topic_type, name, partial(self.republish_callback, topic_name = name ), 0)

    def republish_callback(self, msg, topic_name):
        # Republish ROS2 message in MQTT topic
        last_subtopic = topic_name.split('/')[-1]
        json_msg = None
        if last_subtopic == "kinematics":
            json_msg = json.dumps({
            "uuid": msg.uuid,
            "geo_point": {
                "latitude": msg.geo_point.latitude,
                "altitude": msg.geo_point.altitude,
                "longitude": msg.geo_point.longitude
            },
            "yaw": msg.yaw,
            "pitch": msg.pitch,
            "roll": msg.roll
        })
        elif last_subtopic == "heartbeat":
            json_msg = json.dumps({
            "entity_type": msg.entity_type,
            "entity_uuid": msg.entity_uuid,
            "entity_name": msg.entity_name,
            "heartbeat_rate": msg.heartbeat_rate,
            "timestamp": msg.stamp,
            "type": msg.type
        })
        else:
            self.get_logger().info(f"[MQTT-Mirror] Received message in unknown topic {topic_name}")
            return
        

        self.mqtt_client.publish(topic_name, json_msg, qos=0)

    def on_message(self, client, userdata, msg : mqtt.MQTTMessage):
        topic = msg.topic
        topic_no_whitespace_lowercase = topic.replace("-", "_").lower()
        topic_no_starting_with_number = ""
        topics_split = topic_no_whitespace_lowercase.split('/')
        for i, subtopic in enumerate(topics_split):
            if subtopic[0].isdigit():
                for i, char in enumerate(subtopic):
                    if not char.isdigit():
                        subtopic = subtopic[i:]
                        break 
            topic_no_starting_with_number += '/' + subtopic        

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
            send.uuid = message['uuid']
            send.geo_point.latitude = message['geo_point']['latitude']
            send.geo_point.altitude = message['geo_point']['altitude']
            send.geo_point.longitude = message['geo_point']['longitude']
            send.yaw = message['yaw']
            send.pitch = message['pitch']
            send.roll = message['roll']
        else:
            self.get_logger().info(f"[MQTT-Mirror] Received message in unknown topic {topic}")
            return
        
        if topic_no_starting_with_number not in self.publishers_:
                # https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html#
                _qos = QoSProfile(durability=qos.QoSDurabilityPolicy.VOLATILE,
                           reliability=qos.QoSReliabilityPolicy.BEST_EFFORT, history=qos.QoSHistoryPolicy.KEEP_LAST, depth=1)

                try:
                    self.publishers_[topic_no_starting_with_number] = self.create_publisher(type(send), topic_no_starting_with_number, 0)
                except InvalidTopicNameException as err_name:
                    self.get_logger().info(f"[MQTT-Mirror] {err_name}")
                    return
                
                self.get_logger().info(f"[MQTT-Mirror] Created publisher for {topic_no_starting_with_number}") 
        self.publishers_[topic_no_starting_with_number].publish(send)

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