import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from rclpy import qos
from rclpy.qos import QoSProfile
import paho.mqtt.client as mqtt

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
        send = String()
        topic = msg.topic
        last_subtopic = topic.split('/')[-2]

        message = msg.payload.decode("utf-8")
        send.data = message

        if last_subtopic == "position":
            pass
        else:
            if topic not in self.publishers_:
                _qos = QoSProfile(durability=qos.QoSDurabilityPolicy.VOLATILE,
                           reliability=qos.QoSReliabilityPolicy.BEST_EFFORT, history=qos.QoSHistoryPolicy.KEEP_LAST, depth=1)

                self.publishers_[topic] = self.create_publisher(String, topic, 0)
            self.publishers_[topic].publish(send)
            

        
def main(args=None):
    rclpy.init(args=args)
    mqtt_mirror = MqttMirror()


    rclpy.spin(mqtt_mirror)
    mqtt_mirror.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    