import rclpy
import sys
from rclpy.node import Node
from autoware_auto_perception_msgs.msg import DetectedObjects
from tier4_perception_msgs.msg import DetectedObjectsWithFeature
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.timer import Timer

class DetectedObjectsRelay(Node):

    def __init__(self, sub_topic):
        super().__init__('detected_objects_republisher')

        # Define QoS profile to match the publisher
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)

        # Create publishers for republishing messages
        self.det_objects_pub = self.create_publisher(
            DetectedObjects, 'detected_objects', qos_profile)

        # Create subscribers for both topics
        self.det_obj_wfeat_sub = self.create_subscription(
            DetectedObjectsWithFeature,
            sub_topic,
            self.detected_objects_wfeat_callback,
            qos_profile)

    def detected_objects_wfeat_callback(self, msg: DetectedObjectsWithFeature):
        det_objs = DetectedObjects()
        det_objs.header = msg.header
        for obj in msg.feature_objects:
          det_objs.objects.append(obj.object)
        self.det_objects_pub.publish(det_objs) # ignore features

def main(args=None):
    rclpy.init(args=args)
    node = DetectedObjectsRelay(sys.argv[1])

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

   
