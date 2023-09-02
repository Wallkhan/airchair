import math
import numpy as np
import json 
import rclpy
import time
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import TransformStamped, Transform

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped



class ArucoTarget(Node):
    _DICTS = {
        "4x4_100" : cv2.aruco.DICT_4X4_100,
        "4x4_1000" : cv2.aruco.DICT_4X4_1000,
        "4x4_250" : cv2.aruco.DICT_4X4_250,
        "4x4_50" : cv2.aruco.DICT_4X4_50,
        "5x5_100" : cv2.aruco.DICT_5X5_100,
        "5x5_1000" : cv2.aruco.DICT_5X5_1000,
        "5x5_250" : cv2.aruco.DICT_5X5_250,
        "5x5_50" : cv2.aruco.DICT_5X5_50,
        "6x6_100" : cv2.aruco.DICT_6X6_100,
        "6x6_1000" : cv2.aruco.DICT_6X6_1000,
        "6x6_250" : cv2.aruco.DICT_6X6_250,
        "6x6_50" : cv2.aruco.DICT_6X6_50,
        "7x7_100" : cv2.aruco.DICT_7X7_100,
        "7x7_1000" : cv2.aruco.DICT_7X7_1000,
        "7x7_250": cv2.aruco.DICT_7X7_250,
        "7x7_50": cv2.aruco.DICT_7X7_50,
        "apriltag_16h5" : cv2.aruco.DICT_APRILTAG_16H5,
        "apriltag_25h9" : cv2.aruco.DICT_APRILTAG_25H9,
        "apriltag_36h10" : cv2.aruco.DICT_APRILTAG_36H10,
        "apriltag_36h11" : cv2.aruco.DICT_APRILTAG_36H11,
        "aruco_original" : cv2.aruco.DICT_ARUCO_ORIGINAL
    }
# values from chair defintion
    _TARGET_WIDTH = 0.45
    _TARGET_LENGTH = 0.01
    _TARGET_HEIGHT = 0.60
    _TARGET_XOFFSET = -0.45
    _TARGET_ZOFFSET = -0.3


    def __init__(self, tag_set="6x6_1000", target_width=0.20):
        super().__init__('aruco_target')
        self.get_logger().info(f'{self.get_name()} created')

        self.declare_parameter('chair_name', "chair_a")
        self.declare_parameter('image', "camera/image_raw")
        self.declare_parameter('info', "camera/camera_info")
        self.declare_parameter('target_offset', "target_offset")
        self.declare_parameter('board_definition', "{}")

        self._chair_name = self.get_parameter('chair_name').get_parameter_value().string_value
        self._image_topic = self.get_parameter('image').get_parameter_value().string_value
        self._info_topic = self.get_parameter('info').get_parameter_value().string_value
        self._target_topic = self.get_parameter('target_offset').get_parameter_value().string_value
        self._board_definition = json.loads(self.get_parameter('board_definition').get_parameter_value().string_value)
        self.get_logger().info(f'{self.get_name()} has board definition {self._board_definition}')

        self.create_subscription(Image, f"/{self._chair_name}/{self._image_topic}", self._image_callback, 1)
        self.create_subscription(CameraInfo, f"/{self._chair_name}/{self._info_topic}", self._info_callback, 1)
        self._target_pub = self.create_publisher(PointStamped, f"/{self._chair_name}/{self._target_topic}", 1)

        self._bridge = CvBridge()

        dict = ArucoTarget._DICTS.get(tag_set.lower(), None)
        self.get_logger().info(f'{self.get_name()} has dict {dict}')
        if dict is None:
            self.get_logger().error(f'ARUCO tag set {tag_set} not found')
        else:
            self._aruco_dict = cv2.aruco.getPredefinedDictionary(dict)
            self._aruco_param = cv2.aruco.DetectorParameters()
            self._aruco_detector = cv2.aruco.ArucoDetector(self._aruco_dict, self._aruco_param)
            self._markerLength = .13   # Here, our measurement unit is meters.
            self._markerSeparation = 0.01   # Here, our measurement unit is meters.
            
            ids = self._board_definition['ids']
            self.get_logger().error(f'ids is {ids}')
#            self._ids = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8])
            self._ids = np.array(json.loads(ids))
            self.get_logger().error(f'afterm9dsm9sm{self._ids}')
            self._board = cv2.aruco.GridBoard((3, 3), self._markerLength, self._markerSeparation, cv2.aruco.getPredefinedDictionary(dict), self._ids)
            self._target_width = target_width
            self._image = None
            self._cameraMatrix = None
            self._ids = self._board.getIds()
            self.get_logger().info(f"using dictionary {tag_set}, with board {self._ids}")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _info_callback(self, msg):
        if msg.distortion_model != "plumb_bob":
            self.get_logger().error(f"We can only deal with plumb_bob distortion {msg.distortion_model}")
        self._distortion = np.reshape(msg.d, (1,5))
        self._cameraMatrix = np.reshape(msg.k, (3,3))

    def _image_callback(self, msg):
        self._image = self._bridge.imgmsg_to_cv2(msg, "bgr8") 

        grey = cv2.cvtColor(self._image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = self._aruco_detector.detectMarkers(grey)
        frame = cv2.aruco.drawDetectedMarkers(self._image, corners, ids)
        if ids is None:
#            self.get_logger().info(f"No targets found!")
            return
        if self._cameraMatrix is None:
            self.get_logger().info(f"We have not yet received a camera_info message")
            return
        obj_points, img_points = cv2.aruco.Board.matchImagePoints(self._board, detectedCorners=corners, detectedIds=ids)

        retval, rvec, tvec = cv2.solvePnP(obj_points, img_points, self._cameraMatrix, self._distortion)

        x = ArucoTarget._TARGET_WIDTH/2
        y = ArucoTarget._TARGET_HEIGHT
        z = 0  # keep in target plane
        axis = np.float32([[x-0.05, y-0.05, z-0.05], [x-0.05, y+0.05, z-0.05], [x+0.05, y+0.05, z-0.05], [x+0.05, y-0.05, z-0.05],
            [x-0.05, y-0.05, z+0.05], [x-0.05, y+0.05, z+0.05], [x+0.05, y+0.05, z+0.05],[x+0.05, y-0.05, z+0.05]])
        imgpts, jac = cv2.projectPoints(axis, rvec, tvec, self._cameraMatrix, self._distortion)
        imgpts = np.int32(imgpts).reshape(-1, 2)

        result = self._image.copy()
		
        # Draw the bottom side (over the marker)
        cv2.drawContours(result, [imgpts[:4]], -1, (255, 0, 0), 2)
        cv2.drawContours(result, [imgpts[4:]], -1, (0, 255, 0), 2)
        cv2.drawContours(result, [np.array( [imgpts[0], imgpts[1], imgpts[5], imgpts[4]])], -1, (0, 0, 255), 2)
        cv2.drawContours(result, [np.array( [imgpts[2], imgpts[3], imgpts[7], imgpts[6]])], -1, (255, 0, 255), 2)
        cv2.drawContours(result, [np.array( [imgpts[1], imgpts[2], imgpts[6], imgpts[5]])], -1, (255, 255, 0), 2)
        cv2.drawContours(result, [np.array( [imgpts[0], imgpts[3], imgpts[7], imgpts[4]])], -1, (0, 255, 255), 2)

        if retval != 0:
            result = cv2.drawFrameAxes(result, self._cameraMatrix, self._distortion, rvec, tvec, self._target_width)
        cv2.imshow('window', result)
        cv2.waitKey(3)
		
        r, _ = cv2.Rodrigues(rvec)


        px = r[0][0] * x + r[0][1] * y + r[0][2] * z + tvec[0]
        py = r[1][0] * x + r[1][1] * y + r[1][2] * z + tvec[1]
        pz = r[2][0] * x + r[2][1] * y + r[2][2] * z + tvec[2]
   
        tx = pz
        ty = -px
        tz = -py
        px = tx
        py = ty
        pz = tz

        try:
            if False:
# I think that there is a bug in the Buffer() code in Python when running in a namespace
# so the hack below gets around it. Horribly
                self.get_logger().info(f" Trying {self._chair_name}/base_link {self._chair_name}/camera_link")
                t = self._tf_buffer.lookup_transform(f"{self._chair_name}/base_link",  "f{self._chair_name}/camera_link", rclpy.time.Time())
                self.get_logger().info(f"Transform lookup worked")
            else:
# these values were obtained by listening to the static transformation outside of a namespace
                t = TransformStamped()
                t.transform.translation.x = 0.662
                t.transform.translation.y = 0.0
                t.transform.translation.z = -0.977
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = -0.125
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 0.992
            pointStamped = PointStamped()
            pointStamped.header.stamp = self.get_clock().now().to_msg()
            pointStamped.header.frame_id = "f{self._chair_name}/camera_link"
            pointStamped.point.x = float(px)
            pointStamped.point.y = float(py)
            pointStamped.point.z = float(pz)
            point_wrt_target = tf2_geometry_msgs.do_transform_point(pointStamped, t)
            point_wrt_target.header.stamp = self.get_clock().now().to_msg()
            self._target_pub.publish(point_wrt_target)
        except TransformException as ex:
            self.get_logger().info(f"Unable to find transformation {ex}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

