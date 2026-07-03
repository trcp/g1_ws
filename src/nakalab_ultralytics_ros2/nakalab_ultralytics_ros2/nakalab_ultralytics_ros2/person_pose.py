#!/usr/bin/env python3
from rclpy.node import Node
from rclpy import qos
import rclpy

from sensor_msgs.msg import Image
from nakalab_ultralytics_interfaces.msg import (
    BoundingBox,
    PersonPose2D,
    PersonPose2DArray,
    Point,
)
from nakalab_ultralytics_interfaces.srv import Detect

from cv_bridge import CvBridge
import cv2

from ament_index_python.packages import get_package_share_directory

import gc
import os


class NakalabUltralyticsRos2(Node):
    def __init__(self):
        # init node
        super().__init__('nakalab_ultralytics_ros2')

        # default value
        default_model_path = os.path.join(
            get_package_share_directory('nakalab_ultralytics_ros2'),
            'models',
            'yolo26l-pose.pt'
        )

        # declare parameter
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('model_path', default_model_path)
        self.declare_parameter('headless', True)
        self.declare_parameter('run_detect', False)

        # init
        self.__cvbridge = CvBridge()
        self.__device = self.get_parameter(
            'device'
        ).get_parameter_value().string_value
        self.__model_path = self.get_parameter(
            'model_path').get_parameter_value().string_value
        self.__model = None
        self.__confidence = 0.1
        self.__window_name = 'nakalab_ultralytics_ros2/person_pose'
        self.__display_warning_reported = False

        # sub
        self.__image_subscriber = self.create_subscription(
            Image,
            '/color_image',
            self.cb_image,
            qos.qos_profile_sensor_data
        )

        # pub
        self.detect_image_pub = self.create_publisher(
            Image,
            'nu_ros2/detect_image',
            qos.qos_profile_sensor_data
        )
        self.person_pose_pub = self.create_publisher(
            PersonPose2DArray,
            'nu_ros2/person_pose_2d',
            10
        )
        self.__detect_service = self.create_service(
            Detect,
            '/nu_ros2/detect_person',
            self.cb_detect_service
        )

        if self.get_parameter('run_detect').get_parameter_value().bool_value:
            self.__load_model(self.__confidence)
        else:
            self.get_logger().info(
                'Pose detection is stopped. Call /nu_ros2/detect_person to start.'
            )

    def cb_image(self, msg: Image):
        if self.__model is None:
            return

        try:
            cv_image = self.__cvbridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as err:
            self.get_logger().error(f'Failed to convert image: {err}')
            return

        try:
            results = self.__model.predict(
                source=cv_image,
                device=self.__device,
                conf=self.__confidence,
                verbose=False
            )
        except Exception as err:
            self.get_logger().error(f'Failed to run pose estimation: {err}')
            return

        if results:
            result = results[0]
            detect_image = result.plot()
        else:
            result = None
            detect_image = cv_image

        self.person_pose_pub.publish(self.__make_person_pose_msg(msg, result))

        detect_msg = self.__cvbridge.cv2_to_imgmsg(
            detect_image,
            encoding='bgr8'
        )
        detect_msg.header = msg.header
        self.detect_image_pub.publish(detect_msg)

        if not self.get_parameter(
                'headless').get_parameter_value().bool_value:
            self.__imshow(detect_image)

    def cb_detect_service(self, request, response):
        if request.run:
            success, message = self.__load_model(float(request.confidence))
        else:
            success, message = self.__unload_model()

        response.success = success
        response.message = message
        return response

    def __load_model(self, confidence):
        if not 0.0 <= confidence <= 1.0:
            return False, 'confidence must be between 0.0 and 1.0.'

        self.__confidence = confidence
        if self.__model is not None:
            message = (
                f'Pose model is already running '
                f'(confidence={self.__confidence:.2f}).'
            )
            self.get_logger().info(message)
            return True, message

        try:
            from ultralytics import YOLO
            self.__model = YOLO(self.__model_path)
        except Exception as err:
            self.__model = None
            message = f'Failed to load pose model: {err}'
            self.get_logger().error(message)
            return False, message

        message = (
            f'Loaded pose model: {self.__model_path} '
            f'(device={self.__device}, confidence={self.__confidence:.2f})'
        )
        self.get_logger().info(message)
        return True, message

    def __unload_model(self):
        if self.__model is None:
            message = 'Pose model is already stopped.'
            self.get_logger().info(message)
            return True, message

        self.__model = None
        gc.collect()
        try:
            import torch
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
        except Exception as err:
            self.get_logger().debug(f'Failed to clear CUDA cache: {err}')

        message = 'Unloaded pose model.'
        self.get_logger().info(message)
        return True, message

    def __make_person_pose_msg(self, image_msg: Image, result):
        pose_array = PersonPose2DArray()
        pose_array.header = image_msg.header

        if result is None or result.boxes is None or result.keypoints is None:
            return pose_array

        boxes_xyxy = result.boxes.xyxy
        box_conf = result.boxes.conf
        keypoints_xy = result.keypoints.xy
        keypoints_conf = result.keypoints.conf

        if boxes_xyxy is None or keypoints_xy is None:
            return pose_array

        boxes_xyxy = boxes_xyxy.detach().cpu().numpy()
        keypoints_xy = keypoints_xy.detach().cpu().numpy()
        if box_conf is not None:
            box_conf = box_conf.detach().cpu().numpy()
        if keypoints_conf is not None:
            keypoints_conf = keypoints_conf.detach().cpu().numpy()

        pose_count = min(len(boxes_xyxy), len(keypoints_xy))
        for pose_index in range(pose_count):
            pose = PersonPose2D()
            pose.bounding_box = self.__make_bounding_box(
                boxes_xyxy[pose_index],
                float(box_conf[pose_index]) if box_conf is not None else 0.0
            )
            pose.confidence = pose.bounding_box.confidence

            keypoint_count = min(
                len(keypoints_xy[pose_index]),
                len(pose.keypoints)
            )
            for keypoint_index in range(keypoint_count):
                keypoint = Point()
                keypoint.x = float(keypoints_xy[pose_index][keypoint_index][0])
                keypoint.y = float(keypoints_xy[pose_index][keypoint_index][1])
                if keypoints_conf is not None:
                    keypoint.confidence = float(
                        keypoints_conf[pose_index][keypoint_index]
                    )
                else:
                    keypoint.confidence = 1.0
                pose.keypoints[keypoint_index] = keypoint

            pose_array.poses.append(pose)

        return pose_array

    def __make_bounding_box(self, xyxy, confidence):
        bounding_box = BoundingBox()
        bounding_box.top_left.x = float(xyxy[0])
        bounding_box.top_left.y = float(xyxy[1])
        bounding_box.top_left.confidence = confidence
        bounding_box.bottom_right.x = float(xyxy[2])
        bounding_box.bottom_right.y = float(xyxy[3])
        bounding_box.bottom_right.confidence = confidence
        bounding_box.confidence = confidence
        return bounding_box

    def __imshow(self, image):
        if not os.environ.get('DISPLAY'):
            if not self.__display_warning_reported:
                self.get_logger().warning(
                    'DISPLAY is not set. Skipping cv2.imshow output.'
                )
                self.__display_warning_reported = True
            return

        cv2.imshow(self.__window_name, image)
        cv2.waitKey(1)

    def destroy_node(self):
        self.__unload_model()
        if os.environ.get('DISPLAY'):
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NakalabUltralyticsRos2()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
