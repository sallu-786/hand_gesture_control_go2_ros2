import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.go2.video.video_client import VideoClient
import mediapipe as mp

class HandGestureNode(Node):
    def __init__(self):
        super().__init__('hand_gesture_node')

        ChannelFactoryInitialize(0)
        self.robot_state = unitree_go_msg_dds__SportModeState_()
        self.test = SportClient()
        self.test.SetTimeout(10.0)
        self.test.Init()

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils

        # Init DDS and subscriber
        self.sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self.sub.Init(self.HighStateHandler, 10)
        time.sleep(1)

        self.client = VideoClient()
        self.client.SetTimeout(3.0)
        self.client.Init()

        self.timer = self.create_timer(0.1, self.process_frame)

    def HighStateHandler(self, msg: SportModeState_):
        self.robot_state = msg

    def process_frame(self):
        code, data = self.client.GetImageSample()
        if code != 0:
            self.get_logger().warn("Error getting image sample: %d" % code)
            return

        image_data = np.frombuffer(bytes(data), dtype=np.uint8)
        image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        result = self.hands.process(image_rgb)

        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
                thumb_ip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_IP]
                thumb_state = 'down' if thumb_tip.y > thumb_ip.y else 'up'

                if thumb_state == 'up':
                    self.test.StandUp()
                    self.get_logger().info("Thumb up: Standing up")
                elif thumb_state == 'down':
                    self.test.StandDown()
                    self.get_logger().info("Thumb down: Standing down")

        cv2.imshow("front_camera", image)
        if cv2.waitKey(20) == 27:
            self.get_logger().info("ESC pressed. Shutting down...")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = HandGestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.hands.close()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

