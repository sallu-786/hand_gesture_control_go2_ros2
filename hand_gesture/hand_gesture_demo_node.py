

import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp

class HandGestureNode(Node):
    def __init__(self):
        super().__init__('hand_gesture_node')

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils

        # OpenCV webcam capture
        self.cap = cv2.VideoCapture(0)  # 0 is default webcam

        self.timer = self.create_timer(0.1, self.process_frame)  # 10 Hz

    def process_frame(self):
        ret, image = self.cap.read()
        if not ret:
            self.get_logger().warn("Cannot get webcam frame")
            return

        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        result = self.hands.process(image_rgb)

        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks[0]:
                self.mp_drawing.draw_landmarks(image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
                thumb_ip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_IP]
                thumb_state = 'down' if thumb_tip.y > thumb_ip.y else 'up'

                self.get_logger().info(f"Thumb is {thumb_state}")

        cv2.imshow("hand_gesture_test", image)
        if cv2.waitKey(20) == 27:  # ESC key to exit
            self.get_logger().info("ESC pressed. Shutting down...")
            rclpy.shutdown()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HandGestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.hands.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

