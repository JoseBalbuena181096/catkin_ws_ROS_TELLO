#!/usr/bin/env python3
import sys
import rospy
import cv2
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError 
import mediapipe as mp

class Image_converter:
    def __init__(self):
        self.bridge = CvBridge()    
        self.image_sub = rospy.Subscriber("/tello/image_raw",
                                          Image,
                                          self.callback,
                                          queue_size=1,
                                          buff_size=2**24)
        self.pubCenterFace = rospy.Publisher('/face_detection/center',
                                            Int32MultiArray,
                                            queue_size= 10)
        self.mp_face_detection = mp.solutions.face_detection
        self.mp_drawing = mp.solutions.drawing_utils
        self.center_detections = Int32MultiArray()


    def resize_image(self, image, scale_percent):
        width = int(image.shape[1]*(scale_percent/100))
        height = int(image.shape[0]*(scale_percent/100))
        dim = (width, height)
        resize = cv2.resize(image, dim, interpolation= cv2.INTER_AREA)
        return resize

    def face_detection(self, image):
        with self.mp_face_detection.FaceDetection(
            min_detection_confidence = 0.8) as face_detection:
            
            results = face_detection.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            (rows, cols, _) = image.shape

            if results.detections is not None:
                for detection in results.detections:
                    xmin = int(detection.location_data.relative_bounding_box.xmin * cols)
                    ymin = int(detection.location_data.relative_bounding_box.ymin * rows)
                    w = int(detection.location_data.relative_bounding_box.width * cols)
                    h = int(detection.location_data.relative_bounding_box.height * rows)
                    cv2.rectangle(image, (xmin, ymin), (xmin+w, ymin+h), (255, 0, 0), 8)
                    ymid = ymin + int(h/2)
                    xmid = xmin + int(w/2)
                    cv2.line(image, (xmid, 0), (xmid, rows), color=(0, 255, 0), thickness=3)
                    cv2.line(image, (0, ymid), (cols, ymid), color=(0, 255, 0), thickness=3)
                    cv2.circle(image, (xmid, ymid), radius=3, color=(0, 0, 255), thickness=3)
                    data = f'Center X: {xmid}, Y: {ymid}'
                    cv2.putText(image, data, (20, 20), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 0, 255))
                    rospy.loginfo(data)
        
        return image, xmid, ymid

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")                         
        except CvBridgeError as e:
            print(e)
        (rows,cols,channels) = cv_image.shape
        #cv_image = self.resize_image(cv_image, 100)
        print(f'rows: {rows}, cols: {cols}, channels: {channels}')
        (cv_image, xmid, ymid) = self.face_detection(cv_image)
        self.center_detections.data = [int(xmid), int(ymid)]
        self.pubCenterFace.publish(self.center_detections)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

def main(args):
    ic = Image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)