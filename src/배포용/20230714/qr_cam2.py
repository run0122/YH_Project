#!/usr/bin/python
# -*- coding : utf-8 -*-

import cv2
from pyzbar import pyzbar
import rospy
from std_msgs.msg import String

cap = cv2.VideoCapture(2)

allowed_formats = ['QRCODE', 'CODE128', 'EAN13']

rospy.init_node('qr_cam')

info_pub = rospy.Publisher('/qr_code_info', String, queue_size=10)

info_msg = None

while True:
    ret, frame = cap.read()

    barcodes = pyzbar.decode(frame)

    for barcode in barcodes:
        barcode_type = barcode.type

        if barcode_type in allowed_formats:
            qr_code_data = barcode.data.decode("utf-8")

            if qr_code_data == 'http://m.site.naver.com/1aNhl':
                info_msg = "Google"
            elif qr_code_data == 'http://naver.com':
                info_msg = "Naver"
            elif qr_code_data == 'https://www.daum.net/':
                info_msg = "Daum"
            else:
                info_msg = "Unknown"
            
        if info_msg == "Google":
            info_pub.publish(info_msg)
            print("QR Code Detected")
            info_msg = "Detected"
            break

    cv2.imshow("QR Code Scanner", frame)

    if info_msg == "Detected":
        print("QR Code Detection Terminated")
        break

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
