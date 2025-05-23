# lane_detector.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import rospy, time
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String, Float32 
import signal
import sys
import os
from preprocessor import PreProcessor 

def signal_handler(sig, frame):
    time.sleep(1)
    cv2.destroyAllWindows() # 실행 중인 모든 OpenCV 창 닫기
    os.system('killall -9 python rosout') # ROS 관련 프로세스 강제 종료
    sys.exit(0) # 프로그램 종료

signal.signal(signal.SIGINT, signal_handler) # Ctrl-c 시그널 핸들러 등록


image = np.empty(shape=[0]) # 카메라 이미지를 저장할 전역 변수

# 차선 인식 결과 퍼블리셔 선언
pub_target_x = None
pub_lane_status = None
pub_lane_curvature = None 


CAM_FPS = 30    # 카메라 FPS
WIDTH, HEIGHT = 640, 480 # 카메라 이미지 가로x세로 크기

def img_callback(data):
    global image
    image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
    
    if data.encoding == 'rgb8': 
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)


def start():
    pre_module = PreProcessor(roi_height=HEIGHT, roi_width=WIDTH)

    global image 
    global pub_target_x, pub_lane_status, pub_lane_curvature 

    rospy.init_node('lane_detector')
    
    pub_target_x = rospy.Publisher('/lane_detection/target_x', Int32, queue_size=1)
    pub_lane_status = rospy.Publisher('/lane_detection/lane_status', String, queue_size=1)
    pub_lane_curvature = rospy.Publisher('/lane_detection/lane_curvature', Float32, queue_size=1) 
    
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)

    print ("----- lane detector node started -----")

    rospy.wait_for_message("/usb_cam/image_raw", Image)

    while not rospy.is_shutdown():
        img = image.copy()

        # BEV 변환
        warped_img = pre_module.warp_perspect(img) 

        # 색상 필터링 (warped_img는 흑백이므로, color_filter에 전달 전에 BGR로 변환)
        warped_img_bgr = cv2.cvtColor(warped_img, cv2.COLOR_GRAY2BGR)
        white_parts_binary = pre_module.color_filter(warped_img_bgr)

        # 슬라이딩 윈도우 기반 차선 검출 및 목표점, 곡률 계산
        target_x, overlay_lx, overlay_ly, overlay_rx, overlay_ry, bev_debug_img, lane_curvature = \
            pre_module.find_lane_lines(white_parts_binary)
        
        # 검출된 차선 정보를 원본 이미지 위에 오버레이하여 시각적으로 확인
        final_overlay_original_view = pre_module.overlay_line(img.copy(), overlay_lx, overlay_ly, overlay_rx, overlay_ry, bev_debug_img)
        
        #=========================================
        # 차선 인식 결과 토픽 발행
        #=========================================
        target_x_msg = Int32()
        target_x_msg.data = target_x
        pub_target_x.publish(target_x_msg)

        lane_status_msg = String()
        lane_status_msg.data = pre_module.current_line_status
        pub_lane_status.publish(lane_status_msg)

        lane_curvature_msg = Float32() 
        lane_curvature_msg.data = float(lane_curvature) 
        pub_lane_curvature.publish(lane_curvature_msg)
        
        print(f"Published: Target X: {target_x}, Status: {pre_module.current_line_status}, Curvature: {lane_curvature:.2f}")


if __name__ == '__main__':
    start()