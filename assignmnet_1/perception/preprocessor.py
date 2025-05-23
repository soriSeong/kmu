# preprocessor.py
import cv2
import numpy as np
import rospy # 로깅을 위해 필요

class PreProcessor:
    def __init__(self, roi_height=480, roi_width=640):
        # --- `self.roi_height`와 `self.roi_width`를 먼저 초기화 ---
        self.roi_height = roi_height # ROI 높이 (BEV 이미지의 높이)
        self.roi_width = roi_width   # ROI 폭 (BEV 이미지의 폭)
        # -------------------------------------------------------------

        # --- Bird Eye View (BEV) 변환을 위한 원근 변환 점들 (⭐ 중요: 카메라에 맞춰 조정 필수 ⭐) ---
        self.src_points = np.float32([
            (0, 479),   # 원본 이미지의 좌측 하단 (예시)
            (640, 479), # 원본 이미지의 우측 하단 (예시)
            (400, 280), # 원본 이미지의 우측 상단 (차선 소실점 근처, 예시)
            (240, 280)  # 원본 이미지의 좌측 상단 (차선 소실점 근처, 예시)
        ])
        # 변환될 BEV 이미지 상의 목적지 점들 (직사각형 형태로 펼쳐짐)
        # 이제 self.roi_height와 self.roi_width가 먼저 정의되어 있습니다.
        self.dst_points = np.float32([
            (100, self.roi_height),  # BEV 이미지의 좌측 하단 (예시)
            (self.roi_width - 100, self.roi_height),  # BEV 이미지의 우측 하단 (예시)
            (self.roi_width - 100, 0),    # BEV 이미지의 우측 상단 (예시)
            (100, 0)     # BEV 이미지의 좌측 상단 (예시)
        ])
        self.M = cv2.getPerspectiveTransform(self.src_points, self.dst_points) # 원본 -> BEV 변환 행렬
        self.Minv = cv2.getPerspectiveTransform(self.dst_points, self.src_points) # BEV -> 원본 역변환 행렬 (오버레이용)

        # 차선 추적 상태 변수 (이전 프레임 정보 유지를 위함)
        self.last_left_fit = None # 이전 프레임의 왼쪽 차선 다항식 계수
        self.last_right_fit = None # 이전 프레임의 오른쪽 차선 다항식 계수
        self.current_line_status = "LOST" # 현재 차선 상태: "LEFT", "RIGHT", "CENTER_BOTH", "LOST"

        # 슬라이딩 윈도우 파라미터 (이 값들을 조정하여 차선 검출 성능 최적화)
        self.window_height = 20 # 각 슬라이딩 윈도우의 높이
        self.nwindows = 24      # 사용할 슬라이딩 윈도우의 개수
        self.margin = 40        # 윈도우의 좌우 폭 절반 (차선 픽셀을 찾을 검색 범위)
        self.minpix = 50        # 윈도우 내에서 차선 픽셀로 간주할 최소 개수

        # 디버그용 `imshow` 윈도우 생성 (중복 방지 및 초기화)
        cv2.namedWindow("BEV Filtered Image", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Initial Regions", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Sliding Windows Debug", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Final Overlay (BEV)", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Final Overlay (Original View)", cv2.WINDOW_NORMAL)


    def warp_perspect(self, img):
        # 이미지를 흑백으로 변환 (BEV 변환은 주로 흑백 이미지에 적용)
        if len(img.shape) == 3: # 컬러 이미지인 경우
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else: # 이미 흑백인 경우
            img_gray = img

        # BEV 변환 적용: 원근 변환 행렬 `self.M`을 사용하여 이미지를 위에서 본 것처럼 변환
        warped = cv2.warpPerspective(img_gray, self.M, (self.roi_width, self.roi_height), flags=cv2.INTER_LINEAR)
        return warped # 흑백 BEV 이미지 반환

    def color_filter(self, img):
        # HSV 색공간 변환 (BGR에서 HSV)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # 이 함수는 BGR 이미지를 가정

        # 흰색 픽셀 범위를 설정 (이 범위는 환경에 따라 조정 필요)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 25, 255])

        # HSV 범위 내의 흰색만 마스크 생성
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # 마스크를 이용하여 원본 이미지에서 흰색 부분만 추출
        white_parts_color = cv2.bitwise_and(img, img, mask=mask)

        # 추출된 흰색 부분을 흑백 이미지로 변환 (Sliding Window에 사용하기 위함)
        white_parts_gray = cv2.cvtColor(white_parts_color, cv2.COLOR_BGR2GRAY)

        # 디버그용 이미지 표시
        cv2.imshow("HSV Filtered Image", white_parts_gray)
        
        # 추가: 이진화 (Sliding Window가 `0` 또는 `255` 값만 가진 이미지에서 가장 잘 작동)
        ret, binary_img = cv2.threshold(white_parts_gray, 50, 255, cv2.THRESH_BINARY) # 임계값 50 (조정 필요)
        
        cv2.imshow("BEV Filtered Image", binary_img)
        cv2.waitKey(1)
        return binary_img # 이진화된 흑백 이미지 반환


    def measure_curvature(self, fit_cr, ym_per_pix, xm_per_pix):
        '''
        BEV 공간에서 차선 곡률을 계산합니다.
        fit_cr: BEV 공간에서 픽셀 단위로 피팅된 다항식 계수 (Ax^2 + Bx + C, x = Ay^2 + By + C 형태)
        ym_per_pix: BEV에서 Y축 픽셀당 미터 (m/pixel)
        xm_per_pix: BEV에서 X축 픽셀당 미터 (m/pixel)
        '''
        # 곡률을 계산할 Y좌표 (일반적으로 차량과 가장 가까운 지점, 즉 이미지 최하단)
        ploty = np.linspace(0, self.roi_height - 1, self.roi_height)
        y_eval = np.max(ploty) # 이미지 최하단 Y좌표

        # 픽셀 단위를 미터 단위로 변환하여 다항식 계수 재계산
        # x = Ay^2 + By + C 에서 A, B, C를 미터 단위로 변환
        A = fit_cr[0] / (xm_per_pix / (ym_per_pix**2))
        B = fit_cr[1] / (xm_per_pix / ym_per_pix)
        
        # 곡률 공식: ((1 + (2Ay + B)^2)^(3/2)) / |2A|
        # y_eval은 픽셀 단위이므로 미터 단위로 변환된 Y값 `y_eval * ym_per_pix` 사용
        curvature = ((1 + (2*A*y_eval*ym_per_pix + B)**2)**1.5) / np.abs(2*A)
        return curvature


    def find_lane_lines(self, binary_warped_img):
        height, width = binary_warped_img.shape[:2]
        out_img_debug = cv2.cvtColor(binary_warped_img, cv2.COLOR_GRAY2BGR) 

        nonzero = binary_warped_img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        # --- 1. 히스토그램 기반 초기 차선 시작점 찾기 ---
        # 이미지 하단 (높이의 2/3 지점부터)의 X축 픽셀 분포(히스토그램)를 분석하여 차선 시작점 예측
        histogram_bottom_slice = binary_warped_img[height * 2 // 3:, :] 
        histogram = np.sum(histogram_bottom_slice, axis=0)

        midpoint = np.int32(width // 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # 초기 시작점 시각화 (디버그용)
        debug_initial = out_img_debug.copy()
        cv2.line(debug_initial, (leftx_base, height), (leftx_base, height - 50), (0, 255, 255), 3) 
        cv2.line(debug_initial, (rightx_base, height), (rightx_base, height - 50), (255, 255, 0), 3) 
        cv2.putText(debug_initial, "Initial Search", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        cv2.imshow("Initial Regions", debug_initial)
        cv2.waitKey(1)
        
        all_left_lane_pixels_x = []
        all_left_lane_pixels_y = []
        all_right_lane_pixels_x = []
        all_right_lane_pixels_y = []

        # 초기 차선 존재 여부 판단 (히스토그램 피크의 강도 기준)
        left_line_found_init = histogram[leftx_base] > self.minpix * 5 
        right_line_found_init = histogram[rightx_base] > self.minpix * 5

        current_leftx = leftx_base
        current_rightx = rightx_base

        if left_line_found_init or right_line_found_init: # 적어도 하나의 차선이 초기 감지되면 슬라이딩 윈도우 시작
            for window in range(self.nwindows): # `nwindows` 개수만큼 윈도우를 위로 이동하며 탐색
                # 윈도우의 Y 좌표 범위 (이미지 하단에서 위로 이동)
                win_y_low = height - (window + 1) * self.window_height
                win_y_high = height - window * self.window_height

                # 윈도우의 X 좌표 범위 (현재 차선 위치를 중심으로 `margin` 만큼)
                win_left_x_low = current_leftx - self.margin
                win_left_x_high = current_leftx + self.margin
                win_right_x_low = current_rightx - self.margin
                win_right_x_high = current_rightx + self.margin

                # 디버그 이미지에 현재 윈도우 시각화 (왼쪽: 녹색, 오른쪽: 파란색)
                cv2.rectangle(out_img_debug, (win_left_x_low, win_y_low), (win_left_x_high, win_y_high), (0, 255, 0), 2)
                cv2.rectangle(out_img_debug, (win_right_x_low, win_y_low), (win_right_x_high, win_y_high), (0, 0, 255), 2)

                # 현재 윈도우 내의 차선 픽셀 인덱스 찾기
                good_left_inds_in_window = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                                            (nonzerox >= win_left_x_low) & (nonzerox < win_left_x_high)).nonzero()[0]
                good_right_inds_in_window = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                                             (nonzerox >= win_right_x_low) & (nonzerox < win_right_x_high)).nonzero()[0]

                # 다음 윈도우의 시작점 업데이트 (현재 윈도우에서 충분한 픽셀이 감지되면 평균 X값 사용)
                if len(good_left_inds_in_window) > self.minpix:
                    current_leftx = np.int32(np.mean(nonzerox[good_left_inds_in_window]))
                elif self.last_left_fit is not None: # 픽셀이 부족하면 이전 프레임의 다항식으로 예측
                    current_leftx = np.int32(self.last_left_fit[0] * ((win_y_low + win_y_high) / 2)**2 +
                                             self.last_left_fit[1] * ((win_y_low + win_y_high) / 2) + self.last_left_fit[2])

                if len(good_right_inds_in_window) > self.minpix:
                    current_rightx = np.int32(np.mean(nonzerox[good_right_inds_in_window]))
                elif self.last_right_fit is not None: # 픽셀이 부족하면 이전 프레임의 다항식으로 예측
                    current_rightx = np.int32(self.last_right_fit[0] * ((win_y_low + win_y_high) / 2)**2 +
                                             self.last_right_fit[1] * ((win_y_low + win_y_high) / 2) + self.last_right_fit[2])

                # 감지된 픽셀들을 전체 리스트에 추가 (나중에 다항식 피팅에 사용)
                all_left_lane_pixels_x.extend(nonzerox[good_left_inds_in_window])
                all_left_lane_pixels_y.extend(nonzeroy[good_left_inds_in_window])
                all_right_lane_pixels_x.extend(nonzerox[good_right_inds_in_window])
                all_right_lane_pixels_y.extend(nonzeroy[good_right_inds_in_window])

            # 슬라이딩 윈도우 디버그 이미지 표시
            cv2.putText(out_img_debug, "Sliding Windows", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.imshow("Sliding Windows Debug", out_img_debug)
            cv2.waitKey(1)

        # 2. 다항식 피팅 (Polynomial Fitting)
        left_fit = None
        right_fit = None
        ploty = np.linspace(0, height - 1, height) # BEV 이미지의 Y축 전체 범위

        if len(all_left_lane_pixels_x) > self.minpix * 5: 
            left_fit = np.polyfit(all_left_lane_pixels_y, all_left_lane_pixels_x, 2) 
            self.last_left_fit = left_fit 
        elif self.last_left_fit is not None: 
            left_fit = self.last_left_fit
        
        if len(all_right_lane_pixels_x) > self.minpix * 5:
            right_fit = np.polyfit(all_right_lane_pixels_y, all_right_lane_pixels_x, 2)
            self.last_right_fit = right_fit
        elif self.last_right_fit is not None:
            right_fit = self.last_right_fit

        # 3. PID 제어를 위한 목표 값 (`target_x`), 차선 상태 (`current_line_status`), 그리고 **곡률(`lane_curvature`)** 계산
        target_x = width // 2 
        lane_curvature = 0.0 # 기본 곡률 값 (직선)
        
        overlay_lx, overlay_ly, overlay_rx, overlay_ry = [], [], [], []

        # BEV 이미지 픽셀당 미터 값 (이 값들은 실제 측정 또는 카메라 캘리브레이션으로 구해야 함)
        # ⭐⭐ 중요: 이 값들을 실제 차량 및 BEV 설정에 맞게 보정해야 합니다. ⭐⭐
        # 예시: 720픽셀에 30미터라고 가정 (Y축) / 700픽셀에 3.7미터라고 가정 (X축, 도로 폭)
        ym_per_pix = 30/720 
        xm_per_pix = 3.7/700 

        if left_fit is not None and right_fit is not None:
            # 양쪽 라인이 모두 감지되었을 때: 두 라인의 중앙점을 목표로
            left_x_at_bottom = left_fit[0]*(height-1)**2 + left_fit[1]*(height-1) + left_fit[2]
            right_x_at_bottom = right_fit[0]*(height-1)**2 + right_fit[1]*(height-1) + right_fit[2]
            target_x = np.int32((left_x_at_bottom + right_x_at_bottom) / 2)
            self.current_line_status = "CENTER_BOTH"

            # 오버레이용 라인 좌표 계산
            overlay_lx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
            overlay_ly = ploty
            overlay_rx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
            overlay_ry = ploty

            # 두 라인의 평균 곡률 계산
            left_curvature = self.measure_curvature(left_fit, ym_per_pix, xm_per_pix)
            right_curvature = self.measure_curvature(right_fit, ym_per_pix, xm_per_pix)
            lane_curvature = (left_curvature + right_curvature) / 2 

        elif left_fit is not None:
            # 왼쪽 라인만 감지되었을 때
            left_x_at_bottom = left_fit[0]*(height-1)**2 + left_fit[1]*(height-1) + left_fit[2]
            # `width * 0.135`는 대략 차선 폭의 절반을 의미합니다. 이 값은 조정이 필요할 수 있습니다.
            target_x = np.int32(left_x_at_bottom + width * 0.135) 
            self.current_line_status = "LEFT"

            overlay_lx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
            overlay_ly = ploty
            
            lane_curvature = self.measure_curvature(left_fit, ym_per_pix, xm_per_pix)

        elif right_fit is not None:
            # 오른쪽 라인만 감지되었을 때
            right_x_at_bottom = right_fit[0]*(height-1)**2 + right_fit[1]*(height-1) + right_fit[2]
            target_x = np.int32(right_x_at_bottom - width * 0.135)
            self.current_line_status = "RIGHT"

            overlay_rx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
            overlay_ry = ploty

            lane_curvature = self.measure_curvature(right_fit, ym_per_pix, xm_per_pix)

        else:
            # 라인을 찾지 못했거나 이전 정보도 없을 경우: 기본적으로 이미지 중앙을 목표로 설정
            self.current_line_status = "LOST"
            rospy.logwarn("[PreProcessor] BEV: 차선을 잃었거나 찾지 못함. 목표 X좌표를 중앙으로 설정.")
            lane_curvature = 0.0 

        # 최종 BEV 디버그 이미지 생성 (차선 및 목표점 표시)
        final_bev_debug_img = out_img_debug.copy()
        if len(overlay_lx) > 0:
            pts = np.array([np.transpose(np.vstack([overlay_lx, overlay_ly]))])
            cv2.polylines(final_bev_debug_img, np.int32([pts]), False, (0, 255, 0), 5) 
        if len(overlay_rx) > 0:
            pts = np.array([np.transpose(np.vstack([overlay_rx, overlay_ry]))])
            cv2.polylines(final_bev_debug_img, np.int32([pts]), False, (0, 0, 255), 5) 
        
        cv2.circle(final_bev_debug_img, (target_x, height - 10), 10, (255, 255, 255), -1) 
        cv2.putText(final_bev_debug_img, f"Target X: {target_x}", (10, height - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(final_bev_debug_img, f"Status: {self.current_line_status}", (10, height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(final_bev_debug_img, f"Curvature: {lane_curvature:.2f}", (10, height - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.imshow("Final Overlay (BEV)", final_bev_debug_img)
        cv2.waitKey(1)

        # PID 제어를 위한 `target_x`, 오버레이 시각화를 위한 라인 좌표, 그리고 **곡률** 반환
        return target_x, overlay_lx, overlay_ly, overlay_rx, overlay_ry, final_bev_debug_img, lane_curvature


    def overlay_line(self, original_img, lx, ly, rx, ry, bev_debug_img):
        height, width = original_img.shape[:2]
        
        lane_img_bev = np.zeros((self.roi_height, self.roi_width, 3), dtype=np.uint8)
        
        if len(lx) > 0 and len(ly) > 0:
            left_line_pts = np.array([np.transpose(np.vstack([lx, ly]))])
            cv2.polylines(lane_img_bev, np.int32([left_line_pts]), False, (0, 255, 0), 10) 
        if len(rx) > 0 and len(ry) > 0:
            right_line_pts = np.array([np.transpose(np.vstack([rx, ry]))])
            cv2.polylines(lane_img_bev, np.int32([right_line_pts]), False, (0, 0, 255), 10) 

        unwarped_lines = cv2.warpPerspective(lane_img_bev, self.Minv, (width, height), flags=cv2.INTER_LINEAR)
        
        result = cv2.addWeighted(original_img, 1, unwarped_lines, 0.5, 0) 
        
        cv2.imshow("Final Overlay (Original View)", result)
        cv2.waitKey(1)
        
        return result