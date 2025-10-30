#!/usr/bin/env python3
"""
OpenCV + Cubic Spline 경로 추정 차선 추종
- OpenCV로 차선 검출
- Cubic Spline으로 부드러운 경로 생성
- PID 제어
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.interpolate import CubicSpline

class OpenCVSplineLaneFollower(Node):
    def __init__(self):
        super().__init__('opencv_spline_lane_follower')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscriber & Publisher
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 이미지 크기
        self.img_width = 640
        self.img_height = 480
        
        # 차량 중심 위치 (이미지 하단 중앙)
        self.car_center_x = 320
        self.car_center_y = 450  # 이미지 하단 부근
        
        # ROI 설정
        self.roi_top_offset = 200
        self.roi_height = 200
        
        # HSV 노란색 차선 범위
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([30, 255, 255])
        
        # 목표 지점들 (경로 추정에 사용)
        self.target_points = []
        
        # PID 제어 파라미터 (조향 반응성 증가)
        self.kp = 0.012  # 증가
        self.kd = 0.005  # 증가
        self.error_old = 0.0
        
        # 속도 설정 (급커브 대응)
        self.normal_speed = 0.5  # 기본 속도 감소
        self.slow_speed = 0.25   # 급커브 속도 감소
        self.very_slow_speed = 0.15  # 매우 급한 커브용
        
        self.get_logger().info('OpenCV + Spline 차선 추종 시작!')
        
    def detect_lane_points(self, roi, offset_x, offset_y):
        """ROI에서 차선 포인트들 검출"""
        # HSV 변환
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # 노란색 필터링
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        
        # Canny edge detection
        blur = cv2.GaussianBlur(mask, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        
        # 여러 높이에서 차선 중심점 찾기
        points = []
        h, w = roi.shape[:2]
        
        # 5개 높이에서 샘플링
        for y in range(0, h, h // 5):
            if y >= h:
                continue
                
            # 해당 높이의 라인에서 흰색 픽셀 찾기
            row = edges[y, :]
            white_pixels = np.where(row > 0)[0]
            
            if len(white_pixels) > 0:
                # 중심점 계산
                center_x = int(np.mean(white_pixels))
                points.append((center_x + offset_x, y + offset_y))
        
        return points, mask
    
    def plan_path_with_spline(self, points):
        """Cubic Spline을 사용하여 부드러운 경로 생성"""
        if len(points) < 3:
            return None, None
        
        # 포인트를 y 기준으로 정렬
        sorted_points = sorted(points, key=lambda p: p[1])
        
        x_points = [p[0] for p in sorted_points]
        y_points = [p[1] for p in sorted_points]
        
        try:
            # Cubic Spline 보간
            cs = CubicSpline(y_points, x_points, bc_type='natural')
            
            # 부드러운 경로 생성 (100개 점)
            y_new = np.linspace(min(y_points), max(y_points), 100)
            x_new = cs(y_new)
            
            return x_new, y_new
            
        except Exception as e:
            self.get_logger().warn(f'Spline 생성 실패: {e}')
            return None, None
    
    def find_target_point(self, x_path, y_path, lane_curvature=0):
        """경로에서 차량 전방의 목표점 찾기"""
        if x_path is None or y_path is None:
            return self.car_center_x
        
        # 곡률에 따라 look-ahead distance 조정
        # 커브가 급하면 가까이, 직선이면 멀리 본다
        base_distance = 80
        if abs(lane_curvature) > 100:  # 급커브
            look_ahead_distance = 60  # 가까이 봄
        elif abs(lane_curvature) > 50:  # 중간 커브
            look_ahead_distance = 70
        else:  # 직선
            look_ahead_distance = base_distance
        
        target_y = self.car_center_y - look_ahead_distance
        
        # 목표 y 좌표에 가장 가까운 점 찾기
        idx = np.argmin(np.abs(y_path - target_y))
        target_x = x_path[idx]
        
        return target_x
    
    def image_callback(self, msg):
        try:
            # ROS Image -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            height, width = cv_image.shape[:2]
            self.img_width = width
            self.img_height = height
            
            # ROI 영역 설정
            roi_top = self.roi_top_offset
            roi_bottom = roi_top + self.roi_height
            
            # 왼쪽 ROI
            left_roi = cv_image[roi_top:roi_bottom, 0:width//2]
            # 오른쪽 ROI
            right_roi = cv_image[roi_top:roi_bottom, width//2:width]
            
            # 차선 포인트 검출
            left_points, left_mask = self.detect_lane_points(left_roi, 0, roi_top)
            right_points, right_mask = self.detect_lane_points(right_roi, width//2, roi_top)
            
            # 양쪽 차선 포인트 합치기
            all_points = left_points + right_points
            
            # 차선 검출 모드 저장
            lane_mode = "None"
            
            # 차선 중심 포인트 계산
            if len(left_points) > 0 and len(right_points) > 0:
                lane_mode = "Both Lanes"
                # 양쪽 차선의 중간 지점들 계산
                center_points = []
                for lp in left_points:
                    # 같은 높이의 오른쪽 포인트 찾기
                    for rp in right_points:
                        if abs(lp[1] - rp[1]) < 20:  # 비슷한 높이
                            center_x = (lp[0] + rp[0]) // 2
                            center_y = (lp[1] + rp[1]) // 2
                            center_points.append((center_x, center_y))
                            break
                
                # 차량 중심점 추가
                center_points.append((self.car_center_x, self.car_center_y))
                
                # Cubic Spline으로 경로 생성
                x_path, y_path = self.plan_path_with_spline(center_points)
                
                # 목표점 찾기
                if x_path is not None:
                    # 곡률 계산 (경로의 변화율)
                    curvature = np.std(np.diff(x_path[:20])) if len(x_path) > 20 else 0
                    target_x = self.find_target_point(x_path, y_path, curvature)
                else:
                    # Spline 실패 시 평균값 사용
                    target_x = int(np.mean([p[0] for p in center_points]))
                
            elif len(left_points) > 0:
                lane_mode = "Left Lane Only"
                # 왼쪽만 검출 - 차선을 따라가되 오른쪽으로 차선폭/2 이동
                # Spline 경로 생성
                x_path, y_path = self.plan_path_with_spline(left_points)
                
                if x_path is not None:
                    # 차선과 평행하게 일정 거리 유지
                    lane_offset = 360  # 차선폭의 절반 (픽셀) - 증가
                    # 차선 경로에서 오른쪽으로 offset만큼 이동한 경로 생성
                    offset_path_x = x_path + lane_offset
                    
                    # 차량 위치 추가하여 부드러운 경로 생성
                    combined_points = [(x, y) for x, y in zip(offset_path_x, y_path)]
                    combined_points.append((self.car_center_x, self.car_center_y))
                    
                    # 다시 Spline으로 부드럽게
                    x_path_final, y_path_final = self.plan_path_with_spline(combined_points)
                    
                    if x_path_final is not None:
                        curvature = np.std(np.diff(x_path[:20])) if len(x_path) > 20 else 0
                        target_x = self.find_target_point(x_path_final, y_path_final, curvature)
                    else:
                        target_x = int(np.mean(offset_path_x))
                else:
                    # Spline 실패 시 평균값 + offset
                    target_x = int(np.mean([p[0] for p in left_points])) + 240
                
            elif len(right_points) > 0:
                lane_mode = "Right Lane Only"
                # 오른쪽만 검출 - 차선을 따라가되 왼쪽으로 차선폭/2 이동
                # Spline 경로 생성
                x_path, y_path = self.plan_path_with_spline(right_points)
                
                if x_path is not None:
                    # 차선과 평행하게 일정 거리 유지
                    lane_offset = 360  # 차선폭의 절반 (픽셀) - 증가
                    # 차선 경로에서 왼쪽으로 offset만큼 이동한 경로 생성
                    offset_path_x = x_path - lane_offset
                    
                    # 차량 위치 추가하여 부드러운 경로 생성
                    combined_points = [(x, y) for x, y in zip(offset_path_x, y_path)]
                    combined_points.append((self.car_center_x, self.car_center_y))
                    
                    # 다시 Spline으로 부드럽게
                    x_path_final, y_path_final = self.plan_path_with_spline(combined_points)
                    
                    if x_path_final is not None:
                        curvature = np.std(np.diff(x_path[:20])) if len(x_path) > 20 else 0
                        target_x = self.find_target_point(x_path_final, y_path_final, curvature)
                    else:
                        target_x = int(np.mean(offset_path_x))
                else:
                    # Spline 실패 시 평균값 - offset
                    target_x = int(np.mean([p[0] for p in right_points])) - 240
                
            else:
                # 차선 미검출
                x_path = None
                y_path = None
                target_x = self.car_center_x
            
            # 제어 계산
            self.control_robot(target_x)
            
            # 디버그 이미지 표시
            display = cv_image.copy()
            
            # ROI 박스
            cv2.rectangle(display, (0, roi_top), (width//2, roi_bottom), (0, 255, 0), 2)
            cv2.rectangle(display, (width//2, roi_top), (width, roi_bottom), (0, 255, 0), 2)
            
            # 검출된 포인트들
            for point in left_points:
                cv2.circle(display, point, 3, (0, 255, 255), -1)
            for point in right_points:
                cv2.circle(display, point, 3, (0, 255, 255), -1)
            
            # Spline 경로 그리기
            if x_path is not None and y_path is not None:
                for i in range(len(x_path)-1):
                    pt1 = (int(x_path[i]), int(y_path[i]))
                    pt2 = (int(x_path[i+1]), int(y_path[i+1]))
                    cv2.line(display, pt1, pt2, (255, 0, 255), 3)
            
            # 목표점
            cv2.circle(display, (int(target_x), self.car_center_y - 100), 8, (0, 0, 255), -1)
            
            # 차량 중심
            cv2.circle(display, (self.car_center_x, self.car_center_y), 8, (255, 255, 0), -1)
            
            # 이미지 중앙선
            cv2.line(display, (width//2, 0), (width//2, height), (0, 0, 255), 2)
            
            # 상태 텍스트
            error = self.car_center_x - target_x
            status = f"Mode: {lane_mode} | Target: {int(target_x)} | Error: {int(error)}"
            cv2.putText(display, status, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.imshow('OpenCV + Spline Lane Detection', display)
            cv2.imshow('Left Mask', left_mask)
            cv2.imshow('Right Mask', right_mask)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'이미지 처리 오류: {e}')
    
    def control_robot(self, target_x):
        """로봇 제어 (급커브 대응 강화)"""
        cmd = Twist()
        
        # 오차 계산
        error = self.car_center_x - target_x
        
        # PID 제어 (비선형 게인 적용)
        derivative = error - self.error_old
        
        # 오차가 클수록 강한 조향 (비선형)
        if abs(error) > 100:
            # 매우 급한 커브 - 게인 증가
            kp_adaptive = self.kp * 1.8
            kd_adaptive = self.kd * 1.5
        elif abs(error) > 60:
            # 급한 커브
            kp_adaptive = self.kp * 1.4
            kd_adaptive = self.kd * 1.2
        else:
            # 일반 주행
            kp_adaptive = self.kp
            kd_adaptive = self.kd
        
        angular_z = kp_adaptive * error + kd_adaptive * derivative
        
        # 조향 제한 (35도까지 사용)
        max_angular = 1.2  # 더 큰 각속도 허용
        angular_z = np.clip(angular_z, -max_angular, max_angular)
        
        # 속도 조절 (3단계)
        if abs(error) > 100:
            # 매우 급한 커브 - 매우 느리게
            cmd.linear.x = self.very_slow_speed
            status = "Very Sharp Curve"
        elif abs(error) > 60:
            # 급한 커브 - 느리게
            cmd.linear.x = self.slow_speed
            status = "Sharp Curve"
        else:
            # 일반 주행 - 정상 속도
            cmd.linear.x = self.normal_speed
            status = "Normal"
        
        cmd.angular.z = angular_z
        
        self.cmd_vel_pub.publish(cmd)
        self.error_old = error
        
        # 로그
        if abs(error) > 10:
            self.get_logger().info(
                f'{status} | 오차: {error:4.0f}px | 조향: {angular_z:+.3f} | 속도: {cmd.linear.x:.2f}m/s'
            )

def main(args=None):
    rclpy.init(args=args)
    node = OpenCVSplineLaneFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자가 중지함')
    finally:
        # 정지
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

