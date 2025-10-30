# SLLIDAR 각도/거리 필터링 사용 가이드

## 📋 개요
원하는 **각도 범위**와 **거리 범위**의 LiDAR 데이터만 수집하고 발행할 수 있는 필터링 노드입니다.

## 🚀 사용 방법

### 1. 환경 설정
```bash
cd ~/lidar_ws
source install/setup.bash
```

### 2. 기본 실행 (0~360도, 전체 범위)
```bash
ros2 launch sllidar_ros2 sllidar_a1_filtered_launch.py
```

### 3. 특정 각도 범위 지정 실행

#### 예제 1: 정면만 (앞쪽 60도)
```bash
ros2 launch sllidar_ros2 sllidar_a1_filtered_launch.py angle_min_deg:=330.0 angle_max_deg:=30.0
```

#### 예제 2: 전방 180도
```bash
ros2 launch sllidar_ros2 sllidar_a1_filtered_launch.py angle_min_deg:=0.0 angle_max_deg:=180.0
```

#### 예제 3: 전방 + 가까운 거리만 (3m 이내)
```bash
ros2 launch sllidar_ros2 sllidar_a1_filtered_launch.py angle_min_deg:=0.0 angle_max_deg:=180.0 range_max:=3.0
```

#### 예제 4: 근거리 장애물 감지 (0.5~2m)
```bash
ros2 launch sllidar_ros2 view_sllidar_a1_filtered_launch.py angle_min_deg:=0.0 angle_max_deg:=360.0 range_min:=0.5 range_max:=2.0
```

#### 예제 5: 원거리만 감지 (5m 이상)
```bash
ros2 launch sllidar_ros2 view_sllidar_a1_filtered_launch.py range_min:=5.0 range_max:=100.0
```

### 4. 필터링 노드만 단독 실행
기존에 라이다가 실행 중일 때:
```bash
# 각도만 필터링
ros2 run sllidar_ros2 angle_filter_node.py --ros-args -p angle_min_deg:=45.0 -p angle_max_deg:=135.0

# 각도 + 거리 필터링
ros2 run sllidar_ros2 angle_filter_node.py --ros-args \
  -p angle_min_deg:=0.0 -p angle_max_deg:=180.0 \
  -p range_min:=0.5 -p range_max:=3.0
```

## 📊 토픽 정보

### 입력 토픽
- `/scan` : 원본 LiDAR 데이터 (0~360도 전체)

### 출력 토픽
- `/scan_filtered` : 필터링된 LiDAR 데이터 (지정한 각도 범위만)

### 토픽 확인
```bash
# 토픽 목록 확인
ros2 topic list

# 필터링된 데이터 확인
ros2 topic echo /scan_filtered

# 토픽 정보 확인
ros2 topic info /scan_filtered
```

## 🎯 각도 시스템

```
           0° (정면)
              |
              |
270° -------- + -------- 90°
   (좌측)     |     (우측)
              |
            180° (후면)
```

## 📝 실전 예제

### 장애물 회피 로봇 (전방 120도, 3m 이내)
```bash
ros2 launch sllidar_ros2 sllidar_a1_filtered_launch.py \
    angle_min_deg:=300.0 \
    angle_max_deg:=60.0 \
    range_max:=3.0
```

### 벽 따라가기 (우측 90도, 0.3~1.5m)
```bash
ros2 launch sllidar_ros2 sllidar_a1_filtered_launch.py \
    angle_min_deg:=270.0 \
    angle_max_deg:=360.0 \
    range_min:=0.3 \
    range_max:=1.5
```

### 주차 보조 (후방 180도, 5m 이내)
```bash
ros2 launch sllidar_ros2 sllidar_a1_filtered_launch.py \
    angle_min_deg:=90.0 \
    angle_max_deg:=270.0 \
    range_max:=5.0
```

### 사람 감지 (전방, 1~4m)
```bash
ros2 launch sllidar_ros2 view_sllidar_a1_filtered_launch.py \
    angle_min_deg:=315.0 \
    angle_max_deg:=45.0 \
    range_min:=1.0 \
    range_max:=4.0
```

## 🔧 다른 파라미터와 함께 사용

```bash
ros2 launch sllidar_ros2 view_sllidar_a1_filtered_launch.py \
    serial_port:=/dev/ttyUSB0 \
    serial_baudrate:=115200 \
    angle_min_deg:=45.0 \
    angle_max_deg:=135.0 \
    range_min:=0.5 \
    range_max:=5.0 \
    frame_id:=laser
```

## 📏 파라미터 설명

| 파라미터 | 단위 | 기본값 | 설명 |
|---------|------|--------|------|
| `angle_min_deg` | 도(°) | 0.0 | 최소 각도 |
| `angle_max_deg` | 도(°) | 360.0 | 최대 각도 |
| `range_min` | 미터(m) | 0.0 | 최소 감지 거리 |
| `range_max` | 미터(m) | 100.0 | 최대 감지 거리 |

## 📦 Python에서 필터링된 데이터 구독

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class FilteredDataSubscriber(Node):
    def __init__(self):
        super().__init__('filtered_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan_filtered',  # 필터링된 토픽 구독
            self.callback,
            10)
    
    def callback(self, msg):
        # 각도 범위 출력
        print(f"각도: {np.degrees(msg.angle_min):.1f}° ~ {np.degrees(msg.angle_max):.1f}°")
        print(f"거리: {msg.range_min:.2f}m ~ {msg.range_max:.2f}m")
        
        # 거리 데이터 (무한대 제외)
        ranges = np.array([r for r in msg.ranges if r != float('inf')])
        print(f"유효 포인트 수: {len(ranges)}")
        if len(ranges) > 0:
            print(f"최소 거리: {np.min(ranges):.2f}m")
            print(f"최대 거리: {np.max(ranges):.2f}m")

def main():
    rclpy.init()
    node = FilteredDataSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ⚠️ 주의사항

1. **각도 단위**: 모든 각도는 **도(degree) 단위**입니다
2. **거리 단위**: 모든 거리는 **미터(m) 단위**입니다
3. **각도 범위**: 0~360도 사이의 값을 사용하세요
4. **거리 범위**: 0.05m (라이다 최소값) ~ 센서 최대 거리
5. **토픽 이름**: 필터링된 데이터는 `/scan_filtered` 토픽으로 발행됩니다
6. **범위 밖 데이터**: 지정 범위 밖의 데이터는 `inf`(무한대)로 표시됩니다

## 🔍 문제 해결

### 데이터가 나오지 않는 경우
```bash
# 1. 라이다가 실행 중인지 확인
ros2 topic hz /scan

# 2. 필터링 노드가 실행 중인지 확인
ros2 node list | grep angle_filter

# 3. 각도 범위가 유효한지 확인
ros2 param get /angle_filter_node angle_min_deg
ros2 param get /angle_filter_node angle_max_deg
```

### 런타임에 각도 변경
```bash
ros2 param set /angle_filter_node angle_min_deg 45.0
ros2 param set /angle_filter_node angle_max_deg 135.0
```

## 🎓 추가 정보

- 필터링은 데이터 수신 후 소프트웨어 레벨에서 처리됩니다
- 라이다 하드웨어는 여전히 360도 전체를 스캔합니다
- 필터링된 데이터는 원본보다 가볍고 처리가 빠릅니다
- 여러 각도/거리 범위를 동시에 보고 싶다면 필터링 노드를 여러 개 실행하세요
- 거리 필터링은 장애물 감지, 안전 구역 설정 등에 유용합니다

