# SLAM
2025 SLAM 공부

단안 카메라를 사용한 Slam을 아래 데이터셋을 사용하여 구현.

https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download

## 구성
+ 이미지 데이터셋 불러오기
+ 3D시각화 함수
+ 2D시각화 함수
+ 노드(카메라 자세) 클래스
+ 엣지(자세 변화량) 클래스
+ 맵포인트 클래스
+ Pose graph Optimization 클래스
+ ORB특징점 추출 함수
+ 특징점 매칭 함수
+ 자세 추정 함수
+ 삼각 측량 함수
+ 다중 시점 삼각 측량 함수
+ 정보 행렬 계산 함수
+ 루프 감지 함수
+ 번들 조정 함수
+ (재투영 오차 시각화 및 특징점 매칭 시각화 함수)
+ 실행 함수

## 순서
1. 첫번째 사진에서 ORB를 이용해 특징점을 추출한다.
2. 두번째 사진에서도 특징점을 추출한다.
3. 두 사진에서 특징점 매칭을 진행한다.
4. 매칭된 특징점을 이용해 회전행렬과 평행이동 행렬, 불확실성을 구한다. 
5. 불확실성을 이용해 정보행렬을 계산한다.
6. 이전 자세에 회전행렬과 평행이동을 적용하여 현재 카메라의 자세를 추정한다.
7. 스케일팩터를 계산한다.
8. 추정된 자세를 노드 클래스에 저장하고, 정보행렬 및 변화량을 엣지 클래스에 저장한다.
9. 그 후 투영행렬과 특징점들을 이용하여 삼각측량을 진행하여 맵포인트를 구한다.
10. 맵포인트를 맵포인트 클래스에 저장한다.
11. 반복한다.
12. 다중시점 삼각측량을 실시한다.
13. 번들조정을 실시한다. 
14. 루프감지를 실시한다.
15. 루프를 감지했을 경우 해당 엣지를 추가한다.
16. 루프최적화를 실시한다.
17. 반복한다.

## 결과
<img width="608" height="521" alt="image" src="https://github.com/user-attachments/assets/0b8d6f1a-4d45-42ee-b28f-47ad24a96d5b" />

<img width="591" height="524" alt="image" src="https://github.com/user-attachments/assets/a6714f72-a066-4caf-867a-f83c4c5c3326" />

처음에는 Pose graph Optimization만 사용하여 그래프 최적화를 진행하였지만, 번들조정을 추가한 후로는 실제 움직임과 달리 값이 이상해지는 경향이 있어서 제외하고 진행했다.

번들조정의 경우는 삼각측량함수의 영향을 굉장히 많이 받는데, 특히 단안카메라의 경우 스케일정보를 정확히 알 수 없기 때문에 번들조정을 적용하는 과정에서 굉장히 많은 시행착오를 겪었다. 다중시점 삼각측량을 적용하고, 재투영오차가 그나마 안정되어 그 상태로 사용하게 되었다. 

번들조정을 적용한 후로 그래프최적화도 한번 더 적용했을 때는 꽤 실제와 비슷한 경로가 구현되었지만, 여전히 문제가 존재했다.

기본적인 경로의 경우는 자세추정함수의 영향을 굉장히 많이 받기 때문에, 자세추정 함수에 각도변화량 필터를 걸어줘서 최대한 비슷한 경로를 구현해내게 되었다. 

그 이후로 그래프최적화나 루프감지 후 최적화를 진행하게 되면서 오히려 결과가 이상해지는 현상이 나타나서 최적화의 경우는 번들조정만 적용하게 되었다. 

루프를 감지했을 경우 처리하는 로직을 좀 수정하면 더 괜찮은 결과가 나올 수 있을 것 같다. 

또한, 자세추정에 경우도 변화량이 너무 큰 경우는 반영을 안하기보다도 적게 반영하는 것도 하나의 방법이 될 수 있을 것 같다.

어느정도 구현을 하는데 성공은 했지만 monocular 카메라 하나만으로는 장애물의 스케일 정보를 정확히 알 수 없기 때문에 한계가 있었다. 

실제로 정확한 매핑을 하기 위해서는 최소 Odometry센서 추가로 마우스에 사용되는 광센서나 IMU센서와 융합을 하는 과정이 필요로 할 것같다. 


## 추가 연구
# ROS2 Jazzy + Gazebo Sim 기반 4WD 로봇 SLAM / Navigation 시뮬레이션

Windows 환경에서 **ROS2 Jazzy**, **Gazebo Sim (gz sim)**, **RobotCAD**, **Nav2**, **SLAM Toolbox**를 사용해 4륜 구동 로봇의 **SLAM**, **맵 저장**, **Localization**, **Navigation**까지 구성한 프로젝트입니다.

## 프로젝트 개요

이 프로젝트의 목표는 다음과 같습니다.

- 시뮬레이션에서 로봇을 구동한다.
- 목표 좌표를 주면 해당 좌표까지 이동하도록 만든다.
- 이후 실제 로봇 적용 시 고려사항까지 확장한다.

이번 단계에서는 다음 내용을 다룹니다.

- 4WD 로봇 모델 설계
- RobotCAD 기반 URDF / SDF 생성
- Gazebo Sim에서 로봇 구동
- Lidar / IMU / Odometry 브리징
- SLAM Toolbox를 이용한 지도 생성
- Nav2 기반 Localization / Navigation
- 좌표 지정 이동 실험
- Windows + WSL + Pixi + Gazebo 환경에서의 트러블슈팅

향후에는 아래 내용을 추가로 진행할 예정입니다.

- 실제 로봇 적용 시 고려사항 정리
- 장애물 회피 자동화
- SLAM 자동화
- Visual SLAM 적용
- Custom SLAM 적용

---

## 로봇 구조 선택

처음에는 **Ackermann 조향 방식**도 고려했습니다.  
Ackermann 구조는 자동차처럼 조향 각을 제어할 수 있지만, 좁은 공간에서 사용하는 모바일 로봇에는 제자리 회전이 어렵고 기동성이 떨어질 수 있습니다.

그래서 이번 프로젝트에서는 다음과 같은 방향으로 결정했습니다.

- 조향장치 없이
- 4 wheel robot 구조 사용
- diff drive 방식으로 제어

즉, **좁은 공간에서 움직이기 쉬운 4WD 이동 로봇**을 목표로 설계했습니다.

---

## 개발 환경

- **OS**: Windows
- **WSL2**: Ubuntu
- **시뮬레이터**: Gazebo Sim (`gz sim`)
- **ROS2**: Jazzy
- **패키지 매니저**: Pixi
- **CAD / 로봇 모델링**: RobotCAD
- **언어**: Python, C++, PowerShell, Bash

주요 구성 요소는 다음과 같습니다.

- **RobotCAD**: 로봇 모델 작성
- **URDF / SDF**: 로봇 및 월드 기술
- **Gazebo Sim**: 물리 시뮬레이션
- **ros_gz_bridge**: Gazebo ↔ ROS2 토픽 연결
- **slam_toolbox**: 지도 생성
- **Nav2**: localization / navigation
- **robot_localization (EKF)**: wheel odom + imu 융합

---

## 환경 구성

### 1. WSL / Docker / Ubuntu

처음에는 RobotCAD를 Docker + WSL2 환경에서 사용하려고 했습니다.

Ubuntu 컨테이너는 아래처럼 생성할 수 있습니다.

```bash
docker search ubuntu
docker pull ubuntu
docker run -it --name ubuntu ubuntu
docker start ubuntu
docker attach ubuntu
```

초기 설정 예시는 아래와 같습니다.

```bash
apt update
apt upgrade
apt install sudo -y
useradd [계정]
passwd
mkdir /home/[계정]
chown [계정]:[계정] /home/[계정]
sudo vim /etc/sudoers
```

하지만 실제로는 Windows + WSL2 + Docker Desktop 조합에서 권한 문제, 네트워크 문제, GPU 인식 문제 등이 발생했습니다.  
그래서 최종적으로는 **RobotCAD는 WSL 쪽에서 사용하고, ROS2 / Gazebo는 Windows에서 구동**하는 방향으로 정리했습니다.

---

### 2. WSL DNS 문제 해결

WSL Ubuntu에서 DNS 문제로 인터넷이 안 되는 경우가 있어 아래처럼 설정했습니다.

```bash
sudo rm /etc/resolv.conf
sudo bash -c 'echo "nameserver 8.8.8.8" > /etc/resolv.conf'
sudo chattr +i /etc/resolv.conf

sudo bash -c 'echo "[network]" > /etc/wsl.conf'
sudo bash -c 'echo "generateResolvConf = false" >> /etc/wsl.conf'
sudo bash -c 'echo "[boot]" > /etc/wsl.conf'
sudo bash -c 'echo "systemd = true" >> /etc/wsl.conf'
```

재시작은 아래처럼 진행했습니다.

```bash
wsl -t Ubuntu
wsl -d Ubuntu
```

---

### 3. Docker 엔진 직접 설치

Docker Desktop 연동 대신 WSL Ubuntu 내부에 Docker 엔진을 직접 설치했습니다.

```bash
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
```

```bash
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
```

```bash
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo systemctl start docker
sudo docker run hello-world
sudo usermod -aG docker $USER
newgrp docker
```

자동 시작 설정은 아래와 같이 했습니다.

```bash
sudo systemctl enable docker.service
sudo systemctl enable containerd.service
```

---

### 4. ROS2 Jazzy + Pixi 환경 구성

Windows에서 Gazebo와 ROS2를 같이 쓰기 위해 Pixi 기반 환경을 구성했습니다.

```bash
pixi init ros2_slam -c robostack-jazzy -c conda-forge
pixi add ros-jazzy-desktop-full
pixi shell
```

추가 패키지는 아래처럼 설치했습니다.

```bash
pixi add git colcon-common-extensions
pixi add ros-jazzy-nav2-bringup ros-jazzy-navigation2
```

PowerShell 자동화 예시는 아래와 같습니다.

```powershell
if (($(pwd) -like "*SLAM*") -and ($env:ROS_DISTRO -ne "jazzy")){
    $env:ROS_DISTRO = "jazzy"
    pixi shell
}
```

---

### 5. Gazebo 실행을 위한 Visual Studio 환경 설정

Windows에서 Gazebo 실행 시 Visual Studio 개발 환경이 필요한 경우가 있었습니다.

```powershell
pushd "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools"
cmd /c "VsDevCmd.bat&set" |
foreach {
  if ($_ -match "=") {
    $v = $_.split("=")
    set-item -force -path "ENV:\$($v[0])" -value "$($v[1])"
  }
}
popd
```

Gazebo 실행은 아래처럼 진행했습니다.

```bash
gz sim -s
gz sim -g
```

---

## RobotCAD 기반 로봇 모델링

로봇은 RobotCAD에서 다음과 같이 구성했습니다.

- Base
- 4개의 Wheel
- Lidar
- IMU
- base_footprint
- 각 Link와 Joint 연결

핵심은 다음과 같습니다.

- Robot은 **Link들을 Joint로 연결한 구조**
- 좌표계를 일치시켜야 TF와 구동이 정상 동작
- RobotCAD에서 모델링 후 **URDF 생성**
- URDF를 SDF로 변환한 뒤 Gazebo 플러그인을 수동 추가

변환 명령은 아래와 같습니다.

```bash
gz sdf -p Robot.urdf > Robot.sdf
```

xacro를 사용하는 경우는 아래처럼 진행했습니다.

```bash
ros2 run xacro xacro -o output.urdf .\Robot_wrapper.urdf.xacro
gz sdf -p .\output.urdf > Robot.sdf
```

---

## Gazebo 실행 구조

처음에는 URDF 기반 `robot_state_publisher` + spawn 방식을 사용했지만, 센서와 플러그인 제어를 위해 최종적으로는 **SDF를 Gazebo에 직접 spawn**하는 방식으로 정리했습니다.

예시 launch 구성은 아래와 같습니다.

```python
gazebo_spawn = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_ros_gz_sim, 'launch', 'gz_spawn_model.launch.py'),
    ),
    launch_arguments=dict(
        file=path_to_share_dir_clipped + '\\urdf\\Robot.sdf',
        entity_name="Robot",
        x="1.2",
        z="0.5",
        y="3.4",
        topic="Gulitter"
    ).items(),
)
```

시뮬레이션 자동 시작은 아래처럼 할 수 있습니다.

```bash
gz sim -r
```

또는 서비스 호출 방식으로 시작할 수도 있습니다.

```bash
gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'
```

---

## Gazebo ↔ ROS2 브리지

Gazebo 토픽은 기본적으로 `gz topic`에서만 보이기 때문에, ROS2에서 사용하려면 `ros_gz_bridge`가 필요합니다.

예시 `bridge_config.yaml`은 아래와 같습니다.

```yaml
- ros_topic_name: "/scan"
  gz_topic_name: "/lidar"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

- ros_topic_name: "/lidar/points"
  gz_topic_name: "/lidar/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS

- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/model/Robot/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "/wheel/odom"
  gz_topic_name: "/model/Robot/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

- ros_topic_name: "/tf"
  gz_topic_name: "/model/Robot/tf"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS

- ros_topic_name: "/joint_states"
  gz_topic_name: "/world/wall/model/Robot/joint_state"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS

- ros_topic_name: "/imu"
  gz_topic_name: "/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS
```

launch에서 `parameter_bridge` 노드로 실행하는 예시는 아래와 같습니다.

```python
gazebo_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[
        {
            'config_file': path_to_share_dir_clipped + "\\config\\bridge_config.yaml",
            'expand_gz_topic_names': True,
            'use_sim_time': True,
        }
    ],
    output='screen',
)
```

---

## 로봇 구동: DiffDrive + TF + Joint State

4WD 구조지만 제어는 DiffDrive 형태로 구성했습니다.

SLAM / Nav2 연동을 위해 `base_footprint` 링크를 별도로 추가했습니다.

### URDF 예시

```xml
<link name="base_footprint"/>

<joint name="base_joint" type="fixed">
  <parent link="base_footprint"/>
  <child link="link_frame"/>
  <origin xyz="0 0 0.0" rpy="0 0 0"/>
</joint>
```

### SDF 예시

```xml
<link name="base_footprint"/>

<joint name="base_joint" type="fixed">
  <parent>base_footprint</parent>
  <child>link_frame</child>
  <pose>0.0 0.0 0.0 0 0 0</pose>
</joint>
```

### DiffDrive 플러그인

```xml
<plugin
  filename="gz-sim-diff-drive-system"
  name="gz::sim::systems::DiffDrive">
  <left_joint>joint_frame_left_front_wheel</left_joint>
  <left_joint>joint_frame_left_back_wheel</left_joint>
  <right_joint>joint_frame_right_front_wheel</right_joint>
  <right_joint>joint_frame_right_back_wheel</right_joint>
  <wheel_separation>0.15</wheel_separation>
  <wheel_radius>0.0375</wheel_radius>
  <frame_id>odom</frame_id>
  <child_frame_id>base_footprint</child_frame_id>
  <odom_publish_frequency>30</odom_publish_frequency>
</plugin>
```

### Joint State Publisher 플러그인

```xml
<plugin
  filename="gz-sim-joint-state-publisher-system"
  name="gz::sim::systems::JointStatePublisher">
  <joint_name>joint_frame_left_front_wheel</joint_name>
  <joint_name>joint_frame_left_back_wheel</joint_name>
  <joint_name>joint_frame_right_front_wheel</joint_name>
  <joint_name>joint_frame_right_back_wheel</joint_name>
  <update_rate>30</update_rate>
</plugin>
```

---

## Lidar 추가

Lidar를 링크 / 조인트로 로봇에 부착하고 Gazebo 센서로 설정했습니다.

중요 포인트는 다음과 같습니다.

- 센서 링크 좌표계 설정
- `gz_frame_id` 지정
- RViz에서 LaserScan 표시
- 필요 시 PointCloud도 bridge

예시:

```xml
<gz_frame_id>link_lidar</gz_frame_id>
```

RViz 설정은 아래처럼 진행했습니다.

- TF 표시 활성화
- LaserScan 추가
- Style / Size 조정
- Fixed Frame 확인

---

## IMU 추가 및 EKF 융합

회전 시 슬립이 커서 Navigation 품질이 좋지 않아 IMU를 추가했습니다.

### SDF 예시

```xml
<plugin
  filename="gz-sim-imu-system"
  name="gz::sim::systems::Imu">
</plugin>

<link name="imu_link">
  <sensor name="imu" type="imu">
    <always_on>true</always_on>
    <update_rate>200</update_rate>
    <topic>/imu</topic>
    <gz_frame_id>imu_link</gz_frame_id>
    <imu>
      <angular_velocity>
        <x><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></x>
        <y><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></y>
        <z><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></z>
      </angular_velocity>
      <linear_acceleration>
        <x><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></x>
        <y><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></y>
        <z><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></z>
      </linear_acceleration>
    </imu>
  </sensor>
</link>

<joint name="imu_joint" type="fixed">
  <parent>link_frame</parent>
  <child>imu_link</child>
  <pose>0.0 0.0 0.068 0 0 0</pose>
</joint>
```

### URDF 예시

```xml
<joint name="imu_joint" type="fixed">
  <parent link="link_frame"/>
  <child link="imu_link"/>
  <origin xyz="0.0 0 0.068" rpy="0 0 0"/>
</joint>

<link name="imu_link"/>
```

### EKF 설정 예시

```yaml
ekf_filter_node:
  ros__parameters:
    imu0: /imu
    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, true,
                  true, true, true,
                  true, true, true]
    imu0_differential: false

    odom0: /wheel/odom
    odom0_config: [true, true, false,
                   false, false, true,
                   false, false, false,
                   false, false, false,
                   false, false, false]
```

launch 예시는 아래와 같습니다.

```python
ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[
        configured_params,
        {'use_sim_time': use_sim_time},
    ],
    remappings=[('odometry/filtered', '/odom')]
)
```

---

## SLAM 구성

SLAM은 `slam_toolbox`의 `online_async_launch.py`를 사용했습니다.

```bash
ros2 launch slam_toolbox online_async_launch.py
```

필요한 입력은 아래와 같습니다.

- `/scan`
- `/tf`
- `/odom`

결과로 아래 토픽을 사용할 수 있습니다.

- `/map`
- `/pose`

즉, Lidar와 Odometry, TF를 이용해 지도를 생성하고 로봇 위치를 추정할 수 있습니다.

맵 저장은 아래 명령으로 진행했습니다.

```bash
ros2 run nav2_map_server map_saver_cli -f C:\GuYeChan\Develop\ROS2\ros2_slam\src\car\car\saved_map\map
```

---

## Navigation 구성

Navigation은 크게 두 단계로 나뉩니다.

### 1. Localization

- `map_server`
- `amcl`
- `lifecycle_manager_localization`

### 2. Navigation

- `controller_server`
- `planner_server`
- `smoother_server`
- `behavior_server`
- `bt_navigator`
- `waypoint_follower`
- `velocity_smoother`
- `collision_monitor`
- `docking_server`
- `lifecycle_manager_navigation`

핵심 포인트는 다음과 같습니다.

- `map_server`는 저장된 `map.yaml`을 사용
- `amcl`로 현재 위치 추정
- Nav2는 composition 방식으로 실행
- `cmd_vel`, `odom`, `tf`, `scan` 토픽 정합이 매우 중요

---

## 목표 좌표로 이동

RViz의 `Publish Point` 패널 또는 직접 토픽 publish 방식으로 목표 좌표를 줄 수 있습니다.

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 0.0}}}"
```

RViz에서는 아래 도구를 추가해 사용했습니다.

- SetInitialPose
- SetGoal
- Map
- TF
- Global Costmap
- Local Costmap

---

## Python / C++ 제어 메모

### Python 제어

Gazebo transport를 직접 사용할 수도 있습니다.

- Node: `gz.transport[버전]`
- Msg: `gz.msgs[버전].[메시지]`

속도 제어는 `Twist` 메시지를 `cmd_vel`로 publish 하면 됩니다.

```python
node = gz_Node()
self.vel_pub = node.advertise(topic, Twist)
while not self.vel_pub.has_connections():
    time.sleep(0.2)
```

### C++ 노드 메모

- ROS2 노드는 포인터 기반으로 선언
- `spin()`은 executor를 돌리며 callback / timer를 관리
- 여러 노드가 있을 경우 `MultiThreadedExecutor` 사용 가능

---

## 시뮬레이션 튜닝

4WD 로봇은 회전 시 슬립이 심해질 수 있어 아래 요소들을 조정했습니다.

- 로봇 전체 질량 감소
- 바퀴 collision 마찰계수 조정
- PID 게인 조정
- slip 파라미터 조정
- 접촉 복원 파라미터 조정

예시는 아래와 같습니다.

```xml
<soft_cfm>0</soft_cfm>
<soft_erp>0.2</soft_erp>
<max_vel>0.01</max_vel>
<min_depth>0.001</min_depth>
```

주의할 점은 다음과 같습니다.

- 속도가 너무 빠르면 슬립이 심해져 localization / navigation 성능이 크게 떨어질 수 있음
- 4WD 구조는 일반적인 2륜 diff-drive보다 회전 특성 튜닝이 더 까다로움

---

## 주요 트러블슈팅

### 1. Gazebo topic이 ROS2에서 안 보이는 문제

Gazebo 토픽은 기본적으로 `gz topic -l`로 확인해야 합니다.  
ROS2에서 사용하려면 `ros_gz_bridge`가 반드시 필요합니다.

### 2. RViz에서 TF를 못 찾는 문제

Gazebo가 pause 상태면 TF가 제대로 안 들어올 수 있습니다.  
이 경우 Gazebo에서 재생 버튼을 눌러 해결할 수 있었습니다.

### 3. joint_states가 이상한 문제

SDF 안에 아래 플러그인이 중복되어 있으면 joint state가 비정상 동작하는 문제가 있었습니다.

```xml
<plugin
  filename="gz-sim-physics-system"
  name="gz::sim::systems::Physics">
</plugin>
```

### 4. base_frame 설정 문제

로봇 설계 시 축 기준이 맞지 않아 `local_costmap`, `global_costmap`의 `base_frame`을 `link_frame`이 아니라 `base_footprint`로 설정해야 정상 동작했습니다.

### 5. Windows 환경 불안정성

Windows + Gazebo + RViz 조합에서는 아래 문제가 자주 발생했습니다.

- RViz가 안 켜짐
- Gazebo GUI 비정상
- Initial Pose 설정 실패
- 실행할 때마다 결과가 다름

재실행을 반복하면 해결되는 경우가 많았습니다.

---

## 프로젝트 구조 예시

```text
ros2_slam/
├─ src/
│  └─ car/
│     ├─ urdf/
│     │  ├─ Robot.urdf
│     │  ├─ Robot.sdf
│     │  └─ Robot_wrapper.urdf.xacro
│     ├─ meshes/
│     ├─ launch/
│     │  ├─ gazebo.launch.py
│     │  ├─ slam.launch.py
│     │  └─ navigation.launch.py
│     ├─ config/
│     │  └─ bridge_config.yaml
│     ├─ params/
│     │  └─ nav2_params.yaml
│     ├─ saved_maps/
│     │  └─ map.yaml
│     ├─ src/
│     │  ├─ slam.py
│     │  └─ visualize_lidar.cpp
│     └─ CMakeLists.txt
├─ pixi.toml
└─ README.md
```

---

## 실행 순서 예시

### 1. Gazebo 실행

```bash
gz sim -r
```

### 2. 로봇 spawn + bridge + RViz 실행

```bash
ros2 launch car gazebo.launch.py
```

### 3. SLAM 실행

```bash
ros2 launch slam_toolbox online_async_launch.py
```

### 4. teleop으로 조작

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/model/Robot/cmd_vel
```

### 5. 맵 저장

```bash
ros2 run nav2_map_server map_saver_cli -f C:\GuYeChan\Develop\ROS2\ros2_slam\src\car\car\saved_map\map
```

### 6. Localization + Navigation 실행

저장한 맵을 기반으로 localization / navigation launch를 실행합니다.

---

## 느낀 점

4 wheel drive 로봇에 대한 예제가 많지 않았고, 특히 **Gazebo Sim + Windows + ROS2 Jazzy** 조합은 자료가 매우 적어서 개발 난이도가 높았습니다.

가장 어려웠던 부분은 아래와 같습니다.

- Windows 환경에서 Gazebo 관련 자료 부족
- RobotCAD → URDF → SDF → Gazebo 플러그인 연결 과정
- TF / odom / base frame 정합
- 4WD 회전 슬립 보정
- 브리지 토픽 이름 및 프레임 설정
- RViz / Gazebo의 불안정성

그럼에도 최종적으로는 다음 단계까지 구성할 수 있었습니다.

- 로봇 모델 생성
- Gazebo 시뮬레이션 구동
- Lidar / IMU / Odom 연동
- SLAM
- 지도 저장
- Localization
- Goal 기반 Navigation

---

## 앞으로 할 일

- 실제 로봇 하드웨어 적용
- 장애물 회피 고도화
- SLAM 자동화
- Visual SLAM 적용
- Custom SLAM 적용
- 실제 센서 기반 튜닝
- 성능 및 안정성 개선

---

## 회고

처음에는 단순히 좌표를 주면 이동하는 것까지 금방 될 줄 알았지만, 실제로는 환경 구성, 프레임 정합, 센서 브리징, 물리 튜닝, Nav2 파라미터 조정 등 훨씬 많은 부분을 직접 맞춰야 했습니다.

특히 Windows 환경과 4WD 구조 때문에 시행착오가 많았지만, 그 과정을 하나씩 해결하면서 최종적으로 전체 파이프라인을 끝까지 구성할 수 있었습니다.

다음에는 이 프로젝트를 기반으로 실제 로봇 적용 단계까지 확장해볼 예정입니다.
