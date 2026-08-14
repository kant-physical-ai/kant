# ros2 cli

- ros2 topic list
- 등등..

# rviz2

- RViz2는 ROS2의 대표 3D 시각화 도구입니다. 추상적인 숫자 토픽을 공간 속 형상으로 보여 줍니다.

```shell
rviz2
```

- ros2 launch urdf_tutorial display.launch.py model:=/home/pa/workspaces/pa/source/kimhyunha/daily/20260812/robot.urdf

# turtlebot3 rviz2

```shell
ros2 launch turtlebot3_bringup rviz2.launch.py
# Gazebo 실행
# ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

# turtlebot3 gazebo

- https://docs.robotis.com/docs/systems/turtlebot3/simulation/gazebo_simulation

```shell
cd ~/turtlebot3_sw
colcon build
source install/setup.bash

# other bot type
export TURTLEBOT3_MODEL=burger
# export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo empty_world.launch.py

# other world
# ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

# turtlebot3 keyboard

- https://docs.robotis.com/docs/systems/turtlebot3/simulation/gazebo_simulation

```shell
ros2 run turtlebot3_teleop teleop_keyboard
```

# turtlebot3 SLAM

- https://docs.robotis.com/docs/systems/turtlebot3/simulation/slam_simulation

```shell
export TURTLEBOT3_MODEL=burger
`ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
````

```shell
ros2 run nav2_map_server map_saver_cli -f ~/map
```

# turtlebot3_navigation

- https://docs.robotis.com/docs/systems/turtlebot3/simulation/navigation_simulation/

```shell
ros2 launch turtlebot3_navigation turtlebot3_navigation.launch.py
$ export TURTLEBOT3_MODEL=burger
`$ ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
````

# 기록과 재생

```shell
ros2 bag record /scan /image /tf      # 지정 토픽 기록 (-a는 전체)
ros2 bag record -a -o field_test_01   # 전체를 이름 붙여 기록
ros2 bag play field_test_01           # 재생
ros2 bag info field_test_01           # 내용 요약(토픽·기간·메시지 수)

#play loop option 
ros2 bag play --loop field_test_01


# ros2 bag record /scan /image /tf녹화 중인 터미널에서:
#Space : 일시정지(Pause)
#Space 다시 : 녹화 재개(Resume)
#Ctrl+C : 녹화 종료 및 bag 저장 완료
```

## play proxy

### 예시 1: 단일 토픽 이름 바꾸기

- 만약 저장된 백 파일 안의 /scan 토픽을 /scan_test라는 토픽으로 바꿔서 쏘고 싶다면:

```bash
ros2 bag play field_test_01 --ros-args -r /scan:=/scan_test
예시 2: 여러 토픽을 한 번에 바꾸기
/scan은 /scan_test로, /cmd_vel은 /cmd_vel_backup으로 동시에 바꾸고 싶다면 -r 옵션을 여러 번 이어 붙이면 돼.
```

```bash
ros2 bag play field_test_01 --ros-args -r /scan:=/scan_test -r /cmd_vel:=/cmd_vel_backup
```

### 2. 유용한 응용 옵션들 (추천)

- 토픽 이름을 바꾸는 것 외에도, 실무에서 ros2 bag play를 쓸 때 유용하게 조합하는 옵션들이 있어.

### 원하는 토픽만 골라서 재생하기 (--topics)

백 파일 안에 수십 개의 토픽이 들어있는데, 내가 변경하고 싶은 토픽 한두 개만 재생하고 싶을 때 사용해.

### 여러 토픽

```shell
ros2 bag play field_test_01 --ros-args \
  -r /scan:=/scan_test \
  -r /cmd_vel:=/cmd_vel_test \
  -r /odom:=/odom_test
```

```bash
ros2 bag play field_test_01 --topics /scan --ros-args -r /scan:=/scan_test
```

### 네임스페이스 (Namespace) 한 번에 통째로 붙이기 (-r __ns:=...)

- 토픽 개별 변경이 아니라, 모든 토픽 앞에 /robot1 같은 네임스페이스를 일괄적으로 붙여서 발행하고 싶을 때 유용해.



