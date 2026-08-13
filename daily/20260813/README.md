# TF2
- [manipulator.urdf](manipulator.urdf)
  - ros2 launch urdf_tutorial display.launch.py model:=/home/pa/workspaces/pa/source/kimhyunha/20260813/manipulator.urdf
- [quadruped.urdf](quadruped.urdf)
  - ros2 launch urdf_tutorial display.launch.py model:=/home/pa/workspaces/pa/source/kimhyunha/20260813/quadruped.urdf

# ros2
- workspace: [ros2_ws](ros2_ws)
- 참고 터틀봇: https://github.com/ROBOTIS-GIT/turtlebot3/tree/main/turtlebot3_example/turtlebot3_example
## launch

## package
```shell
ros2 pkg create tt
```
### C++
```shell
ros2 pkg create --build-type ament_cmake my_cpp_pkg --dependencies rclcpp std_msgs --node-name sample_node
```
### python
```shell
ros2 pkg create --build-type ament_python my_py_pkg --dependencies rclpy std_msgs --node-name sample_node
```

### 옵션,설명,예시
- --build-type {ament_cmake, ament_python, cmake}",패키지의 빌드 방식을 설정합니다. (기본값: ament_cmake),--build-type ament_python
- --dependencies [의존성 목록... ],package.xml 및 빌드 파일에 추가할 의존성 패키지들을 지정합니다.,--dependencies rclcpp std_msgs
- --node-name NODE_NAME,패키지 내부 생성 시 기본으로 포함될 빈 템플릿 실행 파일(Node)의 이름을 지정합니다.,--node-name my_node
- --library-name LIBRARY_NAME,C++ 패키지의 경우 기본으로 생성될 라이브러리(공유 라이브러리) 이름을 지정합니다.,--library-name my_lib
- --description DESCRIPTION,package.xml에 들어갈 패키지 설명을 작성합니다. (따옴표로 묶어야 함),"--description ""My custom ROS 2 package"""
- --license LICENSE,패키지의 라이선스를 지정합니다. (?를 전달하면 지원하는 라이선스 목록을 볼 수 있음),--license Apache-2.0
- --maintainer-name NAME,패키지 메인테이너(관리자)의 이름을 지정합니다.,"--maintainer-name ""Hong Gildong"""
- --maintainer-email EMAIL,패키지 메인테이너의 이메일 주소를 지정합니다.,"--maintainer-email ""gildong@email.com"""
- --destination-directory PATH,패키지를 생성할 경로를 지정합니다. (기본값: 현재 디렉토리),--destination-directory ~/ros2_ws/src
- --package-format {2, 3}",package.xml의 포맷 버전을 선택합니다.,--package-format 3




## colcon
```shell
cd workspace
colcon build
```

## ros2 execute
```shell
source install/setup.bash
# package run
ros2 run my_cpp_pkg sample_node

# launch run
ros2 launch my_cpp_pkg sample_launch.py
```


## python
- 만약 colcon build 할때 catkin_pkg가 없다고 하면 설치해줘라 pip install catkin_pkg
