
# practice
- [subscriber.py](subscriber.py)
- [multithread_subscriber.py](multithread_subscriber.py)
- [hard_work_subscriber.py](hard_work_subscriber.py)
- [publisher.py](publisher.py)
- [robot.urdf](robot.urdf) 
  - ros2 launch urdf_tutorial display.launch.py model:=/home/pa/workspaces/pa/source/kimhyunha/20260812/robot.urdf

# 왜 rclpy가 import되나?
rclpy는 venv에 없는 시스템 ROS 설치본에서 import됩니다.
```text
     - 위치: /opt/ros/humble/local/lib/python3.10/dist-packages/ — ROS 2 humble이                                             
       제공하는 파이썬 패키지                                                                                                 
     - requirements.txt는 pip freeze로 .venv에 pip 설치된 것만 기록한 것이므로                                                
       ROS 재단(apt)로 설치된 rclpy는 포함되지 않습니다.                                                                      
                                                                                                                              
     import가 되는 이유는 /opt/ros/humble 환경이 아마 PYTHONPATH나 ROS 셸 환경(                                               
     source /opt/ros/humble/setup.bash)을 통해 venv 파이썬에도 보이기 때문입니다.                                             
     단, 주의할 점이 하나 있는데:                                                                                             
                                                                                                                              
     - .venv는 Python 3.10 기반, /opt/ros/humble도 Python 3.10이라 버전은                                                     
       맞습니다.                                                                                                              
     - test 실행 때 봤던 launch_testing pytest 플러그인 충돌도 이 ROS 경로가                                                  
       PYTHONPATH에 섞여 있는 게 원인입니다.                                           
                                                                                                                              
     즉, ROS 전용(20260812/) 코드와 순수 pip 라이브러리(src/libs, python/)는 같은      
     인터프리터를 쓰되 서로 다른 공급자(pip vs ROS apt)에서 온 것입니다.                
     requirements.txt는 어디까지나 pip 설치분 목록일 뿐, ROS 패키지는 apt/rosdep                                              
     이 관리합니다.                                
```