c++
===


# apt update and c++ setting install
```shell
sudo apt update
sudo apt install -y build-essential cmake gdb valgrind git
```


# cmake
```shell
cmake --version
g++ --version
```

# execute 
```shell
g++ -Wall -Wextra -std=c++17 -o hello ./hello_world.cpp
./hello
```

# memory monitor compile
```shell
cmake -DCMAKE_BUILD_TYPE=Debug -DMEMORY_MONITOR=ON .
make

# or
g++ -fsanitize=address -g -Wall -Wextra -std=c++17 -o stack_heap_monitor ./stack_heap_monitor.cpp
```



# practice
- [hello_world](./hello_world.cpp)
- [calculator](./calculator.cpp)
- [class_test](./class_test.cpp)
1. **스택 vs 힙**: 지역 변수로 `Motor m;`(스택)과 `auto p = std::make_unique<Motor>();`(힙)을 만들고, 함수가 끝날 때 각각 어떻게 정리되는지 소멸자에 출력을 넣어 관찰해 보세요.
 - [stack_heap_monitor](./monitor/stack_heap_monitor.cpp)
2. **다형성**: `Sensor`(순수 가상 `read`)를 상속한 `Lidar`, `Imu`를 만들고, `std::vector<std::unique_ptr<Sensor>>`에 담아 다형성 루프로 읽어 보세요. 가상 소멸자를 일부러 빼 보고 경고/동작 차이를 확인해 보세요.
 - [sensor](./sensor/SensorTest.cpp) 
3. **STL**: 센서 이름→최근값을 `unordered_map`으로, 측정 로그를 `vector`로 관리하고, `std::count_if`로 "1m 이내 측정 개수"를 세어 보세요.
- [sensor](./sensor/SensorTest.cpp)
4. **템플릿**: 본문의 `clamp<T>`를 작성해 `double` 속도와 `int` 픽셀값에 모두 적용해 보세요.
- [sensor](./sensor/SensorTest.cpp)
5. **(심화) 누수 실험**: `new Motor()`를 delete 없이 반복하는 루프를 만들고, `valgrind`(또는 sanitizer `-fsanitize=address`)로 누수를 검출한 뒤, `make_unique`로 바꿔 누수가 사라지는지 확인해 보세
- [stack_heap_monitor](./monitor/stack_heap_monitor.cpp)



# 맨토님 결과
- [hello_world](./result/hello_world.cpp)