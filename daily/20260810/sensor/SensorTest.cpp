#include <iostream>
#include <memory>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include "IMU.h"
#include "Lidar.h"
#include "MathUtils.h"
#include "Sensor.h"

int main() {
    std::cout << "Hello World!" << std::endl;
    // std::vector<std::unique_ptr<Sensor>> sensors = {
    //     std::make_unique<IMU>(),
    //     std::make_unique<Lidar>()
    // };

    /*
    2: 다형성: Sensor(순수 가상 read)를 상속한 Lidar, Imu를 만들고,
    std::vector<std::unique_ptr<Sensor>>에 담아 다형성 루프로 읽어 보세요.
    가상 소멸자를 일부러 빼 보고 경고/동작 차이를 확인해 보세요.
     */
    std::vector<std::unique_ptr<Sensor>> sensors;
    sensors.emplace_back(std::make_unique<IMU>("IMU Sensor"));
    sensors.emplace_back(std::make_unique<Lidar>("Lidar Sensor"));

    std::unordered_map<std::string, std::vector<double>> sensorDatas;
    for (auto& sensor : sensors) {
        std::cout << "sensor: "<<  sensor->getName() <<", isConnected:" << sensor->isConnected() << std::endl;
        auto data = sensor->read();
        sensorDatas[sensor->getName()] = data;
        for (size_t i = 0; i < data.size(); i++) {
            std::cout << data[i] << " ";
        }
        std::cout << std::endl;
    }

    /*
     3: STL: 센서 이름→최근값을 unordered_map으로, 측정 로그를 vector로 관리하고, std::count_if로 "1m 이내 측정 개수"를 세어 보세요.
     1m  = 여기에서 데이터값이 0.1가 1m라고 생각한다
    */
    for (auto& sensorData : sensorDatas) {
        // count_if
        auto count = std::count_if(sensorData.second.begin(), sensorData.second.end(), [](double value) {
            return value < 0.1;
        });
        std::cout << "sensor: " << sensorData.first << ", count: " << count << std::endl;
    }

    /*
    4: 템플릿: 본문의 clamp<T>를 작성해 double 속도와 int 픽셀값에 모두 적용해 보세요.
    */
    for (const auto& sensorData : sensorDatas) {
        const auto maxIt = std::max_element(sensorData.second.begin(), sensorData.second.end());
        if (maxIt != sensorData.second.end()) {
            std::cout << "sensor: " << sensorData.first << ", max value: " << *maxIt << std::endl;
        }

        double maxByClamp = 0.0;
        for (double value : sensorData.second) {
            maxByClamp = MathUtils::clamp(value, maxByClamp, 1.0);
        }
        std::cout << "sensor: " << sensorData.first << ", max value by clamp: " << maxByClamp << std::endl;
    }

    return 0;
}
