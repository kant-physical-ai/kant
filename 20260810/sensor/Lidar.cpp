#include "Lidar.h"

#include <vector>

Lidar::Lidar() {
    connect();
}

Lidar::~Lidar() {
    disconnect();
    std::cout << this->getName() << ": Lidar 소멸자 실행: 연결을 해제했습니다.\n";
}

std::vector<double> Lidar::read() {
    std::vector<double> data = {0.5,0.02,0.1,0.3,0.1,0.04,0.7,0.9};
    return data;
}

void Lidar::connect() {
    connected_ = true;
}

void Lidar::disconnect() {
    connected_ = false;
}