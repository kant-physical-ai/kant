#include "IMU.h"

IMU::IMU() {
    connect();
}

IMU::~IMU() {
    disconnect(); // 객체가 소멸할 때 안전하게 연결 해제
    std::cout << this->getName()<<": IMU 소멸자 실행: 연결을 해제했습니다.\n";
}
std::vector<double> IMU::read() {
    std::vector<double> data = {0.2,0.5,0.02,0.5,0.08,0.06,0.7,0.7};
    return data;
}

void IMU::connect() {
    connected_ = true;
}
void IMU::disconnect() {
    connected_ = false;
}
