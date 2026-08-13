#ifndef KIMHYUNHA_SENSOR_H
#define KIMHYUNHA_SENSOR_H
#include <iostream>
#include <string>
#include <utility>
#include <vector>
class Sensor {
public:
    Sensor() = default;
    explicit Sensor(std::string name) : name_(std::move(name)) {}
    virtual ~Sensor() = default;          // 가상 소멸자 (상속 시 필수!)
    virtual std::vector<double> read() = 0;  // = 0: 순수 가상 (자식이 반드시 구현)
    bool isConnected() const { return connected_; }
    void setName(const std::string& name) { name_ = name; }
    const std::string& getName() const { return name_; }
protected:
    bool connected_ = false;
    std::string name_;
};

#endif // KIMHYUNHA_SENSOR_H
