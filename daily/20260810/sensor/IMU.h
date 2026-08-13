#ifndef KIMHYUNHA_IMU_H
#define KIMHYUNHA_IMU_H

#include "Sensor.h"


class IMU : public Sensor {
public:
    IMU();
    explicit IMU(std::string name) : Sensor(std::move(name)) {}

    ~IMU() override;

    std::vector<double> read() override;

private:
    void connect();

    void disconnect();
};


#endif //KIMHYUNHA_IMU_H
