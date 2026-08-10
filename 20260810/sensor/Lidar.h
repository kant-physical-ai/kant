#ifndef KIMHYUNHA_LIDAR_H
#define KIMHYUNHA_LIDAR_H

#include "Sensor.h"

class Lidar : public Sensor {
public:
    Lidar();
    explicit Lidar(std::string name) : Sensor(std::move(name)) {

    }

    ~Lidar() override;

    std::vector<double> read() override;

private:
    void connect();

    void disconnect();
};

#endif // KIMHYUNHA_LIDAR_H
