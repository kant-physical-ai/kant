#include <iostream>  // std::cout 등 표준 입출력 스트림
#include <vector>    // std::vector 동적 배열 컨테이너
#include <memory>    // std::unique_ptr, std::make_unique 스마트 포인터
#include <random>    // std::mt19937, std::uniform_real_distribution 난수 생성

// 모든 센서의 공통 인터페이스를 정의하는 추상 기반 클래스.
// 이 클래스 자체로는 객체를 만들 수 없고, 반드시 파생 클래스(IMU, Lidar 등)를
// 통해서만 사용된다 (name()이 순수 가상 함수이기 때문).
class Sensor {
public:
    // 가상 소멸자: Sensor*를 통해 파생 클래스 객체를 delete할 때
    // 파생 클래스의 소멸자까지 올바르게 호출되도록 보장한다.
    // (다형성을 쓰는 기반 클래스는 반드시 가상 소멸자를 가져야 함)
    virtual ~Sensor() = default;

    // 순수 가상 함수(= 0): 파생 클래스가 반드시 오버라이드해야 하는 함수.
    // 센서의 이름(예: "IMU", "Lidar")을 반환한다.
    virtual const char* name() const = 0;

    // 센서가 측정한 데이터 벡터를 읽기 전용 참조로 반환.
    // const 참조를 반환하므로 호출 측에서 복사 비용 없이 조회만 가능하다.
    const std::vector<double>& data() const { return data_; }

protected:
    // 실제 센서 측정값을 저장하는 벡터. 파생 클래스 생성자에서 채워진다.
    // protected이므로 파생 클래스에서는 접근 가능하지만 외부에서는 불가능.
    std::vector<double> data_;

    // count개의 난수를 [lo, hi] 범위에서 생성해 벡터로 반환하는 헬퍼 함수.
    // static이므로 특정 객체 인스턴스 없이 Sensor::randomValues(...)처럼 호출 가능하며,
    // 파생 클래스들이 실제 센서 데이터처럼 보이는 값을 흉내 내는 데 사용한다.
    static std::vector<double> randomValues(size_t count, double lo, double hi) {
        // std::random_device로 시드를 얻고, 메르센 트위스터 엔진(mt19937)으로 난수 생성.
        // static으로 선언해 함수가 호출될 때마다 엔진을 새로 만들지 않고 재사용한다.
        static std::mt19937 gen{std::random_device{}()};
        // [lo, hi] 범위의 균등 분포(실수) 정의.
        std::uniform_real_distribution<double> dist(lo, hi);
        // 요청한 개수(count)만큼 크기를 갖는 벡터 생성.
        std::vector<double> values(count);
        // 벡터의 각 원소를 난수로 채운다.
        for (auto& v : values) v = dist(gen);
        return values;
    }
};

// IMU(관성 측정 장치, Inertial Measurement Unit)를 흉내 내는 클래스.
// public으로 Sensor를 상속해 다형성(Sensor* / Sensor&로 다룰 수 있음)을 구현한다.
class IMU : public Sensor {
public:
    // 생성자에서 가속도계 3축(x,y,z) + 자이로스코프 3축(x,y,z), 총 6개의 값을
    // -10.0 ~ 10.0 범위의 난수로 채워 data_에 저장한다.
    IMU() { data_ = randomValues(6, -10.0, 10.0); }  // accel xyz + gyro xyz

    // 기반 클래스의 순수 가상 함수 name()을 오버라이드하여 "IMU" 반환.
    const char* name() const override { return "IMU"; }
};

// Lidar(레이저 거리 측정 센서)를 흉내 내는 클래스.
class Lidar : public Sensor {
public:
    // 8개의 거리(range) 측정값을 0.0 ~ 30.0미터 범위의 난수로 채운다.
    Lidar() { data_ = randomValues(8, 0.0, 30.0); }  // range readings (m)

    const char* name() const override { return "Lidar"; }
};

int main() {

    // 서로 다른 타입(IMU, Lidar)의 센서 객체를 하나의 컨테이너에 담기 위해
    // 공통 기반 타입인 Sensor에 대한 unique_ptr(스마트 포인터) 벡터를 사용한다.
    // unique_ptr을 쓰면 벡터가 소멸될 때 각 센서 객체도 자동으로 delete되어
    // 메모리 누수를 걱정할 필요가 없다(RAII).
    std::vector<std::unique_ptr<Sensor>> sensors;

    // make_unique로 IMU, Lidar 객체를 힙에 생성하고 그 소유권을 벡터에 넘긴다(emplace_back).
    // 이때 Sensor 포인터로 업캐스팅되어 벡터에 저장되므로, 벡터 입장에서는
    // 실제 타입이 IMU인지 Lidar인지 몰라도 된다 (다형성).
    sensors.emplace_back(std::make_unique<IMU>());
    sensors.emplace_back(std::make_unique<Lidar>());

    // 벡터에 담긴 각 센서를 순회하며 결과를 출력.
    // sensor->name()은 실제 객체의 타입(IMU/Lidar)에 따라 다른 함수가 호출된다
    // (virtual 함수를 통한 동적 바인딩/런타임 다형성).
    for (const auto& sensor : sensors) {
        std::cout << "Sensor: " << sensor->name() << " -> [";
        // 각 센서가 가진 측정값들을 콤마로 구분해 출력.
        for (size_t i = 0; i < sensor->data().size(); ++i) {
            std::cout << sensor->data()[i];
            // 마지막 원소가 아니면 콤마와 공백을 추가로 출력.
            if (i + 1 < sensor->data().size()) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }

    return 0;  // 프로그램 정상 종료
}