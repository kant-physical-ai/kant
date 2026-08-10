#include <iostream>
#include <memory>

class Motor {
private:
    std::string name;

public:
    // Motor(std::string name) {
    //     std::cout << "Motor" << name << std::endl;
    //     this->name = name;
    // }
    Motor(std::string name) : name(name) {
        std::cout << "Motor" << name << " and " << std::endl;
    }

    void run(int speed) {
        std::cout << "Motor" << name << " running at speed: " << speed << std::endl;
    }

    ~Motor() {
        std::cout << "~Motor" << name << std::endl;
    }
};

int main() {


    // 스코프를 벗어나면 자동 소멸됨
    for (int i = 0; i < 10; i++) {
        int z=3;
        Motor motor("stack");
        motor.run(100);
    }


    // point 모던 클래스 사용법으로 하게되면 명확하게 delete해주지 않는이상 소멸안됨
    // 5: (심화) 누수 실험: new Motor()를 delete 없이 반복하는 루프를 만들고, valgrind(또는 sanitizer -fsanitize=address)로 누수를 검출한 뒤, make_unique로 바꿔 누수가 사라지는지 확인해 보세
    for (int i = 0; i < 10; i++) {
        Motor* motor = new Motor("pointer");
        motor->run(100);
        // delete motor; // 명확하게 해줘야함  이거 넣고 뺴고하면서 누수 있는지 확인해봐야함
        // g++ -fsanitize=address -g -Wall -Wextra -std=c++17 -o stack_heap_monitor ./stack_heap_monitor.cpp

    }

    // 포인터 접근이지만 그래도 관리를 해주는거라
    for (int i = 0; i < 10; i++) {
        // auto motor = std::make_unique<Motor>("MakeUnique");
        std::unique_ptr<Motor> motor = std::make_unique<Motor>("MakeUnique");
        motor->run(100);
    }


    return 0;
}
