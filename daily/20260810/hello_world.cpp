#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <math.h>
#include <cmath>


int add(int a, int b) {
    return a + b;
}
int sub(int a, int b) {
    return a - b;
}
int mul(int a, int b) {
    return a * b;
}

int divs(int a, int b) {
    return a / b;
}


int main() {
    // 2byte short
    short count = 10;

    // 4byte short
    int countInt = -10;

    // os마다 다르게 측정됨 byte long
    long countLong = 10;

    // unsigned
    unsigned int countUnsigned = -10;

    // 4byte float 실수
    float countFloat = 10.0;

    // 8byte double 실수
    double countDouble = 10.0;

    // 1bit boolean
    bool countBool = true;

    // 1byte char
    char countChar = 'a';

    // string
    std::string countString = "hello";

    // const variable
    const long countConst = 10;

    // constexpr
    constexpr long countConstexpr = 10;

    // auto variable
    auto countAuto = 10;


    // int array
    int arr[10] = {1,2,3,4,5,6,7,8,9,10};

    // vector int array
    std::vector<int> arrVec = {1,2,3,4,5,6,7,8,9,10};

    // array int array
    std::array<int, 10> arrArr = {1,2,3,4,5,6,7,8,9,10};


    // scope block
    {
        int count = 10;
        std::cout << "count: " << count << std::endl;
    }

    // single line variable declaration
    int a=7,b=3;
    std::cout << "a+b: " << (a+b) << std::endl;

    // right side operation
    std::cout << "10+10: " << 10+10 << std::endl;
    std::cout << "10*10: " << (10*10) << std::endl;
    std::cout << "10/10: " << (10/10) << std::endl;
    // fmod
    std::cout << "10fmod10: " << fmod(10.0f, 10.0f) << std::endl;
    // mod
    std::cout << "10%10: " << (10%10) << std::endl;

    //
    count = count + 1;
    count++;
    count--;
    ++count;
    --count;
    count += 1;
    count *= 4;


    // boolalpha
    // 1. 그냥 출력할 때
    std::cout << (count > 10) << std::endl;
    // 출력 결과: 1

    // 2. std::boolalpha를 쓸 때  한번 boolalpha 쓰게되면 내부 상태값바껴서 뒤쪽부터 나오는것 boolean 들은 true, false로 출력된다  뭐이러냐
    std::cout << std::boolalpha << (count > 10) << std::endl;
    // 출력 결과: true

    // 유지되는데요, 이걸 다시 원래의 1과 0(숫자) 출력 상태로 되돌리는 방법이 있습니다.
    // 바로 std::noboolalpha를 사용하는 것입니다!


    // and or operator
    std::cout << "and:" << (count > 10 && count < 20) << std::endl;
    std::cout << "or:" << (count > 10 || count < 20) << std::endl;


    // sqrt
    std::cout << "sqrt(100): " << std::sqrt(100) << std::endl;

    std::cout << "Count: " << count << std::endl;
    std::cout << "CountInt: " << countInt << std::endl;
    std::cout << "CountLong: " << countLong << std::endl;
    std::cout << "CountUnsigned: " << countUnsigned << std::endl;
    std::cout << "CountUnsigned: " << (unsigned)countUnsigned << std::endl;
    std::cout << "CountUnsigned: " << (unsigned short)countUnsigned << std::endl;
    std::cout << "CountUnsigned: " << (unsigned long)countUnsigned << std::endl;
    std::cout << "CountUnsigned: " << (unsigned long long)countUnsigned << std::endl;
    std::cout << "CountUnsigned: " << (unsigned int)countUnsigned << std::endl;
    std::cout << "CountUnsigned: " << (unsigned char)countUnsigned << std::endl;
    std::cout << "CountUnsigned: " << (unsigned short)countUnsigned << std::endl;


    std::cout << "CountFloat: " << countFloat << std::endl;
    std::cout << "CountDouble: " << countDouble << std::endl;
    std::cout << "CountBool: " << countBool << std::endl;
    std::cout << "CountChar: " << countChar << std::endl;
    std::cout << "CountString: " << countString << std::endl;

    // for each array   // 일반 array는 length, size같은거 없다
    for (int i = 0; i < 10; i++) {
        std::cout << "arr[" << i << "]: " << arr[i] << std::endl;
    }

    std::cout << "sizeof(int): " << sizeof(arr[0]) << std::endl;
    std::cout << "sizeof(int): " << sizeof(int) << std::endl;
    std::cout << "sizeof(int): " << arrVec.size() << std::endl;
    std::cout << "sizeof(float): " << arrArr.size() << std::endl;

    // for each
    for (auto i : arrVec) {
        std::cout << "arrVec[" << i << "]: " << i << std::endl;
    }

    // switch case
    char c = 'a';
    switch (c) {
        case 'a':
            std::cout << "a" << std::endl;
            break;
        case 'b':
            std::cout << "b" << std::endl;
            break;
        default:
            std::cout << "default" << std::endl;
            break;
    }

    //while
    int i = 0;
    while (i < 10) {
        std::cout << "i: " << i << std::endl;
        i++;

        if (i == 2) {
            continue;
        }

        if (i == 5) {
            break;
        }
    }


    // std::cout << "Hello, World!" << std::endl;
    return 0;
}
