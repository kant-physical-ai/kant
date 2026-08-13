#include <iostream>

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
    std::cout << "Hello World calculator" << std::endl;

    std::cout << "Choose operation: " << std::endl;
    std::cout << "1: add +" << std::endl;
    std::cout << "2: sub -" << std::endl;
    std::cout << "3: mul *" << std::endl;
    std::cout << "4: div /" << std::endl;
    int ch;
    std::cin >> ch;
    std::cout << std::endl;

    int num1, num2;
    std::cout << "Enter two numbers: ";
    std::cin >> num1 >> num2;

    switch (ch) {
        case 1:
            std::cout << num1 << " + " << num2 << " = " << add(num1, num2) << std::endl;
            break;
        case 2:
            std::cout << num1 << " - " << num2 << " = " << sub(num1, num2) << std::endl;
            break;
        case 3:
            std::cout << num1 << " * " << num2 << " = " << mul(num1, num2) << std::endl;
            break;
        case 4:
            if (num2 == 0) {
                std::cout << "Error: Division by zero" << std::endl;
            } else {
                std::cout << num1 << " / " << num2 << " = " << divs(num1, num2) << std::endl;
            }
            break;
    }

    return 0;
}