#include <iostream>
#include <string>

// 클래스 선언
class Person {
private:
	std::string name; // 멤버 변수 (외부 차단)

public:
	// 1. 생성자 (객체가 만들어질 때 자동 실행)
	Person(std::string n) {
    	name = n;
    	std::cout << name << " 객체가 생성되었습니다!" << std::endl;
	}

	// 2. 멤버 함수 (기능)
	void sayHello() {
    	std::cout << "안녕하세요, 저는 " << name << "입니다!" << std::endl;
	}

	// 3. 소멸자 (객체가 소멸할 때 자동 실행) -> 여기서 '~'가 쓰입니다!
	~Person() {
    	std::cout << name << " 객체가 사라집니다!" << std::endl;
	}
};

int main() {
	// 객체 생성 (스택에 생성됨)
	Person p("철수");
	p.sayHello();
	// Person* p = new Person("철수");
	// p->sayHello();
	// delete p;

/*
Motor* moter_ptr = new Motor();

// ... 이런저런 작업 수행 ...
moter_ptr->start();

// 사용이 끝났다면 반드시 해제!
delete moter_ptr;
moter_ptr = nullptr; // 안전을 위해 포인터 비우기
*/

	return 0; // main 함수가 끝나며 p 객체가 소멸할 때 소멸자가 자동 호출됨
}

