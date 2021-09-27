#include "iostream"
class A {
 public:
  A() {
		a_ = std::string("a_mamer");
	}
	 std::string a_;
};
class B {
 public:
	B(A* a) {
		a_ = a;
		std::cout << "B.a_ = " << a_->a_ << std::endl;
	};
	~B() {
		std::cout << "Deconstuct b" << std::endl;
		delete a_;//如果不主动delete掉a，那么析构函数默认是不delete掉a的
	}
	A* a_;
};
int main() {
 A* a = new A();
 printf("Before deconstruct B: a = %s\n",a->a_.c_str());
 {
	 B b(a);
 }

 printf("After deconstruct B: a = %s\n",a->a_.c_str());
 delete a;
}
