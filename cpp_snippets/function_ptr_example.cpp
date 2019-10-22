/**
 * A simple std::function example with default empty function argument
 */

#include <functional>

void my_func(int a, int b) {
    printf("a = %d, b = %d\n", a, b);
}

void call_func(std::function<void()>func = []{}) {
    func();
}

int main(int argc, char** argv) {
    call_func();
    //call_func(std::bind(my_func, 1, 2));
}
