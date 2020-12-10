/**
 * An example of how to bind a lambda function
 */

#include <functional>
#include <string>

class MyClass
{
    public:
        void func(std::string a)
        {
            printf("%s\n", a.c_str());
        }
};

int main(int argc, char** argv)
{
    using namespace std::placeholders;

    MyClass my_class;

    std::bind(
    [&](int a)
    {
            my_class.func(std::to_string(a));
    }, _1)(9);
}
