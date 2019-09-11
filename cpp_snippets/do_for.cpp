#include <functional>
#include <chrono>
#include <thread>

void do_for(std::chrono::microseconds duration, std::chrono::microseconds period, std::function<void()> fn)
{
    auto now = std::chrono::system_clock::now();
    while (std::chrono::system_clock::now() <= now + duration)
    {
        fn();
        std::this_thread::sleep_for(period);
    }
}

int main(int argc, char** argv)
{
    bool my_bool = false;
    do_for(std::chrono::seconds(4), std::chrono::milliseconds(500), [&]{
        if (my_bool) printf("TRUE\n");
        else printf("FALSE\n");
    });
}
