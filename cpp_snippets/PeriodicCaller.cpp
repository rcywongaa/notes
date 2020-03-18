#include <chrono>
#include <functional>
#include <thread>

class PeriodicCaller
{
    public:
        PeriodicCaller(std::function<void()> func, std::chrono::microseconds period) :
            is_continue(true),
            func(func),
            period(period),
            runner(std::bind(&PeriodicCaller::loop, this))
        {};

        void loop()
        {
            while (is_continue)
            {
                func();
                std::this_thread::sleep_for(period);
            }
        };

        ~PeriodicCaller()
        {
            is_continue = false;
            runner.join();
        };
    private:
        bool is_continue;
        std::function<void()> func;
        std::chrono::microseconds period;
        std::thread runner;
};
