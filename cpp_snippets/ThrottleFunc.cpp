#include <functional>
#include <chrono>
#include <optional>

class ThrottleFunc
{
    public:
        ThrottleFunc(std::function<void()>func, std::chrono::milliseconds throttle_period) : FUNC(func), PERIOD(throttle_period)
        {
            ;
        }

        void operator()()
        {
            if (!last_called || std::chrono::system_clock::now() - *last_called >= PERIOD)
            {
                FUNC();
                last_called = std::chrono::system_clock::now();
            }
        }
    private:
        std::optional<std::chrono::system_clock::time_point> last_called;
        const std::function<void()> FUNC;
        const std::chrono::milliseconds PERIOD;
};
