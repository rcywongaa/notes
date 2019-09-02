#include <condition_variable>
#include <mutex>
#include <functional>
#include <string>
#include <chrono>
#include <thread>

class FuncBool
{
    public:
        FuncBool(bool initial_value, std::function<void()> true_func, std::function<void()> false_func, bool is_repeat = false, std::chrono::milliseconds period = std::chrono::milliseconds(100)) :
            is_alive(true),
            current_value(initial_value),
            TRUE_FUNC(true_func),
            true_runner(std::bind(&FuncBool::loopTrueFunc, this)),
            FALSE_FUNC(false_func),
            false_runner(std::bind(&FuncBool::loopFalseFunc, this)),
            IS_REPEAT(is_repeat),
            PERIOD(period)
        {}

        ~FuncBool()
        {
            is_alive = false;
            true_cv.notify_all();
            true_runner.join();
            false_cv.notify_all();
            false_runner.join();
        }

        void setTrue()
        {
            std::lock_guard<std::mutex> g(mtx);
            if (current_value != true)
            {
                current_value = true;
                if (IS_REPEAT) true_cv.notify_all();
                else TRUE_FUNC();
            }
        }

        void setFalse()
        {
            std::lock_guard<std::mutex> g(mtx);
            if (current_value != false)
            {
                current_value = false;
                if (IS_REPEAT) false_cv.notify_all();
                else FALSE_FUNC();
            }
        }

        void loopTrueFunc()
        {
            while(is_alive)
            {
                std::unique_lock<std::mutex> l(mtx);
                true_cv.wait(l);
                while (is_alive && current_value == true)
                {
                    TRUE_FUNC();
                    mtx.unlock();
                    std::this_thread::sleep_for(PERIOD);
                    mtx.lock();
                }
            }
        }

        void loopFalseFunc()
        {
            while(is_alive)
            {
                std::unique_lock<std::mutex> l(mtx);
                false_cv.wait(l);
                while (is_alive && current_value == false)
                {
                    FALSE_FUNC();
                    mtx.unlock();
                    std::this_thread::sleep_for(PERIOD);
                    mtx.lock();
                }
            }
        }

        explicit operator bool()
        {
            std::lock_guard<std::mutex> g(mtx);
            return current_value;
        }

    private:
        bool is_alive;
        std::mutex mtx;
        bool current_value;

        std::condition_variable true_cv;
        const std::function<void()> TRUE_FUNC;
        std::thread true_runner;

        std::condition_variable false_cv;
        const std::function<void()> FALSE_FUNC;
        std::thread false_runner;
        const bool IS_REPEAT;
        const std::chrono::milliseconds PERIOD;
};

void printTrue()
{
    printf("True\n");
}

int main(int argc, char** argv)
{
    FuncBool flag(false, printTrue, []{printf("False\n");}, true);
    flag.setTrue();
    if (flag) printf("1. IS TRUE\n");
    if (!flag) printf("1. IS FALSE\n");
    std::this_thread::sleep_for(std::chrono::seconds(3));
    flag.setFalse();
    if (flag) printf("2. IS TRUE\n");
    if (!flag) printf("2. IS FALSE\n");
    std::this_thread::sleep_for(std::chrono::seconds(3));
}


