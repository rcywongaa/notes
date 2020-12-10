/**
 * A primitive type (in this case, bool)
 * where setting its value will automatically trigger the calling of a function
 * if is_repeat == true, the function keeps getting called, and takes the current value as input
 */

#include <type_traits>
#include <condition_variable>
#include <mutex>
#include <functional>
#include <string>
#include <chrono>
#include <thread>

template <typename T>
class SmartPrimitive
{
    public:
        SmartPrimitive(T initial_value, std::function<void(T)> func, bool is_repeat = false, std::chrono::milliseconds period = std::chrono::milliseconds(100)) :
            is_alive(true),
            current_value(initial_value),
            FUNC(func),
            runner(std::bind(&SmartPrimitive::loop, this)),
            IS_REPEAT(is_repeat),
            PERIOD(period)
        {}

        /** Delete copy constructor
         * std::thread is non-copyable
         * We don't want to deal with locking and copying class variables
         * std::mutex is non-copyable
         */
        SmartPrimitive(const FuncPrimitive&) = delete; // delete copy constructor

        /** Delete assignment operator
         * std::thread is non-copyable
         * We don't want to deal with locking and assigning class variables
         * std::mutex is non-assignable
         */
        SmartPrimitive& operator=(const FuncPrimitive&) = delete; // delete assignment operator

        /** Delete move constructor and operator
         * Moving std::thread objet invalidates the original thread object
         * std::mutex is non-moveable
         */
        SmartPrimitive(FuncPrimitive&&) = delete; // delete move constructor
        SmartPrimitive& operator=(FuncPrimitive&&) = delete; // delete move-assign operator

        ~SmartPrimitive()
        {
            is_alive = false;
            cv.notify_all();
            runner.join();
        }

        void set(T new_value)
        {
            std::lock_guard<std::mutex> g(mtx);
            if (current_value != new_value)
            {
                current_value = new_value;
                FUNC(current_value);
                if (IS_REPEAT) cv.notify_all();
            }
        }

        void loop()
        {
            while(is_alive)
            {
                std::unique_lock<std::mutex> l(mtx);
                cv.wait(l);
                while (is_alive)
                {
                    FUNC(current_value);
                    l.unlock();
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
        T current_value;

        std::condition_variable cv;
        const std::function<void(T)> FUNC;
        std::thread runner;

        const bool IS_REPEAT;
        const std::chrono::milliseconds PERIOD;
};

void printBool(bool b)
{
    if (b) printf("True\n");
    else printf("False\n");
}

int main(int argc, char** argv)
{
    SmartPrimitive<bool> flag(false, printBool, true);
    flag.set(true);
    if (flag) printf("1. IS TRUE\n");
    if (!flag) printf("1. IS FALSE\n");
    std::this_thread::sleep_for(std::chrono::seconds(3));
    flag.set(false);
    if (flag) printf("2. IS TRUE\n");
    if (!flag) printf("2. IS FALSE\n");
    std::this_thread::sleep_for(std::chrono::seconds(3));
}

