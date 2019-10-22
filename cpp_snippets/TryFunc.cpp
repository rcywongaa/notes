/**
 * This function class returns true and calls the underlying function
 * if conditions are met (e.g. called less than X times, called within X seconds)
 * and returns false once that condition expires and no longer calls the underlying function
 */

#include <functional>
#include <chrono>

void my_func() {
    printf("my_func\n");
}

class TimedTryFunc
{
    public:
        TimedTryFunc(std::function<void()>func, std::chrono::milliseconds try_duration) : FUNC(func), TRY_DURATION(try_duration)
        {
            is_first_try = true;
        }
        bool operator()()
        {
            if (is_first_try)
            {
                FUNC();
                is_first_try = false;
                deadline = std::chrono::system_clock::now() + TRY_DURATION;
            }
            else
            {
                if (std::chrono::system_clock::now() < deadline)
                {
                    FUNC();
                    return true;
                }
                else return false;
            }
        }
    private:
        const std::function<void()> FUNC;
        const std::chrono::milliseconds TRY_DURATION;
        std::chrono::system_clock::time_point deadline;
        bool is_first_try;
};

class CountTryFunc
{
    public:
        CountTryFunc(std::function<void()>func, unsigned int num_times) : FUNC(func), MAX_NUM_TRIES(num_times)
        {
            num_tries = 0;
        }
        bool operator()()
        {
            if (num_tries < MAX_NUM_TRIES)
            {
                FUNC();
                num_tries++;
                return true;
            }
            else return false;
        }
    private:
        const std::function<void()> FUNC;
        const unsigned int MAX_NUM_TRIES;
        unsigned int num_tries;
};

int main(int argc, char** argv)
{
    CountTryFunc try_my_func(my_func, 3);
    while (try_my_func())
    {
        ;
    }

    //TimedTryFunc try_my_func(my_func, std::chrono::milliseconds(2000));
    //while (try_my_func())
    //{
        //;
    //}
}
