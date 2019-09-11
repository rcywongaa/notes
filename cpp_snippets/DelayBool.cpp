#include <condition_variable>
#include <mutex>
#include <chrono>
#include <thread>
#include <functional>

class DelayBool
{
    public:
        DelayBool(bool initial_value) :
            is_alive(true),
            current_value(initial_value),
            next_value(initial_value),
            is_interrupted(false),
            delay(std::chrono::microseconds::max()),
            runner(&DelayBool::loop, this)
            {}

        ~DelayBool()
        {
            is_alive = false;
            cv.notify_all();
            runner.join();
        }

        void setTrue(std::chrono::microseconds delay)
        {
            std::lock_guard<std::mutex> lock(mtx);
            if (next_value != true || this->delay != delay)
            {
                this->delay = delay;
                next_value = true;
                is_interrupted = true;
                cv.notify_all();
            }
        }

        void setFalse(std::chrono::microseconds delay)
        {
            std::lock_guard<std::mutex> lock(mtx);
            if (next_value != false || this->delay != delay)
            {
                this->delay = delay;
                next_value = false;
                is_interrupted = true;
                cv.notify_all();
            }
        }

        // Prevent implicit conversion of the resultant bool to other types
        explicit operator bool()
        {
            std::lock_guard<std::mutex> g(mtx);
            return current_value;
        }

        void loop()
        {
            while (is_alive)
            {
                std::unique_lock<std::mutex> lock(mtx);
                if (cv.wait_for(lock, delay, [this]{return this->is_interrupted;}))
                {
                    if (is_interrupted)
                    {
                        printf("INTERRUPTED\n");
                        is_interrupted = false;
                        continue;
                    }
                }
                else
                {
                    // timed out
                    current_value = next_value;
                }
            }
        }

    private:
        bool is_alive;

        std::mutex mtx;
        std::condition_variable cv;
        bool current_value;
        bool next_value;
        std::chrono::microseconds delay;
        bool is_interrupted;
        std::thread runner;
};

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
    DelayBool my_bool(false);
    printf("Setting true in 3s\n");
    my_bool.setTrue(std::chrono::seconds(3));
    do_for(std::chrono::seconds(2), std::chrono::milliseconds(500), [&]{
        if (my_bool) printf("TRUE\n");
        else printf("FALSE\n");
    });
    printf("Setting false in 2s\n");
    my_bool.setFalse(std::chrono::seconds(3));
    do_for(std::chrono::seconds(4), std::chrono::milliseconds(500), [&]{
        if (my_bool) printf("TRUE\n");
        else printf("FALSE\n");
    });
}
