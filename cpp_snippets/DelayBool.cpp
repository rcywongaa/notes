#include <condition_variable>
#include <mutex>
#include <chrono>
#include <thread>

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
                        is_interrupted = false;
                        continue;
                    }
                }
                else
                {
                    // timed out
                    current_value = next_value;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
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

int main(int argc, char** argv)
{
    DelayBool my_bool(false);
    if (my_bool) printf("TRUE\n");
    else printf("FALSE\n");
    my_bool.setTrue(std::chrono::seconds(3));
    while (true)
    {
        if (my_bool) printf("TRUE\n");
        else printf("FALSE\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

