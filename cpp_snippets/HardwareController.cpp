/**
 * A proposed better way of controlling hardware,
 * instead of having to manually deal with retries, reconnections, delays, waits, etc.
 */

#include <thread>
#include <array>
#include <mutex>

class HardwareController
{
    public:
        HardwareController() :
            is_alive(true),
            is_at_required_state(true),
            fail_count(0)
        {
            loop_runner = std::thread(&HardwareController::loop, this);
        }

        ~HardwareController()
        {
            is_alive = false;
            if (loop_runner.joinable()) loop_runner.join();
        }

        bool setOutputState(std::array<bool, 10> desired_state)
        {
            {
                std::lock_guard<std::mutex> guard(mtx);
                is_at_required_state = false;
                required_output_state = desired_state; // Potentially long copy, better to have individual functions for writing to individual states
            }
            for (auto start_time = std::chrono::system_clock::now();
                    std::chrono::system_clock::now() < start_time + TRY_SET_DURATION;
                    std::this_thread::sleep_for(RETRY_PERIOD))
            {
                std::lock_guard<std::mutex> guard(mtx);
                if (is_at_required_state)
                {
                    return true;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_PERIOD));
            }
            return false;
        }

        std::array<bool, 5> getInputState()
        {
            return current_input_state;

        }

    protected:
        void loop()
        {
            while (is_alive)
            {
                {
                    std::lock_guard<std::mutex> guard(mtx);
                    // read input states
                    if (!is_at_required_state)
                    {
                        try
                        {
                            // set output states...
                            is_at_required_state = true;
                            fail_count = 0;
                        }
                        catch (...)
                        {
                            fail_count++;
                            // attempt to reconnect / recover hardware if necessary
                        }
                    }
                }
                std::this_thread::sleep_for(LOOP_PERIOD);
            }
        }


    private:
        std::mutex mtx;
        bool is_alive;
        std::thread loop_runner;
        const std::chrono::milliseconds LOOP_PERIOD{100};

        bool is_at_required_state;
        std::array<bool, 10> required_output_state;
        std::array<bool, 5> current_input_state;

        unsigned int fail_count;

        const std::chrono::milliseconds RETRY_PERIOD{50};
        const std::chrono::milliseconds TRY_SET_DURATION{500};
};

