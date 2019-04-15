#include "State.hpp"

class Controller
{
    public:
        Controller(Target& target);
        ~Controller();
        void open();
        void close();
        void update();
    private:
        bool is_continue;
        Target& target;
        std::unique_ptr<State> state;
        std::thread state_machine_runner;
        Events events;
};
