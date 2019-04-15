#include "Controller.hpp"
#include <chrono>
#include <functional>

using namespace std;

const std::chrono::milliseconds FSM_PERIOD_ms(10);

Controller::Controller(Target& target) :
    is_continue(true),
    target(target),
    state(std::unique_ptr<IdleState>(new IdleState(&target, &events))),
    state_machine_runner(std::bind(&Controller::update, this))
{}

Controller::~Controller()
{
    is_continue = false;
    state_machine_runner.join();
}

void Controller::update()
{
    printf("Start updating FSM\n");
    while (is_continue)
    {
        state->transition(state);
        std::this_thread::sleep_for(FSM_PERIOD_ms);
    }
    printf("Stop updating FSM\n");
}

void Controller::triggerEvent1()
{
    std::lock_guard<std::mutex> lock(events.mtx);
    events.event1 = true;
}

void Controller::triggerEvent2()
{
    std::lock_guard<std::mutex> lock(events.mtx);
    events.event2 = true;
}
