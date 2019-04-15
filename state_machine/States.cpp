#include "States.hpp"

State::State(Target* target, Events* events) :
    target(target), events(events), has_begun(false)
{}

void State::transition(std::unique_ptr<State>& state)
{
    if (!has_begun)
    {
        has_begun = true;
        begin();
    }
    transition_impl(state);
}

void IdleState::begin()
{
    printf("Begin IdleState\n");
}

void IdleState::transition_impl(std::unique_ptr<State>& state)
{
    std::lock_guard<std::mutex> lock(events->mtx);
    if (events->event1)
    {
        events->event1 = false;
        state = std::unique_ptr<OpeningState>(new OpeningState(target, events));
    }
    else if (events->event2)
    {
        events->event2 = false;
        state = std::unique_ptr<ClosingState>(new ClosingState(target, events));
    }
}

void OpeningState::begin()
{
    printf("Begin OpeningState\n");
    target->open();
}

void OpeningState::transition_impl(std::unique_ptr<State>& state)
{
    if (target->isOpen())
    {
        state = std::unique_ptr<IdleState>(new IdleState(pos, events));
    }
}

void ClosingState::begin()
{
    printf("Begin ClosingState\n");
    target->close();
}

void ClosingState::transition_impl(std::unique_ptr<State>& state)
{
    if (target->isClose())
    {
        state = std::unique_ptr<IdleState>(new IdleState(pos, events));
    }
}
