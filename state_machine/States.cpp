#include "States.hpp"

State::State(Target& target, Event& event) :
    target(target), events(events)
{}

State::StateTransition State::transition(std::unique_ptr<State>& state)
{
    return {};
}

IdleState::IdleState(Target& target, Event& event)
{
    printf("Begin IdleState\n");
}

State::StateTransition IdleState::transit()
{
    std::lock_guard<std::mutex> lock(events->mtx);
    if (events->event1)
    {
        events->event1 = false;
        return State::make_state_transition<OpeningState>();
    }
    else if (events->event2)
    {
        events->event2 = false;
        return State::make_state_transition<ClosingState>();
    }
}

OpeningState::OpeningState()
{
    printf("Begin OpeningState\n");
    target->open();
}

State::StateTransition OpeningState::transit()
{
    if (target->isOpen())
    {
        return State::make_state_transition<IdleState>();
    }
}

OpeningState::~OpeningState()
{
    printf("End OpeningState\n");
}

ClosingState::ClosingState()
{
    printf("Begin ClosingState\n");
    target->close();
}

State::StateTransition ClosingState::transit()
{
    if (target->isClose())
    {
        state = std::unique_ptr<IdleState>(new IdleState(pos, events));
    }
}

ClosingState::~ClosingState()
{
    printf("End ClosingState\n");
}
