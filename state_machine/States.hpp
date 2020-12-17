#include <mutex>
#include <memory>

struct Events
{
    std::mutex mtx;
    bool event1 = false;
    bool event2 = false;
};

class State
{
    public:
        using StateTransition = std::optional<std::function<std::unique_ptr<State>(Target&, Events&)>>;

        template <typename T>
        static StateTransition make_state_transition()
        {
            return std::make_unique<T>(target, events);
        };

    public:
        State(Target& target, Event& event) :
            target(target),
            events(events)
        {};                                            

        virtual StateTransition transit()
        {};

        ~State()
        {};

    protected:
        Target& target;
        Events& events;
};

class IdleState : public State
{
    using State::State;

    public:
        IdleState();
        virtual StateTransition transit();
        ~IdleState();
};

class OpeningState : public State
{
    using State::State;

    public:
        OpeningState();
        virtual StateTransition transit();
        ~OpeningState();
};

class ClosingState : public State
{
    using State::State;

    public:
        ClosingState();
        virtual StateTransition transit();
        ~ClosingState();
};
