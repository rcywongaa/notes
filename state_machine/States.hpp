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
        State(Target* target, Events* events);

        void transition(std::unique_ptr<State>& state);

    protected:
        virtual void begin() {};
        // This function must be idempotent
        virtual void transition_impl(std::unique_ptr<State>& state) = 0;

        Target* target;
        Events* events;
        bool has_begun;
};

class IdleState : public State
{
    using State::State;

    protected:
        void begin() override final;
        void transition_impl(std::unique_ptr<State>& state) override final;
};

class OpeningState : public State
{
    using State::State;

    protected:
        void begin() override final;
        void transition_impl(std::unique_ptr<State>& state) override final;
};

class ClosingState : public State
{
    using State::State;

    protected:
        void begin() override final;
        void transition_impl(std::unique_ptr<State>& state) override final;
};
