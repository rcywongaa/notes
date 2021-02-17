#pragma once

#include <actionlib/client/simple_action_client.h>
#include "actionlib/client/action_client.h"
#include "actionlib/client/simple_goal_state.h"
#include "actionlib/client/simple_client_goal_state.h"
#include "actionlib/client/terminal_state.h"

template <class ActionSpec>
class SimpleActionClientInterface
{
    protected:
        ACTION_DEFINITION(ActionSpec);
    public:
        virtual void sendGoal(const Goal& goal) = 0;
        virtual void waitForResult() = 0;
        virtual void sendGoalAndWait(const Goal& goal) = 0;
        virtual actionlib::SimpleClientGoalState getState() = 0;
};

template <class ActionSpec>
class SimpleActionClientWrapper : public SimpleActionClientInterface<ActionSpec>
{
    protected:
        ACTION_DEFINITION(ActionSpec);

    public:
        SimpleActionClientWrapper(actionlib::SimpleActionClient<ActionSpec>& client) :
            client(client)
        {};

        virtual void sendGoal(const Goal& goal) override
        {
            client.sendGoal(goal);
        };

        virtual void waitForResult() override
        {
            client.waitForResult();
        };

        virtual void sendGoalAndWait(const Goal& goal) override
        {
            client.sendGoalAndWait(goal);
        };

        virtual actionlib::SimpleClientGoalState getState() override
        {
            return client.getState();
        };

    private:
        actionlib::SimpleActionClient<ActionSpec>& client;
};
