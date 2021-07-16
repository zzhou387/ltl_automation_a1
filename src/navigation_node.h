//
// Created by ziyi on 7/9/21.
//

#ifndef LTL_AUTOMATION_A1_NAVIGATION_NODE_H
#define LTL_AUTOMATION_A1_NAVIGATION_NODE_H

#include "behaviortree_cpp_v3/bt_factory.h"
#include <string>

using namespace BT;

namespace BTNav {
class LTLPreCheck: public ConditionNode
{
public:
    LTLPreCheck(const std::string& name, const NodeConfiguration& config) : ConditionNode(name, config){}

    static PortsList  providedPorts(){
        return { InputPort<int>("nav_goal")};
    }

    NodeStatus tick() override
    {

    }
};

class MoveAction : public CoroActionNode
{
public:
    MoveAction(const std::string& name, const NodeConfiguration& config) : CoroActionNode(name, config)
    {}

    static PortsList providedPorts()
    {
        return { InputPort<bool>("move_base_finished"), InputPort<bool>("move_base_idle"),
                 InputPort<int>("nav_goal"), OutputPort<std::string>("action") };
    }

    NodeStatus tick() override
    {
        auto nav_goal = getInput<int>("nav_goal");
        if(!nav_goal || nav_goal.value() == 0){
            return NodeStatus::FAILURE;
        }

        setOutput<std::string>("action", "MOVE_COMMAND");
        std::cout << name() << ": MOVE_COMMAND_TO_" << nav_goal.value() << " Yield" << std::endl;
        setStatusRunningAndYield();

        while (true)
        {
            auto move_base_idle = getInput<bool>("move_base_idle");

            auto move_base_finished = getInput<bool>("move_base_finished");
            if (move_base_finished && move_base_finished.value())
            {
                std::cout << name() << ": move_base is finidshed: SUCCESS" << std::endl;
                setOutput<std::string>("action", "NONE");
                return BT::NodeStatus::SUCCESS;
            }

            if (move_base_idle && move_base_idle.value())
            {
                std::cout << name() << ": move_base is idle: FAILURE" << std::endl;
                setOutput<std::string>("action", "NONE");
                return BT::NodeStatus::FAILURE;
            }

            setOutput<std::string>("action", "NONE");
            setStatusRunningAndYield();
        }
    }

    void halt() override
    {
        std::cout << this->name() << " TO_" << getInput<int>("nav_goal").value() << ": halt" << std::endl;
        CoroActionNode::halt();
    }

};

} // end namespace BTNav

#endif //LTL_AUTOMATION_A1_NAVIGATION_NODE_H
