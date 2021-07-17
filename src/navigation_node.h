//
// Created by ziyi on 7/9/21.
//

#ifndef LTL_AUTOMATION_A1_NAVIGATION_NODE_H
#define LTL_AUTOMATION_A1_NAVIGATION_NODE_H

#include "behaviortree_cpp_v3/bt_factory.h"
#include <string>

using namespace BT;


namespace BT {
    typedef std::vector<StringView> LTLState;
    typedef std::vector<std::vector<StringView>> LTLState_Sequence;
    typedef std::vector<StringView> LTLAction_Sequence;

    template <> inline LTLState convertFromString(StringView str)
    {

        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        LTLState output;
//        if (parts.size() != 2)
//        {
//            throw RuntimeError("invalid input)");
//        }
//        else{
//            Position2D output;
//            output.x     = convertFromString<double>(parts[0]);
//            output.y     = convertFromString<double>(parts[1]);
//            return output;
//        }
        for(int i=0; i<parts.size(); i++){
            output.push_back(parts[i]);
        }
    }

    template <> inline LTLState_Sequence convertFromString(StringView str)
    {

        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        LTLState output;
        for(int i=0; i<parts.size(); i++){
            output.push_back(parts[i]);
        }
    }

} //end namespace BT

namespace BTNav {
class LTLPreCheck: public ConditionNode
{
public:
    LTLPreCheck(const std::string& name, const NodeConfiguration& config) : ConditionNode(name, config){}

    static PortsList  providedPorts(){
        return { InputPort<BT::LTLState>("ltl_state_current"), InputPort<BT::LTLState_Sequence>("ltl_state_desired_sequence")};
    }

    NodeStatus tick() override
    {

//        auto int_1 = getInput<int>("in_arg1");
        auto current_state = getInput<BT::LTLState>("ltl_state_current");
        auto desired_state_seq = getInput<BT::LTLState_Sequence>("ltl_state_desired_sequence");
        if(!desired_state_seq) {
            return NodeStatus::FAILURE;
        }
        auto desired_state = desired_state_seq.value()[0];
        if(desired_state == current_state){
            return NodeStatus::SUCCESS;
        } else {
            return NodeStatus::FAILURE;
        }

    }

};

class UpdateLTL : public SyncActionNode
{
public:
    UpdateLTL(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config){}

    static PortsList  providedPorts(){
        return { InputPort<BT::LTLState>("ltl_state_current"), InputPort<BT::LTLState_Sequence>("ltl_state_desired_sequence")};
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
                 InputPort<std::string>("nav_goal"), OutputPort<std::string>("action") };
    }

    NodeStatus tick() override
    {
        auto nav_goal = getInput<std::string>("nav_goal");
        if(!nav_goal || nav_goal.value() == "NONE"){
            return NodeStatus::FAILURE;
        }

        setOutput<std::string>("action", "MOVE_COMMAND");
        std::cout << name() << ": MOVE_COMMAND: " << nav_goal.value() << " Yield" << std::endl;
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
        std::cout << this->name() << getInput<std::string>("nav_goal").value() << ": halt" << std::endl;
        CoroActionNode::halt();
    }

};

} // end namespace BTNav

#endif //LTL_AUTOMATION_A1_NAVIGATION_NODE_H
