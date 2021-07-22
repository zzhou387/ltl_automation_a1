//
// Created by ziyi on 7/9/21.
//

#ifndef LTL_AUTOMATION_A1_NAVIGATION_NODE_H
#define LTL_AUTOMATION_A1_NAVIGATION_NODE_H

#include "behaviortree_cpp_v3/bt_factory.h"
#include <string>

using namespace BT;


namespace BT {
    typedef std::vector<std::string> LTLState;
    typedef std::vector<std::vector<std::string>> LTLState_Sequence;
    typedef std::vector<std::string> LTLAction_Sequence;
    typedef std::vector<std::string> Locomotion_status;

    template <> inline LTLState convertFromString(StringView str)
    {

        // Vector of strings separated by semicolons
        auto parts = splitString(str, ';');
        LTLState output;
        output.reserve( parts.size() );
        for(auto & part : parts){
            output.push_back(std::string(part.data(), part.size()));
        }

        return output;
    }

    template <> inline LTLState_Sequence convertFromString(StringView str)
    {

        // Customized splitting from: "a,b,c;d,e,f"
        auto parts = splitString(str, ';');
        LTLState_Sequence output;
        LTLState output_state;
        for(auto & part : parts){
            auto part_0 = splitString(part, ',');
            for(auto & part_00 : part_0){
                output_state.push_back(std::string(part_00.data(), part_00.size()));
            }
            output.push_back(output_state);
        }

        return output;
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
        if(!desired_state_seq || !current_state) {
            std::cout << name() << ": Fetch ltl state: " << "FAILED" << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }
        auto desired_state = desired_state_seq.value()[0];
        if(desired_state == current_state.value()){
            std::cout << name() << ": Check ltl state: " << "SUCCESS" << std::endl;
            std::cout << "CURRENT: ";
            for(const auto& curr_state : current_state.value()){
                 std::cout << curr_state << " ";
            }
            std::cout << std::endl << std::endl;
            return NodeStatus::SUCCESS;
        } else {
            std::cout << name() << ": Check ltl state: " << "FAILED" << std::endl;
            std::cout << "GET: ";
            for(const auto& curr_state : current_state.value()){
                std::cout << curr_state << " ";
            }
            std::cout << std::endl;
            std::cout << "DESIRED: ";
            for(const auto& des_state : desired_state){
                std::cout << des_state << " ";
            }
            std::cout << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }

    }

};

class LocomotionStatusCheck: public ConditionNode
{
public:
    LocomotionStatusCheck(const std::string& name, const NodeConfiguration& config) : ConditionNode(name, config){}

    static PortsList  providedPorts(){
        return { InputPort<BT::Locomotion_status>("locomotion_status")};
    }

    NodeStatus tick() override
    {

//        auto int_1 = getInput<int>("in_arg1");
        auto loco_status = getInput<BT::Locomotion_status>("locomotion_status");
        if(!loco_status) {
            std::cout << name() << ": Fetch locomotion status: " << "FAILED" << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }
        if(loco_status.value()[1] != "ESTOP"){
            std::cout << name() << ": Check locomotion operating mode: " << "SUCCESS" << std::endl;
            std::cout << "CURRENT: ";
            for(const auto& curr_state : loco_status.value()){
                std::cout << curr_state << " ";
            }
            std::cout << std::endl << std::endl;
            return NodeStatus::SUCCESS;
        } else {
            std::cout << name() << ": Check locomotion operating mode: " << "FAILED" << std::endl;
            std::cout << "CURRENT: ";
            for(const auto& curr_state : loco_status.value()){
                std::cout << curr_state << " ";
            }
            std::cout << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }

    }

};

class UpdateLTL : public SyncActionNode
{
public:
    UpdateLTL(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config){}

    static PortsList  providedPorts(){
        return {BidirectionalPort<BT::LTLAction_Sequence>("action_sequence"),
                BidirectionalPort<BT::LTLState_Sequence>("ltl_state_desired_sequence"),
                OutputPort<std::string>("nav_goal")};
    }

    NodeStatus tick() override {
        auto action_sequence = getInput<BT::LTLAction_Sequence>("action_sequence");
        auto ltl_state_seq = getInput<BT::LTLState_Sequence>("ltl_state_desired_sequence");
        if(action_sequence && ltl_state_seq) {
            BT::LTLAction_Sequence act_seq = action_sequence.value();
            BT::LTLState_Sequence state_seq = ltl_state_seq.value();
            act_seq.erase(act_seq.begin());
            state_seq.erase(state_seq.begin());

            setOutput<BT::LTLState_Sequence>("ltl_state_desired_sequence", state_seq);
            setOutput<BT::LTLAction_Sequence>("action_sequence", act_seq);
            setOutput<std::string>("nav_goal", act_seq[0]);
            std::cout << name() << ": Update ltl state: " << "SUCCESS" << std::endl << std::endl;
            return NodeStatus::SUCCESS;
        } else {
            std::cout << name() << ": Update ltl state: " << "FAILED" << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }
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
        std::cout << name() << ": MOVE_COMMAND: " << nav_goal.value() << " Yield" << std::endl << std::endl;
        setStatusRunningAndYield();

        while (true)
        {
            auto move_base_idle = getInput<bool>("move_base_idle");

            auto move_base_finished = getInput<bool>("move_base_finished");
            if (move_base_finished && move_base_finished.value())
            {
                std::cout << name() << ": move_base is finidshed: SUCCESS" << std::endl << std::endl;
                setOutput<std::string>("action", "NONE");
                return BT::NodeStatus::SUCCESS;
            }

            if (move_base_idle && move_base_idle.value())
            {
                std::cout << name() << ": move_base is idle: FAILURE" << std::endl << std::endl;
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

class RecoveryStand : public CoroActionNode
{
public:
    RecoveryStand (const std::string& name, const NodeConfiguration& config) : CoroActionNode(name, config)
    {}

    static PortsList providedPorts()
    {
        return { InputPort<BT::Locomotion_status>("locomotion_status")};
    }

    NodeStatus tick() override
    {
        auto loco_status = getInput<BT::Locomotion_status>("locomotion_status");
        std::string current_fsm = loco_status.value()[0];
        if(!loco_status || current_fsm != "PASSIVE"){
            return NodeStatus::FAILURE;
        }

//        setOutput<std::string>("action", "MOVE_COMMAND");
        std::cout << name() << ": Try recovery stand: " << " Yield" << std::endl << std::endl;
        setStatusRunningAndYield();

        while (true)
        {
            loco_status = getInput<BT::Locomotion_status>("locomotion_status");
            current_fsm = loco_status.value()[0];

            if (loco_status && current_fsm == "STAND_UP")
            {
                std::cout << name() << ": recovery standup is finidshed: SUCCESS" << std::endl << std::endl;
                return BT::NodeStatus::SUCCESS;
            }

            if (!loco_status)
            {
                std::cout << name() << ": recovery standup is: FAILURE" << std::endl << std::endl;
                return BT::NodeStatus::FAILURE;
            }

            setStatusRunningAndYield();
        }
    }

    void halt() override
    {
        std::cout << this->name() << ": halt" << std::endl;
        CoroActionNode::halt();
    }

};

class LocomotionStart : public CoroActionNode
{
public:
    LocomotionStart (const std::string& name, const NodeConfiguration& config) : CoroActionNode(name, config)
    {}

    static PortsList providedPorts()
    {
        return { InputPort<BT::Locomotion_status>("locomotion_status")};
    }

    NodeStatus tick() override
    {
        auto loco_status = getInput<BT::Locomotion_status>("locomotion_status");
        std::string current_fsm = loco_status.value()[0];
        if(!loco_status || current_fsm != "PASSIVE"){
            return NodeStatus::FAILURE;
        }

//        setOutput<std::string>("action", "MOVE_COMMAND");
        std::cout << name() << ": Try recovery stand: " << " Yield" << std::endl << std::endl;
        setStatusRunningAndYield();

        while (true)
        {
            loco_status = getInput<BT::Locomotion_status>("locomotion_status");
            current_fsm = loco_status.value()[0];

            if (loco_status && current_fsm == "STAND_UP")
            {
                std::cout << name() << ": recovery standup is finidshed: SUCCESS" << std::endl << std::endl;
                return BT::NodeStatus::SUCCESS;
            }

            if (!loco_status)
            {
                std::cout << name() << ": recovery standup is: FAILURE" << std::endl << std::endl;
                return BT::NodeStatus::FAILURE;
            }

            setStatusRunningAndYield();
        }
    }

    void halt() override
    {
        std::cout << this->name() << ": halt" << std::endl;
        CoroActionNode::halt();
    }

};
} // end namespace BTNav

#endif //LTL_AUTOMATION_A1_NAVIGATION_NODE_H
