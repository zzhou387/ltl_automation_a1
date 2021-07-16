//
// Created by ziyi on 7/8/21.
//

#include <unistd.h>
#include <deque>
#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/builtin_string.h>
#include <ltl_automaton_msgs/TransitionSystemStateStamped.h>
#include <ltl_automaton_msgs/LTLPlan.h>
#include "navigation_node.h"
#include <yaml-cpp/yaml.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

using namespace BT;

class LTLA1Planner{
public:
    LTLA1Planner(){
        client_ = std::make_shared<Client>("/move_base", true);
        task_sub_ = nh_.subscribe<ltl_automaton_msgs::LTLPlan>("/prefix_plan", 1, &LTLA1Planner::callbackActionSequence);
        ltl_state_pub_ = nh_.advertise<ltl_automaton_msgs::TransitionSystemStateStamped>("ts_state", 10, true);
        run();
    }
    ~LTLA1Planner() = default;
    void run(){
        // set up the blackboard to cache the lower level codes running status
        is_first = true;
        replan = false;
//        factory_.registerNodeType<>()

        my_blackboard_->set("move_base_finished", false);
        my_blackboard_->set("move_base_idle", false);
        my_blackboard_->set("nav_goal", 0);
        my_blackboard_->set("ltl_state_current", "NONE");
        my_blackboard_->set("ltl_state_desired", "NONE");
        my_blackboard_->set("action", "NONE");
        my_blackboard_->debugMessage();

        bt_filepath = "../resources/default_tree.xml";
        nh_.getParam("bt_filepath", bt_filepath);
        std::string ts_filepath;
        nh_.getParam("transition_system_textfile", ts_filepath);
        transition_system_ = YAML::Load(ts_filepath);



        ROS_INFO("tree file: %s\n", bt_filepath.c_str());
        auto tree = std::make_unique<BT::Tree>();
        auto zmq_publisher = std::make_unique<PublisherZMQ>(*tree);

        while(ros::ok()){
            if(is_first && replan){
                tree = std::make_unique<BT::Tree>(factory_.createTreeFromFile(bt_filepath, my_blackboard_));
                zmq_publisher = std::make_unique<PublisherZMQ>(*tree);
                is_first = false;
                replan = false;
            } else if (!is_first && replan) {
                zmq_publisher.reset();
                tree = std::make_unique<BT::Tree>(factory_.createTreeFromFile(bt_filepath, my_blackboard_));
                zmq_publisher = std::make_unique<PublisherZMQ>(*tree);
                replan = false;
            }
            // update input
            bool move_base_finished = false;
            bool move_base_idle = false;

            // update input
            if (client_->isServerConnected())
            {
                switch (client_->getState().state_)
                {
                    case actionlib::SimpleClientGoalState::REJECTED:
                    case actionlib::SimpleClientGoalState::PREEMPTED:
                    case actionlib::SimpleClientGoalState::ABORTED:
                    case actionlib::SimpleClientGoalState::LOST:
                        move_base_idle = true;
                        break;
                    case actionlib::SimpleClientGoalState::SUCCEEDED:
                        move_base_finished = true;
                        move_base_idle = true;
                        break;
                    case actionlib::SimpleClientGoalState::PENDING:
                    case actionlib::SimpleClientGoalState::ACTIVE:
                    case actionlib::SimpleClientGoalState::RECALLED:
                    default:
                        break;
                }
            }

            my_blackboard_->set("move_base_finished", move_base_finished);
            my_blackboard_->set("move_base_idle", move_base_idle);
//            my_blackboard_->set

// bt
            auto result = tree->tickRoot();
            std::string action;
            int nav_goal;
            my_blackboard_->get(std::string("action"), action);
            my_blackboard_->get(std::string("nav_goal"), nav_goal);

            // output
            move_base_msgs::MoveBaseGoal current_goal;
            if (client_->isServerConnected())
            {
                if (action == "MOVE_COMMAND")
                {
                    if (nav_goal != 0)
                    {
                        auto top = pose_queue.front();
                        pose_queue.pop_front();
                        current_goal.target_pose = top;
                        client.sendGoal(current_goal);
                        ROS_INFO("MOVE");
                    }
                }
            }
            ros::spinOnce();
            sleep(1);
        }


    }

    void callbackActionSequence(const ltl_automaton_msgs::LTLPlan& msg){
        // The xml changes go here
        auto action = msg.action_sequence;
        auto ts_state = msg.ts_state_sequence;
        std::vector<std::string> desired_state_seq;
        for(auto state : ts_state){
            desired_state_seq.push_back(state.states.at(0));
        }
        assert(!replan);
        replan = true;
    }





private:
    std::shared_ptr<Client> client_;
    BehaviorTreeFactory factory_;
    move_base_msgs::MoveBaseGoal current_goal_;
    Blackboard::Ptr my_blackboard_ = Blackboard::create();
    ros::NodeHandle nh_;
    std::string bt_filepath;
    std::vector<std::string> desired_state_seq_;
    YAML::Node transition_system_;

    ros::Subscriber task_sub_;
    ros::Publisher ltl_state_pub_;

    bool is_first;
    bool replan;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "a1_ltl_bt");
    LTLA1Planner runner;
    return 0;
}
