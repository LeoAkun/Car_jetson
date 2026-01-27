# include "control/main_tree_node/IsNavMode.hpp"

IsNavMode::IsNavMode(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}

BT::PortsList IsNavMode::providedPorts(void)
{
    // 定义一个输入端口：mode
    return { BT::InputPort<std::string>("mode") };
}

BT::NodeStatus IsNavMode::tick()
{
    auto mode = getInput<std::string>("mode");
    
    if (mode.value() == "nav")
    {
        std::cout << "选为nav模式 ✅" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

