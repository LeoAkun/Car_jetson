# include "control/main_tree_node/IsSlamMode.hpp"

IsSlamMode::IsSlamMode(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}

BT::PortsList IsSlamMode::providedPorts(void)
{
    // 定义一个输入端口：mode
    return { BT::InputPort<std::string>("mode") };
}

BT::NodeStatus IsSlamMode::tick()
{
    auto mode = getInput<std::string>("mode");

    if (mode.value() == "slam")
    {
        std::cout << "选为slam模式 ✅" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

