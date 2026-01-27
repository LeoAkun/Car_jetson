# include "control/DoSlam_tree_node/IsLiosamKeyPress.hpp"
# include "control/utils/utils.hpp"

IsLiosamKeyPress::IsLiosamKeyPress(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) 
{
    setOutput("key", "none" );
}

BT::PortsList IsLiosamKeyPress::providedPorts(void)
{
    return { BT::OutputPort<std::string>("key")};
}

BT::NodeStatus IsLiosamKeyPress::tick()
{
    char c;

    // std::cout << "IsLiosamKeyPress" << std::endl;
    
    c = Utils::kbhit();
    if(c == 'q')
    {
        setOutput("key", "q" );
        std::cout << "[ IsLiosamKeyPress ] INFO : 按键按下!" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::RUNNING;
    }
}