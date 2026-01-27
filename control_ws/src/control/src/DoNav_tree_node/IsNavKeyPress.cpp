# include "control/DoNav_tree_node/IsNavKeyPress.hpp"
# include "control/utils/utils.hpp"

IsNavKeyPress::IsNavKeyPress(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) 
{
    setOutput("key", "none" );
}

BT::PortsList IsNavKeyPress::providedPorts(void)
{
    return { BT::OutputPort<std::string>("key")};
}

BT::NodeStatus IsNavKeyPress::tick()
{
    char c;

    // std::cout << "IsLiosamKeyPress" << std::endl;
    
    c = Utils::kbhit();
    if(c == 'q')
    {
        setOutput("key", "q" );
        std::cout << "[ IsNavKeyPress ] INFO : 按键按下!" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::RUNNING;
    }
}