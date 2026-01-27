# include "control/main_tree_node/Idle.hpp"

Idle::Idle(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config) 
{
    setOutput("mode", "idle" );
}

BT::PortsList Idle::providedPorts(void)
{
    // 定义两个输出端口
    return { BT::OutputPort<std::string>("mode"),
            BT::OutputPort<std::string>("destination") };
}

BT::NodeStatus Idle::tick()
{
    
    char c;
    std::cout << "[ Idle ] INFO : Idle " << std::endl;
    c = Utils::kbhit();
    if(c == 's')
    {
        // 1.启动建图时，车身x方向必须指向IMU磁北方向
        // 2.必须要记录启动位置时的经纬度 
        // 3.地图之间必须要有重叠部分
        setOutput("mode", "slam" );
        return BT::NodeStatus::SUCCESS;
    }
    else if(c == 'n')
    {
        setOutput("mode", "nav" );
        // setOutput("destination", "Southwest Jiaotong University (Xipu Campus)" );
        setOutput("destination", "西南交通大学犀浦校区" );
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::SUCCESS;
    
}