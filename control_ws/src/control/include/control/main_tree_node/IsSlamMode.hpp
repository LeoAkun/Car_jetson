# ifndef IS_SLAM_HPP
# define IS_SLAM_HPP
#include <behaviortree_cpp_v3/bt_factory.h>

class IsSlamMode : public BT::ConditionNode
{
public:
    IsSlamMode(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts(void);
    BT::NodeStatus tick() override;
};

# endif