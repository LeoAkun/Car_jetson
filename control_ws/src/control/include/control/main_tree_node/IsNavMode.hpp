# ifndef IS_NAV_MODE_HPP
# define IS_NAV_MODE_HPP
#include <behaviortree_cpp_v3/bt_factory.h>

class IsNavMode : public BT::ConditionNode
{
public:
    IsNavMode(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts(void);
    BT::NodeStatus tick() override;
};


# endif