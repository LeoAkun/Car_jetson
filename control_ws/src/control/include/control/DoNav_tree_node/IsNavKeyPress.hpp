# ifndef IS_NAV_PRESS_HPP
# define IS_NAV_PRESS_HPP
#include <behaviortree_cpp_v3/bt_factory.h>

class IsNavKeyPress : public BT::ConditionNode
{
public:
    IsNavKeyPress(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts(void);
    BT::NodeStatus tick() override;
};

# endif