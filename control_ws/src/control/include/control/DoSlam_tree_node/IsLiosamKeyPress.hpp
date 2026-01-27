# ifndef IS_KEY_PRESS_HPP
# define IS_KEY_PRESS_HPP
#include <behaviortree_cpp_v3/bt_factory.h>

class IsLiosamKeyPress : public BT::ConditionNode
{
public:
    IsLiosamKeyPress(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts(void);
    BT::NodeStatus tick() override;
};

# endif