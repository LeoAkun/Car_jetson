# ifndef IDLE_HPP
# define IDLE_HPP

#include <behaviortree_cpp_v3/bt_factory.h>
#include "control/utils/utils.hpp"

class Idle : public BT::AsyncActionNode
{
public:
  explicit Idle(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts(void);
  BT::NodeStatus tick() override;
};


# endif