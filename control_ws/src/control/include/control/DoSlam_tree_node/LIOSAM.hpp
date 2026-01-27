# ifndef LIOSAM_HPP
# define LIOSAM_HPP

#include <behaviortree_cpp_v3/bt_factory.h>
#include <cstdlib>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>

class LIOSAM : public BT::AsyncActionNode
{
private:
  bool running;
  pid_t child_pid; 
  
public:
  explicit LIOSAM(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts(void);
  BT::NodeStatus tick() override;
  void stop_liosam(void);
};

# endif