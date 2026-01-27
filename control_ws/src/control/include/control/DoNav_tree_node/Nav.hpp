# ifndef NAV_HPP
# define NAV_HPP

#include <behaviortree_cpp_v3/bt_factory.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

#define NUM_PROCESS 7

#define SIM_SETUP_PATH "/home/akun/workspace/CAR/simulation_ws/install/setup.bash"
#define RE_LOCALIZATION_SETUP_PATH "/home/akun/workspace/CAR/utils/re_localization/install/setup.bash"
#define NAV2_INIT_POSE_SETUP_PATH "/home/akun/workspace/CAR/nav2/install/setup.bash"
#define LIOSAM_SETUP_PATH "/home/akun/workspace/CAR/LIO-SAM/install/setup.bash"
#define NAV2_SETUP_PATH "/home/akun/workspace/CAR/nav2/install/setup.bash"
#define GLOBAL_GRAPH_SETUP_PATH "/home/akun/workspace/CAR/utils/global_graph_planner/install/setup.bash"
#define MAP_MANAGER_SETUP_PATH "/home/akun/workspace/CAR/utils/map_manger/install/setup.bash"

class Nav : public BT::AsyncActionNode
{
private:
  bool running;
  char *sim_programs[NUM_PROCESS];
  pid_t process_group_id;
  std::string manager_map_config_path;
public:
  explicit Nav(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts(void);
  BT::NodeStatus tick() override;
  void stop_nav(void);

private:
  void execute_ros2_process(int index, pid_t pgid);
};

# endif