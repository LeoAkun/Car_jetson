# ifndef include_hpp
# define include_hpp

# include <rclcpp/rclcpp.hpp>
# include <ament_index_cpp/get_package_share_directory.hpp>
# include <behaviortree_cpp_v3/bt_factory.h>

// 节点
# include "control/main_tree_node/IsSlamMode.hpp"
# include "control/main_tree_node/IsNavMode.hpp"
# include "control/main_tree_node/Idle.hpp"
# include "control/DoSlam_tree_node/LIOSAM.hpp"
# include "control/DoNav_tree_node/Nav.hpp"
# include "control/DoSlam_tree_node/IsLiosamKeyPress.hpp"
# include "control/DoNav_tree_node/IsNavKeyPress.hpp"

# endif