#include "control/include.hpp"

class MainTree
{
public:
    static void register_factory(BT::BehaviorTreeFactory& factory)
    {
        factory.registerNodeType<IsSlamMode>("IsSlamMode");
        factory.registerNodeType<IsNavMode>("IsNavMode");
        factory.registerNodeType<Idle>("Idle"); 
    }
};

class DoSlamTree
{
public:
    static void register_factory(BT::BehaviorTreeFactory& factory)
    {
        factory.registerNodeType<LIOSAM>("LIOSAM"); 
        factory.registerNodeType<IsLiosamKeyPress>("IsLiosamKeyPress");
    }
};

class DoNavTree
{
public:
    static void register_factory(BT::BehaviorTreeFactory& factory)
    {
        factory.registerNodeType<Nav>("Nav"); 
        factory.registerNodeType<IsNavKeyPress>("IsNavKeyPress");
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    BT::BehaviorTreeFactory factory;

    MainTree::register_factory(factory);
    DoSlamTree::register_factory(factory);
    DoNavTree::register_factory(factory);

    // 加载树
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("control");
    std::string tree_file = package_share_dir + "/my_tree/main_tree.xml";
    std::cout << "路径:" << tree_file << std::endl;
    auto tree = factory.createTreeFromFile(tree_file);
    // BT::StdCoutLogger logger_cout(tree);
    
    // 运行tick
    while (rclcpp::ok())
    {
        tree.tickRootWhileRunning();  // 自动处理 RUNNING 状态
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    rclcpp::shutdown();
    return 0;
}