# include "control/DoSlam_tree_node/LIOSAM.hpp"

LIOSAM::LIOSAM(const std::string& name, const BT::NodeConfiguration& config):
    BT::AsyncActionNode(name, config) {
        this->running = false;
        this->child_pid = -1;
    }

BT::PortsList LIOSAM::providedPorts(void)
{
    return { BT::OutputPort<std::string>("mode"),
             BT::InputPort<std::string>("read_key"),
             BT::OutputPort<std::string>("write_key")};
}

void LIOSAM::stop_liosam(void)
{
    if(this->child_pid != -1)
    {
        kill(-getpgid(this->child_pid), SIGINT);
        waitpid(child_pid, NULL, 0);
        this->child_pid = -1;
        return;
    }
    std::cout << "[ LIOSAM ] WARN : 没有启动liosam子进程 ,无法关闭liosam!!" << std::endl;
}

BT::NodeStatus LIOSAM::tick()
{
    // 确保只启动一次
    if(this->running == false)
    {  
        child_pid = fork();
        // 子进程
        if(child_pid == 0)
        {
            
            this->running = true;

            // 设置进程组来启动liosam，便于管理
            setpgid(0, 0);
            std::system("bash -c 'source /home/akun/workspace/CAR/LIO-SAM/install/setup.bash'");
            execlp("ros2", "ros2", "launch", "lio_sam", "run.real_launch.py", (char*)NULL);
            
            this->running = false;
            std::cerr << "[ LIOSAM ] ERROR : execlp失败!!" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        // 父进程
        else if (child_pid > 0) 
        {
            
        }
        else
        {
            std::cerr << "[ LIOSAM ] ERROR : fork失败!!" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    }

    if(getInput<std::string>("read_key").value() == "q")
    {
        this->running = false;
        setOutput("mode", "idle" );
        setOutput("write_key", "none" );
        this->stop_liosam();
        std::cout << "[ LIOSAM ] INFO : 关闭LIOSAM" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        this->running = true;
        std::cout << "[ LIOSAM ] INFO : LIOSAM正在运行中" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

}