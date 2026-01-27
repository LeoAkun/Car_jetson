# include "control/DoNav_tree_node/Nav.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>

Nav::Nav(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config) 
{
    // map_manager放在最后
    this->sim_programs[0] = "source " SIM_SETUP_PATH " && ros2 launch launch_sim run.launch.py"; // 启动仿真环境
    this->sim_programs[1] = "source " RE_LOCALIZATION_SETUP_PATH " && ros2 launch re_localization run.sim_launch.py"; // 启动re_localization服务
    this->sim_programs[2] = "source " NAV2_INIT_POSE_SETUP_PATH " && ros2 run nav2_init_pose nav2_init_pose"; // 启动nav2_init
    this->sim_programs[3] = "source " LIOSAM_SETUP_PATH " && ros2 launch lio_sam run.sim_launch.py"; // 启动LIOSAM定位算法
    this->sim_programs[4] = "source " NAV2_SETUP_PATH " && ros2 launch nav2 run.sim_launch.py";// 启动nav2导航
    this->sim_programs[5] = "source " GLOBAL_GRAPH_SETUP_PATH " && ros2 run global_graph_planner global_graph_planner";// 启动global_graph_planner服务
    this->sim_programs[6] = "source " MAP_MANAGER_SETUP_PATH " && ros2 run map_manager map_manager";// 启动map_manager

    this->process_group_id = -1;
    this->running = false;
    this->manager_map_config_path = "/home/akun/workspace/CAR/utils/map_manger/install/map_manager/share/map_manager/config/generate_gird_config.yaml";
}

BT::PortsList Nav::providedPorts(void)
{
    return { BT::OutputPort<std::string>("mode"),
            BT::OutputPort<std::string>("write_key"),
            BT::InputPort<std::string>("destination"),
            BT::InputPort<std::string>("read_key")};
}

BT::NodeStatus Nav::tick()
{   
    int i;
    pid_t pid;
    auto place = getInput<std::string>("destination");
    std::cout << "Nav! " << "目的地为: " << place.value() << std::endl;
    
    // 确保只启动一次
    if(this->running == false)
    {  
        // 依次创建多个子进程，执行launch_sim, re_local, nav2_init, liosam, global_graph, map_manager
        for (i = 0; i < NUM_PROCESS; i++)
        {
            pid = fork();
            if(pid < 0)
            {
                std::cerr << "[ Nav ] ERROR : fork失败!!" << std::endl;
                exit(EXIT_FAILURE);
            }
            else if(pid == 0)
            {
                // 子进程
                this->execute_ros2_process(i, this->process_group_id);
                exit(EXIT_SUCCESS);
            }
            else
            {
                // 父进程
                if (i == 0) 
                {
                    // 父进程主动将子进程设为组长
                    this->process_group_id = pid;
                    if (setpgid(pid, pid) != 0) {
                        perror("父进程设置 PGID 失败");
                    }
                }
                // 延迟20ms， 防止fork太快
                usleep(20000);
            }
        }
        this->running = true;
    }

    if(getInput<std::string>("read_key").value() == "q")
    {
        // 运行完变为idle模式
        this->running = false;
        setOutput("mode", "idle" );
        setOutput("write_key", "none" );
        this->stop_nav();
        std::cout << "[ Nav ] INFO : 关闭Nav" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        this->running = true;
        std::cout << "[ Nav ] INFO : Nav正在运行中..." << std::endl;
        return BT::NodeStatus::RUNNING;
    }
}

void Nav::execute_ros2_process(int index, pid_t pgid)
{
    // 进程组管理
    if (index == 0)
    {
        // 如果是第一个子进程，传入的参数是(0, -1)，主进程
        if( setpgid(0, 0) == 0) 
        {
            // this->process_group_id = getpid();
        } 
        else 
        {
            exit(EXIT_FAILURE);
        }
    }
    else
    {   
        // 如果是map_manager，则修改其目的地的yaml参数
        if(index == NUM_PROCESS - 1)
        {
            // 读取并修改目的地参数
            YAML::Node config = YAML::LoadFile(this->manager_map_config_path);
            config["generate_gird_config"]["place_name"] = getInput<std::string>("destination").value();
            std::ofstream fout(this->manager_map_config_path);
            fout << config;
        }

        // 后续子进程：加入到第一个子进程创建的组中
        if (setpgid(0, pgid) == 0) {
             printf("  ✅ 进程 %d (PID: %d) 加入进程组 (PGID: %d)\n", index + 1, getpid(), getpgrp());
        } else {
            perror("加入进程组失败");
            exit(EXIT_FAILURE);
        }
    }

    char *args[] = {"bash", "-c", sim_programs[index], NULL};
    execvp("/bin/bash", args);
    perror("execvp 失败 (无法启动 bash)");
    exit(EXIT_FAILURE);
}

void Nav::stop_nav(void)
{
    if(this->process_group_id != -1)
    {
        kill(-this->process_group_id, SIGINT);

        // 等待进程组的所有进程
        int status;
        pid_t wpid;
        while ((wpid = waitpid(-this->process_group_id, &status, 0)) > 0) {}

        this->process_group_id = -1;
        return;
    }
}
