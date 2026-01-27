# ifndef include_hpp
# define include_hpp

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class Utils
{
public: 
    Utils();

    // 非阻塞获取按键值，测试用
    static int kbhit();
};

# endif