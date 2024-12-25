
#include <iostream>
#include <conio.h> 
#include <iostream>
#include <thread>
#include <chrono>


void printDataPeriodically() {
    int count = 0;
    while (true) {
        std::cout << "Data: " << "有数据" << std::endl;
        count++;

        std::this_thread::sleep_for(std::chrono::milliseconds(25));  // 每隔 5 秒打印一次

        if (_kbhit()) 
        { 
            // 检测是否有键盘输入
            char key = _getch();  // 获取按下的键
            if (key == 'K' || key == 'T'|| key == 'k' || key == 't'  )
            {
                std::cout << "You pressed: " << key << std::endl;
            }
        }
    }
}
// int main()
// {
//     std::thread t(printDataPeriodically);
//     t.join();

//     return 0;
// }

