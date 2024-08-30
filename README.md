# 此为涵道SF09II02机型代码
该工程基于FreeRTOS编写，并使用C和C++混合编程。   
FreeRTOS系统和STM32芯片相关驱动初始化的代码都可以使用CubeIDE的ioc文件自动生成。用户主要关注USER文件夹下的代码   
注意！在Core/Src/freertos.c这个文件中，初始化之后需要手动加入头文件 `#include "system/system.hpp"`（该头文件位于USER/drivers/system下，将于下文价绍）
此外，还需要手动在自动生成的`void MX_FREERTOS_Init(void)`函数中加入
```c++
	LL_SPI_Enable(SPI4); //启动SPI4
	LL_SPI_SetRxFIFOThreshold(SPI4, LL_SPI_RX_FIFO_TH_QUARTER);
	HAL_Delay(200);
	LL_TIM_EnableCounter(TIM2);		//启动计时器2,单位为us

  mySystemInit(); // 飞控系统初始化函数
```
如果上述头文件和代码块都按照本工程下的方法，即在两行注释之间加入，那么通过ioc文件重新生成初始化代码后，应该不会再重新手动加入
其他自动生成的文件如有需要更改的，可在ioc中更改并重新生成代码，直接在生成后的代码中也可以更改，比如更改串口波特率，但不推荐，因为不会同步到ioc中

除了USER外，还有两个文件夹不是自动生成的，需要手动添加，即eigen3和matrix。eigen3是C++矩阵运算库，用于进行高效的矩阵运算，主要在ahrs和eskf中使用到；matrix是px4的开源矩阵运算库，主要在controlAllocation中使用到
CubeIDE工程添加外部文件的方法请自行查阅相关资料，我之后有时间会在这里补充。

接下来介绍USER文件夹  
### drivers文件夹下
system为用户编写的FreeRTOS系统文件，包括任务创建，信号量、消息队列定义等。   
hspi为用户编写的集成的STM32LL库的SPI库的SPI通信API，可以在需要使用SPI通信的地方直接调用   
userlib为用户编写的常用函数库，包括SCUT-RobotLab常用数据结构，常用宏定义，常用函数等  
openlog.h为SCUT-RobotLab编写的通过串口写入SD卡的函数库   
queueMessage.hpp为消息队列结构体定义  

### modules文件夹下  
ahrs为姿态航向参考系统，利用加速度计和磁力计通过ESKF（误差状态卡尔曼滤波）解算的到姿态  
alloc为优先级控制分配代码，用于优化涵道无人机操纵面冗余的问题   
bell为蜂鸣器任务，用于控制蜂鸣器发声
control为控制代码   
DPS368为气压计驱动代码   
eskf为在ahrs的基础上融合了通过GPS得到的位置和速度信息的15阶eskf  
gpsMag为获取并解码gps原始数据的代码
icm20602为获取加速度计的代码
led为指示led灯亮灭的代码   
~lsm303d为磁力计驱动代码，暂时未用到~   
motor为电机和舵机的驱动代码，还包括接收舵机小板上的磁力计原始数据   
mtf01为激光-光流一体化模组驱动代码  
openlog为通过串口将数据写入SD卡的驱动代码   
path_follow为路径规划部分代码（目前在control中使用到了Dubins曲线）  
pis为pid控制器设计相关代码  
power为采样电池电压相关代码   
~trajectory为航点轨迹规划相关代码，目前未用到，相关功能已经集成在了control中~  
tran_and_rc的功能有三个：与地面站双向通讯，通过基站接收遥控器数据和接收并转发RTK的校准信号  
~flow、gps_ins_EKF、ms5611、rm3100和transfer目前都未使用，这四个文件夹需要不加入编译，或者直接删除，以防止出现重定义的问题~
