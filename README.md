本系统融合了 FSM 设计， A/D 转换，边缘触发中断。 基于期中
实验进行了两点改进： 第一，将东与北两个方向的车流量进行量化，
以此来给定某方向车辆的通行时间，即绿灯时间，车流量大的一方将
有更多的通行时间，此处用到了数模转换。第二，加入关闭信号灯系
统的功能，在任何情况下，按下关闭系统的开关，将导致所有灯熄灭，
松开关闭系统的开关，系统继续正常运行， 且并不需要从起始状态开
始运行，即不同于断电式的关闭系统， 此处用到了边缘触发的中断。