# parking_GUI
13.Oct Push:
1. 使用app designer的第一个版本

7.Oct Push:
1. 修正了求车辆到障碍物最小距离的函数。
2. 调整了碰撞风险的评分百分比范围，但看起来还得在调整下。
3. 新增碰撞风险评分的附加条件：不发生碰撞才能有风险评分，否则风险评分为0分。
4. 调整了计算总分的条件，现在只要车辆右边六个角点进入库位即可获得评分。

5.Oct Push:
1. 修正了图像显示延迟问题，现在变为图像略微卡顿问题，眨眼补帧。
2. 挡位使能bug修复，之后还需要增加一个手动开关。

4.Oct Push:
5. 新增读取档位和使能信号的订阅器。
6. 新增了用于图像显示的订阅器。

29.Sep Push:
1. 更改了图像显示的逻辑，现在不需要点`开始泊车`也可以显示出实时收到的信息。

27.Sep Push:
1. 提供了人机比拼按钮，但是需要先有一次机器的数据。
2. 调整了界面背景。
3. 调整了图像显示的代码，现在显示速度在我的电脑没问题。

19.Sep Push:
1. 改变了评分显示样式。
2. 可以保存成绩了。
3. 新增调试面板，按`q`查看。
4. 把原来的plot改用set， fill 改用 patch。

5.Sep Push:
1. 优化了评分显示。
2. 可以保存数据了。
3. 合并了程序里面的一些数组。

27.July Push:
1. 添加了评分模块以及相应的显示控件。

21.July Push:
1. 在选择`结束泊车`以后，可以选择`显示轨迹`来显示泊车的轨迹，显示需要的时间15~30秒不等。
2. 在`配置`中设置了一个`一次显示`的开关量，默认选中，选中的时候显示轨迹比较快，但是重复显示需要的时间比较长；没选中的时候第一次显示轨迹需要比较长的时间，但是后面重复显示比较快，这个和算法有关系。

20.July Push:
1. 在`配置`当中新增了一个调整库位深度的输入栏。
2. 重新调整了库位显示。红色边框表示算法的目标库位，绿色边框表示可以停车的空间，左右两侧的梯形代表障碍物，实际上检测到的障碍物是近似于矩形的一个东西，它被包括在这个梯形范围内，这个不是很重要，如果需要显示出原来的障碍物形状也可以。

19.July Push:
1. 新增了一个配置框，点击`配置`以后才会显现交互模块，这个考虑到以后可能会有用途。
2. 改进了程序运行逻辑，点击`配置`（目前只能设置显示图像的x,y范围）以后，需要点击`开始泊车`才会进行数据的接收。
3. 添加了一个显示进程的文字框。
4. 数据显示前面增加一个选中框，如果ros没有发送该数据所属的topic，那么这个框不会被打勾，后面数据会显示NaN。打勾表示这个topic已经被接收。
5. 运行过程中，点击`结束泊车`可以图像的显示。
6. 如果需要记录下一次泊车，再点击`开始泊车`即可，程序会重新初始化数据。
7. 如果点击`重置`，会弹出最开始的配置框，并初始化数据。
8. 点`退出`会直接关闭该界面。

16.July Push:
  可以显示数据和图像
