# 第三版聚类代码思路
主要有是单帧调试和多帧检测两个功能。
1. 单帧调试：
写入需要调试的pcd文件个数以及序列号，会对点云进行删除平面、下采样以及聚类操作

根据自己的需求选择可视化函数（写了两类，一直是直接展示，另一种是提取出聚类进行上色区分展示）

当得到聚类之后，可以在可视化界面上使用鼠标回调函数得到这个聚类的大小、最小外接长方体参数以及线性和平面性等信息，用来在多帧调试时进行筛除聚类

2. 多帧检测：
初始化激光雷达并进行轮询操作，对得到的点云数据进行120°读取（可减少读取时间），再进行相同的删除平面、下采样以及聚类操作，再以可视化展示出聚类的效果。

3. 问题：
调参异常之艰辛，经常单帧调试好了，多帧检测的时候机器人稍微换一个角度或者移动一下位置就检测不到机器人。所以这个方案就止步到了这里。