/**************喵喵工具

pedestrian_frame_transform  
模块负责：
 - 接收机体坐标系加速度与姿态
 - 构建 body->pedestrian 的方向余弦矩阵 DCM
 - 将加速度旋转到行人坐标系
 - 按需在行人坐标系 Z 轴去重力