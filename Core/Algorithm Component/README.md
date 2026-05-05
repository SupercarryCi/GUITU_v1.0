/**************喵喵工具

pedestrian_frame_transform  
模块负责：
 - 接收机体坐标系加速度与姿态
 - 构建 body->pedestrian 的方向余弦矩阵 DCM
 - 将加速度旋转到行人坐标系
 - 按需在行人坐标系 Z 轴去重力

pedestrian_inertial_nav
模块负责：
 - 接收“已经变换到行人坐标系、并且已经去重力”的线加速度，然后积分得到：
 - 行人坐标系下的速度 `velocity_ped_mps`
 - 行人坐标系下的局部位移 `position_ped_m`