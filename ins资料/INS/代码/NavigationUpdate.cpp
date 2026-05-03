#include"IMU_Structs.h"

/* 姿态更新 */
// 输入：当前历元原始数据pos_cur
// 输入：上个历元原始数据pos_prv
// 输入：上个历元位置blh
// 输入：上个历元速度vel
// 输入：上个历元姿态四元数result_k_1
// 输出：当前历元姿态四元数result_k
// 输出：当前历元姿态
bool PostureUpdate(IMUDataEpoch pos_cur, IMUDataEpoch pos_prv, INSDataEpoch result_prv, Quater result_k_1, Quater* result_k, INSDataEpoch* result_cur)
{
	// 1. 等效旋转矢量法更新b系
	Vector3d q_cur(pos_cur.Gyr.X, pos_cur.Gyr.Y, pos_cur.Gyr.Z);	// 角速度增量(rad)
	Vector3d q_prv(pos_prv.Gyr.X, pos_prv.Gyr.Y, pos_prv.Gyr.Z);

	Vector3d ERV_b = q_cur + q_prv.cross(q_cur) / 12.0;	// b系等效旋转矢量（Vector）(rad)
	POSTURE EqualRotaionVec_b;	// b系等效旋转矢量(rad)
	EqualRotaionVec_b.roll = ERV_b[0];
	EqualRotaionVec_b.pitch = ERV_b[1]; 
	EqualRotaionVec_b.yaw = ERV_b[2];

	Quater qbkk_1;	// b系k,k-1四元数(rad)
	qbkk_1.Setkk_1(EqualRotaionVec_b);

	// 2. 等效旋转矢量法更新n系
	double deltaT = pos_cur.TimeStamp - pos_prv.TimeStamp;	// 时间差
	Vector3d w_ie_n(we * cos(result_prv.blh.latitude), 0, -we * sin(result_prv.blh.latitude));	// 列向量(rad)
	
	double R_M = a * (1 - pow(e, 2)) / sqrt(pow(1 - pow(e * sin(result_prv.blh.latitude), 2), 3));
	double R_N = a / sqrt(1 - pow(e * sin(result_prv.blh.latitude), 2));
	Vector3d w_en_n(result_prv.vel.Ve / (R_N + result_prv.blh.H), -result_prv.vel.Vn / (R_M + result_prv.blh.H), -result_prv.vel.Ve * tan(result_prv.blh.latitude) / (R_N + result_prv.blh.H));

	Vector3d ERV_n = (w_ie_n + w_en_n) * deltaT;
	POSTURE EqualRotationVec_n;	// n系等效旋转矢量
	EqualRotationVec_n.roll = ERV_n[0];
	EqualRotationVec_n.pitch = ERV_n[1];
	EqualRotationVec_n.yaw = ERV_n[2];

	Quater qnk_1k;	// n系k-1,k四元数(rad)
	qnk_1k.Setk_1k(EqualRotationVec_n);

	// 3. 计算当前姿态四元数
	Quater middle;
	QuaternionMultiply(qnk_1k, result_k_1, &middle);
	QuaternionMultiply(middle, qbkk_1, result_k);

	// 4. 归一化姿态四元数
	double Rou = sqrt(pow(result_k->q[0], 2) + pow(result_k->q[1], 2) + pow(result_k->q[2], 2) + pow(result_k->q[3], 2));
	for (int i = 0; i < 4; i++) { result_k->q[i] = result_k->q[i] / Rou; }

	// 4. 计算当前姿态
	CalPostureWithQuaternion(*result_k, result_cur);

	return true;
}

/* 速度更新 */
// 输入：当前、上个、上上个原始数据
// 输入：上个、上上个速度
// 输入：上个、上上个位置
// 输入：上个姿态
// 输出：当前速度
bool VelocityUpdate(IMUDataEpoch pos_cur, IMUDataEpoch pos_prv, IMUDataEpoch pos_pprv, INSDataEpoch result_prv, INSDataEpoch result_pprv, INSDataEpoch* result_cur)
{
	// 1. 计算vfk_bk_1 b系下速度增量
	Vector3d vk(pos_cur.Acc.X, pos_cur.Acc.Y, pos_cur.Acc.Z);	// 当前速度增量(m/s)
	Vector3d vk_1(pos_prv.Acc.X, pos_prv.Acc.Y, pos_prv.Acc.Z);	// 上个历元速度增量(m/s)

	Vector3d thetak(pos_cur.Gyr.X, pos_cur.Gyr.Y, pos_cur.Gyr.Z);	// 当前角度增量(rad)
	Vector3d thetak_1(pos_prv.Gyr.X, pos_prv.Gyr.Y, pos_prv.Gyr.Z);	// 上个历元角度增量(rad)

	Vector3d vfk_bk_1 = vk + 0.5 * thetak.cross(vk) + (thetak_1.cross(vk) + vk_1.cross(thetak)) / 12.0;	// b系下速度增量(m/s)

	// 2. 计算zetak_1k n系下等效旋转矢量(k-2,k-1合理外推)
	double deltaT = pos_cur.TimeStamp - pos_prv.TimeStamp;
	Vector3d w_ie_n = Vector3d::Zero(), w_en_n = Vector3d::Zero(), v_k_12 = Vector3d::Zero();

	Extrapolation(pos_cur.TimeStamp, pos_prv.TimeStamp, pos_pprv.TimeStamp, result_prv.vel, result_pprv.vel, result_prv.blh, result_pprv.blh, &w_ie_n, &w_en_n, &v_k_12);
	Vector3d zetak_1k = (w_ie_n + w_en_n) * deltaT;	// n系下等效旋转矢量

	// 3. 计算vfkn 比力积分项
	CosineMatrix Cbn_k_1;
	Cbn_k_1.SetCbn(result_prv.pos);	// 设置k-1时刻余弦矩阵Cbn k-1

	Matrix3d Skew_zetak_1k = Matrix3d::Zero(3, 3);
	SkewSymmetricMatrix(zetak_1k, &Skew_zetak_1k);	// 求n系下等效旋转矢量的反对称阵

	Matrix3d I = Matrix3d::Identity();	// 单位阵
	Vector3d vfkn = (I - 0.5 * Skew_zetak_1k) * Cbn_k_1.Cbn * vfk_bk_1;

	// 4. 计算vgcor_kn 重力/哥式积分项（外推计算）
	Vector3d gpn_k_1 = Vector3d::Zero(), gpn_k_2 = Vector3d::Zero(), gpn_k_12 = Vector3d::Zero();
	Calgpn(result_prv.blh, &gpn_k_1);	// k-1正常重力
	Calgpn(result_pprv.blh, &gpn_k_2);	// k-2正常重力
	gpn_k_12 = gpn_k_1 + deltaT / 2.0 * (gpn_k_1 - gpn_k_2) / (pos_prv.TimeStamp - pos_pprv.TimeStamp);	// 外推的正常重力

	Vector3d vgcor_kn = (gpn_k_12 - (2 * w_ie_n + w_en_n).cross(v_k_12)) * deltaT;

	// 5. 计算当前速度vkn
	Vector3d vkn_1(result_prv.vel.Vn, result_prv.vel.Ve, result_prv.vel.Vd);	// 前一时刻速度
	Vector3d vkn = vkn_1 + vfkn + vgcor_kn;

	// 赋值
	result_cur->vel.Vn = vkn[0];
	result_cur->vel.Ve = vkn[1];
	result_cur->vel.Vd = vkn[2];

	return true;
}

/* 位置更新 */
// 输入：当前、上个、上上个原始数据
// 输入：当前、上个速度
// 输入：上个、上上个位置
// 输出：当前位置
bool PositionUpdate(IMUDataEpoch pos_cur, IMUDataEpoch pos_prv, IMUDataEpoch pos_pprv, INSDataEpoch result_prv, INSDataEpoch result_pprv, INSDataEpoch* result_cur)
{
	// 1. 计算高程
	double deltaT = pos_cur.TimeStamp - pos_prv.TimeStamp;	// 时间差
	result_cur->blh.H = result_prv.blh.H - (result_cur->vel.Vd + result_prv.vel.Vd) / 2.0 * deltaT;

	// 2. 计算纬度
	double R_M_k_1 = a * (1 - pow(e, 2)) / sqrt(pow(1 - pow(e * sin(result_prv.blh.latitude), 2), 3));
	double h_ba = (result_cur->blh.H + result_prv.blh.H) / 2.0;

	result_cur->blh.latitude = result_prv.blh.latitude + (result_cur->vel.Vn + result_prv.vel.Vn) / (2.0 * (R_M_k_1 + h_ba)) * deltaT;

	// 3. 计算经度
	double R_N_k_1 = a / sqrt(1 - pow(e * sin(result_prv.blh.latitude), 2));
	double R_N_k_2 = a / sqrt(1 - pow(e * sin(result_pprv.blh.latitude), 2));
	double R_N12 = R_N_k_1 + deltaT / 2.0 * (R_N_k_1 - R_N_k_2) / (pos_prv.TimeStamp - pos_pprv.TimeStamp);
	double fai_ba = (result_cur->blh.latitude + result_prv.blh.latitude) / 2.0;

	result_cur->blh.longitude = result_prv.blh.longitude + (result_cur->vel.Ve + result_prv.vel.Ve) / (2.0 * (R_N12 + h_ba) * cos(fai_ba)) * deltaT;

	return true;
}

