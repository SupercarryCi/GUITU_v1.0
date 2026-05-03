#include"IMU_Structs.h"

/* 计算加速度计与陀螺仪的平均输出 */
// 输入：原始数据rawdata
// 输入：初值为0的epochnum
// 输出：平均比力
// 输出：平均角速度
void CalAvgAcc_Gyr(IMUDataEpoch rawdata, double* epochnum, double* accmean, double* gyrmean)
{
	// 轮数+1
	(*epochnum)++;

	// 累加比力
	accmean[0] = accmean[0] * (*epochnum - 1) + rawdata.Acc.X;
	accmean[1] = accmean[1] * (*epochnum - 1) + rawdata.Acc.Y;
	accmean[2] = accmean[2] * (*epochnum - 1) + rawdata.Acc.Z;

	// 累加角速度
	gyrmean[0] = gyrmean[0] * (*epochnum - 1) + rawdata.Gyr.X;
	gyrmean[1] = gyrmean[1] * (*epochnum - 1) + rawdata.Gyr.Y;
	gyrmean[2] = gyrmean[2] * (*epochnum - 1) + rawdata.Gyr.Z;

	// 计算平均值
	for (int i = 0; i < 3; i++)
	{
		accmean[i] /= (*epochnum);
		gyrmean[i] /= (*epochnum);
	}
}

/* 加速度计零偏标定 */
// 输入：平均比力
// 输入：原始数据
// 输出：补偿后的数据rawdata
void AccCalibration(double accmean[3], IMUDataEpoch* rawdata)
{
	double biasAcc[3] = { 0.0,0.0,0.0 };
	biasAcc[0] = accmean[0];
	biasAcc[1] = accmean[1];
	biasAcc[2] = accmean[2] + gravity / rate_CGI;

	rawdata->Acc.X -= biasAcc[0];
	rawdata->Acc.Y -= biasAcc[1];
	rawdata->Acc.Z -= biasAcc[2];
}

/* 陀螺仪零偏标定 */
// 输入：平均角速度
// 输入：原始数据
// 输出：补偿后的数据rawdata
void GyrCalibration(double gyrmean[3], IMUDataEpoch* rawdata)
{
	double i1 = we * cos(ours_initial_latitude) * cos(ours_initial_yaw) / 100.0;
	double i2 = we * cos(ours_initial_latitude) * sin(ours_initial_yaw) / 100.0;
	double i3 = we * sin(ours_initial_latitude) / 100.0;
	double biasGyr[3] = { 0.0,0.0,0.0 };
	biasGyr[0] = gyrmean[0];
	biasGyr[1] = gyrmean[1];
	biasGyr[2] = gyrmean[2];

	biasGyr[0] -= i1;
	biasGyr[1] -= i2;
	biasGyr[2] -= i3;
	rawdata->Gyr.X -= biasGyr[0];
	rawdata->Gyr.Y -= biasGyr[1];
	rawdata->Gyr.Z -= biasGyr[2];
}