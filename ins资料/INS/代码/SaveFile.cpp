#include"IMU_Structs.h"

/* 保存示例数据txt格式结果到文件中 */
// 输入：mode为0代表保存原始数据 mode为1代表保存参考数据
void SaveExampleData(FILE* OutputFile, IMUDataEpoch mImu, INSDataEpoch mIns, int mode)
{
	if (mode == 0) fprintf(OutputFile, "%0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f\n", mImu.TimeStamp,
		mImu.Acc.X, mImu.Acc.Y, mImu.Acc.Z,
		mImu.Gyr.X, mImu.Gyr.Y, mImu.Gyr.Z);
	if (mode == 1) fprintf(OutputFile, "%0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f\n", mIns.TimeStamp,
		mIns.blh.latitude, mIns.blh.longitude, mIns.blh.H,
		mIns.vel.Vn, mIns.vel.Ve, mIns.vel.Vd,
		mIns.pos.roll, mIns.pos.pitch, mIns.pos.yaw);
}

/* 保存示例文件结算结果 */
void SaveOurResult(FILE* OutputFile, IMUDataEpoch mImu, VELOCITY vel, POSITION blh, POSTURE pos)
{
	double Yaw = pos.yaw * Deg;
	if (Yaw < 0.0)Yaw += 360;

	fprintf(OutputFile, "%0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f\n", mImu.TimeStamp,
		blh.latitude * Deg, blh.longitude * Deg, blh.H,
		vel.Vn, vel.Ve, vel.Vd,
		pos.roll * Deg, pos.pitch * Deg, Yaw);
}

/* 保存参考真值结果 */
void SaveTrueResult(FILE* OutputFile, INSDataEpoch mIns)
{
	fprintf(OutputFile, "%0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f\n", mIns.TimeStamp,
		mIns.blh.latitude, mIns.blh.longitude, mIns.blh.H,
		mIns.vel.Vn, mIns.vel.Ve, mIns.vel.Vd,
		mIns.pos.roll, mIns.pos.pitch, mIns.pos.yaw);
}

/* 保存差分结果 */
void SaveDiffResult(FILE* OutputFile, IMUDataEpoch mImu, double dvel[3], double dblh[3], double dpos[3])
{
	fprintf(OutputFile, "%0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f\n", mImu.TimeStamp,
		dvel[0], dvel[1], dvel[2],
		dblh[0], dblh[1], dblh[2],
		dpos[0], dpos[1], dpos[2]);
}

/* 保存dnue轨迹结果 */
void SavedENUResult(FILE* OutputFile, IMUDataEpoch mImu, dENU truedata, dENU ourdata)
{
	fprintf(OutputFile, "%0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f\n", mImu.TimeStamp,
		truedata.dN, truedata.dE, truedata.dU,
		ourdata.dN, ourdata.dE, ourdata.dU);
}