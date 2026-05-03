#pragma once
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstdio>

using namespace std;
using namespace Eigen;

/*-----------------------------------------------
    常数定义
------------------------------------------------*/
/* 常数 */
#define PAI 3.141592653589      // 圆周率
#define Deg 180.0 / PAI         // 弧度转化为度
#define Rad PAI / 180.0         // 度转化为弧度

/* WGS84椭球参数 */
#define a 6378137.0             // 地球长半轴(m)
#define b 6356752.3142          // 地球短半轴(m)
#define e sqrt(a * a - b * b) / a       // 椭球第一偏心率
#define we 7.292115e-5          // 地球自转角速度(rad/s)
#define gravity 9.7936174       // 重力加速度9.7936174
#define R_WGS84 6378137.0       // 地球半长轴 [米]
#define F_WGS84 (1.0 / 298.257223563)   // 地球扁率


// 定义并初始化常量
/* 参考数据 */
const double initial_time = 91620.0;                    // 初始时间 (秒)
const double initial_latitude = 23.1373950708 * Rad;    // 北纬，单位：rad
const double initial_longitude = 113.3713651222 * Rad;  // 东经，单位：rad
const double initial_elevation = 2.175;                 // 高程，单位：米
const double initial_velocity_north = 0.0;    // 北向速度，单位：m/s
const double initial_velocity_east = 0.0;     // 东向速度，单位：m/s
const double initial_velocity_upward = 0.0;   // 垂向速度，单位：m/s
const double initial_roll = 0.0107951084511778 * Rad;   // 横滚角，单位：Rad
const double initial_pitch = -2.14251290749072 * Rad;   // 俯仰角，单位：Rad
const double initial_heading = -75.7498049314083 * Rad; // 航向角，单位：Rad

/* 我们的数据 */
const double starttime = 440532.000;    // 有真值参考的开始时间
const double endtime = 442609.990;      // 有真值参考的结束时间

const double ours_initial_week = 2390;          // 初始周
const double ours_initial_second = 440190.020;  // 初始周内秒
const double ours_initial_latitude = 30.5279685193 * Rad;     // 初始纬度
const double ours_initial_longitude = 114.3555367313 * Rad;   // 初始经度
const double ours_initial_height = 23.3659979239;             // 初始高程
const double ours_initial_Vn = 0.0; // 初始北向速度
const double ours_initial_Ve = 0.0; // 初始东向速度
const double ours_initial_Vd = 0.0; // 初始垂向速度
const double ours_initial_roll = -0.2236406320 * Rad;           // 初始横滚角
const double ours_initial_pitch = -0.0932009081 * Rad;          // 初始俯仰角
const double ours_initial_yaw = (189.9907547479 - 360.0) * Rad; // 初始航向角

const int zero_time_intervals_num = 14; // 零速修正区间个数

/* 设备类型 */
enum DeviceType { XWGI, NSC, CGI };     // XWGI为标定(calibration) NSC为对准(align) CGI为纯惯导
#define rate_CGI 100                    // CGI设备的频率
#define acc_CGI 1.0/655360.0       // CGI设备加速度计比例系数
#define gyr_CGI 1.0/160849.543863  // CGI设备陀螺仪比例系数

#define rate_XWGI 100               // XWGI设备采样率
#define acc_XWGI 1.5258789063e-6    // XWGI设备加速度计比例系数
#define gyr_XWGI 1.0850694444e-7    // XWGI设备陀螺仪比例系数


/*-----------------------------------------------
    结构体定义
------------------------------------------------*/
/* GPS时间 */
struct GPSTIME
{
    int Week;   // 周
    double Second;  // 秒
    GPSTIME() { Week = 0; Second = 0; }
};

/* 加速度计数据 */
struct ACCDAT
{
    double X, Y, Z;
    ACCDAT() { X = 0.0; Y = 0.0; Z = 0.0; }
};

/* 陀螺仪数据 */
struct GYRODAT
{
    double X, Y, Z;
    GYRODAT() { X = 0.0; Y = 0.0; Z = 0.0; }
};

/* 位置结果 */
struct POSITION
{
    double latitude;    // 纬度
    double longitude;   // 经度
    double H;           // 高程
    POSITION() { latitude = longitude = H = 0.0; }
};

/* 轨迹结果 */
struct dENU
{
    double dE;
    double dN;
    double dU;
    dENU() { dE = dN = dU = 0.0; }
};

/* 速度结果 */
struct VELOCITY
{
    double Vn, Ve, Vd;  // 北东地速度
    VELOCITY() { Vn = Ve = Vd = 0.0; }
};

/* 姿态结果 */
struct POSTURE
{
    double roll, pitch, yaw;    // 横滚、俯仰、航向角
    POSTURE() { roll = pitch = yaw = 0.0; }
};

/* 原始数据(参考数据) */
struct RAWDAT
{
    GPSTIME time;   // 周 秒
    ACCDAT acc;     // 加速度计数据
    GYRODAT gyr;    // 陀螺仪数据（弧度）
    POSITION blh;   // 参考经纬度
    VELOCITY vel;   // 参考速度
    POSTURE pos;    // 参考姿态
};

/* 四元数 */
struct Quater
{
    double q[4];    // q0 q1 q2 q3
    Quater() { for (int i = 0; i < 4; i++) { q[i] = 0.0; } }

    void Setkk_1(POSTURE pos)   // 设置k,k-1四元数 输入角速度增量pos(弧度)
    {
        double mol_pos12 = sqrt(pow(pos.roll / 2.0, 2) + pow(pos.pitch / 2.0, 2) + pow(pos.yaw / 2.0, 2));  // 姿态向量的模
        double coef = 0.0;  // 系数
        if (mol_pos12 != 0)coef = sin(mol_pos12) / mol_pos12; // 后三项的系数
        q[0] = cos(mol_pos12);
        q[1] = 0.5 * pos.roll * coef;
        q[2] = 0.5 * pos.pitch * coef;
        q[3] = 0.5 * pos.yaw * coef;        
    }

    void Setk_1k(POSTURE pos)   // 设置k-1,k四元数 输入角速度增量pos(弧度)
    {
        double mol_pos12 = sqrt(pow(pos.roll / 2.0, 2) + pow(pos.pitch / 2.0, 2) + pow(pos.yaw / 2.0, 2));  // 姿态向量的模
        double coef = sin(mol_pos12) / mol_pos12;   // 后三项的系数
        q[0] = cos(mol_pos12);
        q[1] = -0.5 * pos.roll * coef;
        q[2] = -0.5 * pos.pitch * coef;
        q[3] = -0.5 * pos.yaw * coef;        
    }

    void SetQbn(POSTURE pos)    // 设置初始姿态四元数
    {
        double fai = pos.roll / 2.0, theta = pos.pitch / 2.0, pusai = pos.yaw / 2.0;
        q[0] = cos(fai) * cos(theta) * cos(pusai) + sin(fai) * sin(theta) * sin(pusai);
        q[1] = sin(fai) * cos(theta) * cos(pusai) - cos(fai) * sin(theta) * sin(pusai);
        q[2] = cos(fai) * sin(theta) * cos(pusai) + sin(fai) * cos(theta) * sin(pusai);
        q[3] = cos(fai) * cos(theta) * sin(pusai) - sin(fai) * sin(theta) * cos(pusai);
    }
};

/* 余弦矩阵 */
struct CosineMatrix
{
    double row1[3]; // 第一行的三个数据
    double row2[3];
    double row3[3];
    Matrix3d Cbn = Matrix3d::Zero(3, 3);

    CosineMatrix() { for (int i = 0; i < 3; i++) { row1[i] = row2[i] = row3[i] = 0.0; } }

    void SetCbn(POSTURE pos)    // 设置Cbn余弦矩阵
    {
        double theta = pos.pitch, pusai = pos.yaw, fai = pos.roll;  // 俯仰角、航向角、横滚角(rad)
        row1[0] = cos(theta) * cos(pusai);
        row1[1] = -cos(fai) * sin(pusai) + sin(fai) * sin(theta) * cos(pusai);
        row1[2] = sin(fai) * sin(pusai) + cos(fai) * sin(theta) * cos(pusai);
        row2[0] = cos(theta) * sin(pusai);
        row2[1] = cos(fai) * cos(pusai) + sin(fai) * sin(theta) * sin(pusai);
        row2[2] = -sin(fai) * cos(pusai) + cos(fai) * sin(theta) * sin(pusai);
        row3[0] = -sin(theta);
        row3[1] = sin(fai) * cos(theta);
        row3[2] = cos(fai) * cos(theta);

        for (int i = 0; i < 3; i++)
        {
            Cbn(0, i) = row1[i];
            Cbn(1, i) = row2[i];
            Cbn(2, i) = row3[i];
        }
    }
};

/* 纯惯导数据结构体（每个历元的数据） */
struct IMUDataEpoch 
{   // double 类型，占 8 字节
    double TimeStamp;   // 时间戳 s
    GYRODAT Gyr;        // 角速度的增量 rad
    ACCDAT Acc;         // 加速度的增量 m/s 
    IMUDataEpoch() { TimeStamp = 0.0; }
};

/* 纯惯导参照结果结构体（每个历元的数据） */
struct INSDataEpoch 
{   // double 类型，占 8 字节
    double TimeStamp;   // 时间戳 s
    POSITION blh;       // 位置 deg
    VELOCITY vel;       // 速度 m/s
    POSTURE pos;        // 姿态 deg
    INSDataEpoch() { TimeStamp = 0.0; }
};

/* 零速修正区间 */
struct TimeInterval
{
    double start;   // 开始静止时刻
    double end;     // 开始启动时刻
    bool used;      // 是否使用过

    TimeInterval(double starttime = 0.0, double endtime = 0.0, bool isUsed = false)
        :start(starttime),end(endtime),used(isUsed){ }
};

/* 零速修正的所有区间 */
struct TimeIntervalsArray
{
    // 总共十四个需要零速修正的区间
    TimeInterval interval[zero_time_intervals_num]; // interval[14]

    // 初始化时间区间
    TimeIntervalsArray()
    {
        interval[0] = TimeInterval(440664, 440708, false);
        interval[1] = TimeInterval(440762, 440838, false);
        interval[2] = TimeInterval(440883, 440915, false);
        interval[3] = TimeInterval(440974, 441044, false);
        interval[4] = TimeInterval(441116, 441171, false);
        interval[5] = TimeInterval(441225, 441271, false);
        interval[6] = TimeInterval(441361, 441428, false);
        interval[7] = TimeInterval(441538, 441597, false);
        interval[8] = TimeInterval(441656, 441680, false);
        interval[9] = TimeInterval(441758, 441796, false);
        interval[10] = TimeInterval(441894, 441926, false);
        interval[11] = TimeInterval(442020, 442063, false);
        interval[12] = TimeInterval(442144, 442233, false);
        interval[13] = TimeInterval(442319, 442609.990, false);
    }

    // 获取指定索引的时间区间
    TimeInterval& getInterval(int index)
    {
        if (index >= 0 && index < 14)return interval[index];
        else {
            cout << "Index out of our time interval bounds!\n";
        }
    }
};

/*-----------------------------------------------
    文件操作函数
------------------------------------------------*/
bool ReadExamplePureIMUData(FILE* file, IMUDataEpoch* mImuDataEpoch);       // 读取一历元纯惯导示例原始数据
bool ReadExamplePureINSData(FILE* file, INSDataEpoch* mInsResultEpoch);     // 读取一历元纯惯导示例参照结果数据
void SaveExampleData(FILE* OutputFile, IMUDataEpoch mImu, INSDataEpoch mIns, int mode);         // 保存一行原始/参考数据

bool ReadIMURawData_CGI(ifstream& file, IMUDataEpoch* Rawdata, DeviceType device);  // 读取一行纯惯导的有效原始数据
bool ReadIMURawData_Else(ifstream& file, IMUDataEpoch* Rawdata, DeviceType device);       // 读取一行纯惯导的有效原始数据
bool ReadTruthData(ifstream& file, INSDataEpoch* Rawdata);  // 读取一行真值
void SaveOurResult(FILE* OutputFile, IMUDataEpoch mImu, VELOCITY vel, POSITION blh, POSTURE pos);   // 保存一行示例文件解算结果
void SaveTrueResult(FILE* OutputFile, INSDataEpoch mIns);  // 保存一行参考真值结果
void SaveDiffResult(FILE* OutputFile, IMUDataEpoch mImu, double dvel[3], double dblh[3], double dpos[3]);   // 保存一行计算结果与真值之间的差分结果
void SavedENUResult(FILE* OutputFile, IMUDataEpoch mImu, dENU truedata, dENU ourdata);  // 保存denu结果

/*-----------------------------------------------
    矩阵打印函数
------------------------------------------------*/
void Matrixprint(const MatrixXd Mat, const string name);    // 输入矩阵 + 矩阵名
void Vectorprint(const VectorXd Mat, const string name);    // 输入向量 + 向量名

/*-----------------------------------------------
    标定加速度计、陀螺仪零偏函数
------------------------------------------------*/
void CalAvgAcc_Gyr(IMUDataEpoch rawdata, double* epochnum, double* accmean, double* gyrmean);   // 计算平均值
void AccCalibration(double accmean[3], IMUDataEpoch* rawdata);  // 标定加速度计零偏
void GyrCalibration(double gyrmean[3], IMUDataEpoch* rawdata);  // 标定陀螺仪零偏

/*-----------------------------------------------
    四元数、矩阵、常数运算函数
------------------------------------------------*/
void QuaternionMultiply(Quater p, Quater q, Quater* result);    // 四元数相乘p o q
void CalPostureWithQuaternion(Quater Q, INSDataEpoch* result);  // 四元数计算姿态
void SkewSymmetricMatrix(Vector3d vec, Matrix3d* result);       // 反对称矩阵
void Calgpn(POSITION blh, Vector3d* gpn);                       // 计算正常重力
void Extrapolation(double tk, double tk_1, double tk_2, 
    VELOCITY vel_prv, VELOCITY vel_pprv, 
    POSITION blh_prv, POSITION blh_pprv, 
    Vector3d* w_ie_n, Vector3d* w_en_n, Vector3d* v_k_12);      // 外推计算wien_k-1/2 wenn_k-1/2 vn_k-1/2

/*-----------------------------------------------
    惯性导航算法函数
------------------------------------------------*/
bool PostureUpdate(IMUDataEpoch pos_cur, IMUDataEpoch pos_prv, INSDataEpoch result_prv, Quater result_k_1, Quater* result_k, INSDataEpoch* result_cur);                 // 姿态更新
bool VelocityUpdate(IMUDataEpoch pos_cur, IMUDataEpoch pos_prv, IMUDataEpoch pos_pprv, INSDataEpoch result_prv, INSDataEpoch result_pprv, INSDataEpoch* result_cur);    // 速度更新
bool PositionUpdate(IMUDataEpoch pos_cur, IMUDataEpoch pos_prv, IMUDataEpoch pos_pprv, INSDataEpoch result_prv, INSDataEpoch result_pprv, INSDataEpoch* result_cur);    // 位置更新

/*-----------------------------------------------
    坐标变换函数
------------------------------------------------*/
void BLHToXYZ(POSITION BLH, double XYZ[3], const double R, const double F);  // BLH2XYZ
void XYZToBLH(double XYZ[3], POSITION* BLH, const double R, const double F);  // XYZ2BLH
void BlhToNeuMat(POSITION* Blh, Matrix3d& Mat);
void CompEnudPos(const double Xs[], const double Xr[], POSITION* Blh, dENU* dENU);  // 计算enu