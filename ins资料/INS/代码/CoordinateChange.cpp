#include "IMU_Structs.h"

/*-----------------------------------------------
    大地坐标 -> 笛卡尔坐标 (BLH -> XYZ)
------------------------------------------------*/
void BLHToXYZ(POSITION BLH, double XYZ[3], const double R, const double F)
{
    double a_cor = R;                           // 长半轴
    double f = F;                           // 扁率
    double b_cor = a_cor * (1.0 - f);               // 短半轴
    double e2 = (a_cor * a_cor - b_cor * b_cor) / (a_cor * a_cor);  // 第一偏心率平方

    double L = BLH.longitude;   // 经度 (弧度)
    double B = BLH.latitude;    // 纬度 (弧度)
    double H = BLH.H;           // 高程 (米)

    double sinB = sin(B);
    double cosB = cos(B);
    double sinL = sin(L);
    double cosL = cos(L);

    double N = a_cor / sqrt(1.0 - e2 * sinB * sinB); // 卯酉圈半径

    XYZ[0] = (N + H) * cosB * cosL;
    XYZ[1] = (N + H) * cosB * sinL;
    XYZ[2] = (N * (1.0 - e2) + H) * sinB;
}

/*-----------------------------------------------
    笛卡尔坐标 -> 大地坐标 (XYZ -> BLH)
------------------------------------------------*/
void XYZToBLH(double XYZ[3], POSITION* BLH, const double R, const double F)
{
    double a_cor = R;
    double b_cor = a_cor * (1 - F);
    double e2 = (a_cor * a_cor - b_cor * b_cor) / (a_cor * a_cor);

    double x = XYZ[0];
    double y = XYZ[1];
    double z = XYZ[2];
    double p = sqrt(x * x + y * y);

    double B = atan2(z, p * (1 - e2)); // 初始纬度 (弧度)
    double H = 0.0;

    int MaxCount = 1000;
    double Threshold = 1e-12;

    int count = 0;
    double delta = 0.0;

    do {
        double sinB = sin(B);
        double N = a_cor / sqrt(1.0 - e2 * sinB * sinB);
        H = p / cos(B) - N;


        double B_new = atan2(z + e2 * (a_cor / sqrt(1.0 - e2 * sinB * sinB)) * sinB, p);
        delta = B - B_new;
        B = B_new;
        count++;
    } while (count < MaxCount && std::fabs(delta) > Threshold);

    if (count >= MaxCount) {
        std::cerr << "警告: 未在" << MaxCount
            << "次迭代内收敛（误差：" << std::fabs(delta)
            << " 弧度）" << std::endl;
    }

    BLH->latitude = B * Deg;           // 纬度 (度)
    BLH->longitude = atan2(y, x) * Deg; // 经度 (度)
    BLH->H = H;                 // 高程 (米)
}

/*-----------------------------------------------
    生成 BLH -> NEU 的旋转矩阵
------------------------------------------------*/
void BlhToNeuMat(POSITION* Blh, Matrix3d& Mat)
{
    double B = Blh->latitude;
    double L = Blh->longitude;

    double sinB = sin(B), cosB = cos(B);
    double sinL = sin(L), cosL = cos(L);

    Mat << -sinL, cosL, 0.0,
        -sinB * cosL, -sinB * sinL, cosB,
        cosB* cosL, cosB* sinL, sinB;
}

/*-----------------------------------------------
    计算 ENU 定位误差
------------------------------------------------*/
void CompEnudPos(const double Xs[], const double Xr[], POSITION* Blh, dENU* denu)
{
    Vector3d Delta;
    Delta << Xs[0] - Xr[0], Xs[1] - Xr[1], Xs[2] - Xr[2];

    Matrix3d R;
    BlhToNeuMat(Blh, R);  // E/N/U方向旋转矩阵

    Vector3d d_neu = R * Delta;

    denu->dE = d_neu(0); // E
    denu->dN = d_neu(1); // N
    denu->dU = d_neu(2); // U
}