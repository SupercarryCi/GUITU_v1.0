#include"IMU_Structs.h"

/**************************************/
/*				示例数据			  */
/**************************************/
/* 读取一历元纯惯导示例数据 */
// 输入：二进制打开的文件 示例：FILE file = fopen(filePath, "rb");
// 输出：一历元数据
bool ReadExamplePureIMUData(FILE* file, IMUDataEpoch* mImuDataEpoch)
{
	if (!file) { cout << "Can't open the Example Data!" << endl; return false; }	// 打不开文件

	// 读一个历元的数据
	size_t readSize = fread(mImuDataEpoch, 8, 7, file);  // 一次读取7个8字节的数据项
	if (readSize != 7) { cout << "Can't read enough IMU data!" << endl; return false; }	// 没读到7个数据
	return true;
}

/* 读取一历元纯惯导参照结果数据 */
// 输入：二进制打开的文件 示例：FILE file = fopen(filePath, "rb");
// 输出：一历元数据
bool ReadExamplePureINSData(FILE* file, INSDataEpoch* mInsResultEpoch)
{
	if (!file) { cout << "Can't open the Example Data!" << endl; return false; }	// 打不开文件

	// 读一个历元的数据
	size_t readSize = fread(mInsResultEpoch, 8, 10, file);  // 一次读取10个8字节的数据项
	if (readSize != 10) { cout << "Can't read enough INS data!" << endl; return false; }	// 没读到10个数据
	return true;
}


/* 读取惯性导航的数据 */
// 输入：文件流flie (ifstream)
// 输入：设备信息（DeviceType）
// 输出：原始数据Rawdata (IMUDataEpoch) 例：IMUDataEpoch rawdata
bool ReadIMURawData_CGI(ifstream& file, IMUDataEpoch* Rawdata, DeviceType device)
{
	// 采样率、加速度计、陀螺数据转换比例因子
	double rate = 0.0, convert_a = 0.0, convert_g = 0.0;
	if (device == CGI) { rate = 1; convert_a = acc_CGI; convert_g = gyr_CGI; }	// 纯惯导 PureIN
	else if (device == XWGI) { rate = 1; convert_a = acc_XWGI; convert_g = gyr_XWGI; }	// 纯惯导 PureIN
	else return false;

	// 读取文件数据
	string line;
	while (getline(file, line))
	{
		// 行为空则继续循环
		if (line.empty())return false;

		// 获得第一个","之前的内容
		stringstream ss(line);
		string part;
		getline(ss, part, ',');

		// 如果第一个","之前的内容不是"#RAWIMUA"的话，则跳过
		if (part != "#RAWIMUA")continue;

		getline(ss, part, ',');	// 跳过COM 1
		getline(ss, part, ',');	// 跳过0
		getline(ss, part, ',');	// 跳过113.0
		getline(ss, part, ',');	// 跳过FINE

		// 读取周、周内秒
		getline(ss, part, ',');	// 周
		getline(ss, part, ',');	// 周内秒
		Rawdata->TimeStamp = stod(part);

		getline(ss, part, ';');	// 跳过21730,14102
		getline(ss, part, ',');	// 跳过第二个周
		getline(ss, part, ',');	// 跳过第二个周内秒
		getline(ss, part, ',');	// 跳过00007777

		/* 轴系调整 */
		// 读取加速度计数据
		getline(ss, part, ',');	// -Z
		Rawdata->Acc.Z = - stod(part) * convert_a * rate;
		getline(ss, part, ',');	// -X
		Rawdata->Acc.X = -stod(part) * convert_a * rate;
		getline(ss, part, ',');	// Y
		Rawdata->Acc.Y = stod(part) * convert_a * rate;

		// 读取陀螺数据
		getline(ss, part, ',');	// -Z
		Rawdata->Gyr.Z = -stod(part) * convert_g * rate * Rad;
		getline(ss, part, ',');	// -X
		Rawdata->Gyr.X = -stod(part) * convert_g * rate * Rad;
		getline(ss, part, ',');	// Y
		Rawdata->Gyr.Y = stod(part) * convert_g * rate * Rad;

		return true;	// 读取成功
	}
	return false;	// 读取一行失败
}

/* 读取一行标定的有效原始数据 */
// 输入：文件流flie (ifstream) 例：ifstream File("path.txt")
// 输出：原始数据Rawdata (RAWDAT) 例：RAWDAT rawdata
bool ReadIMURawData_Else(ifstream& file, IMUDataEpoch* Rawdata, DeviceType device)
{
	// 采样率，加速度计、陀螺仪数据转换比例因子
	double rate = 0.0, convert_a = 0.0, convert_g = 0.0;
	if (device == XWGI) { rate = 1; convert_a = acc_XWGI; convert_g = gyr_XWGI; }	// 标定 Calibration
	else return false;

	// 读取文件数据
	string line;
	if (getline(file, line))
	{
		if (line.empty())return false;	// 行为空时继续循环

		stringstream ss(line);	// 使用字符串流 逐个字段解析

		string part;
		char delimiter;

		// 跳过%RAWIMUSA,2387,558507.850;
		getline(ss, part, ';');

		// 读取周 周内秒
		getline(ss, part, ',');	// 周
		getline(ss, part, ',');	// 周内秒
		Rawdata->TimeStamp = stod(part);

		// 跳过00000077,
		getline(ss, part, ',');

		// 读取加速度计数据
		getline(ss, part, ',');	// -Z
		Rawdata->Acc.Z = -stod(part) * convert_a * rate;
		getline(ss, part, ',');	// X
		Rawdata->Acc.X = stod(part) * convert_a * rate;
		getline(ss, part, ',');	// Y
		Rawdata->Acc.Y = stod(part) * convert_a * rate;

		// 读取陀螺仪数据
		getline(ss, part, ',');	// Z(弧度)
		Rawdata->Gyr.Z = -stod(part) * convert_g * rate;
		getline(ss, part, ',');	// X(弧度)
		Rawdata->Gyr.X = stod(part) * convert_g * rate;
		getline(ss, part, '*');	// Y(弧度)
		Rawdata->Gyr.Y = stod(part) * convert_g * rate;

		return true;	// 读取一行成功
	}
	return false;	// 读取一行失败则返回false
}

/* 读取参考真值的数据 */
// 输入：文件流flie (ifstream)
// 输出：参考真值数据Rawdata (INSDataEpoch) 例：INSDataEpoch rawdata
bool ReadTruthData(ifstream& file, INSDataEpoch* Rawdata)
{
	string line;
	while (getline(file, line)) {
		// 如果行为空，则跳过
		if (line.empty()) {
			continue;
		}

		stringstream ss(line);
		string part;

		// 通过空格分割每个部分
		vector<string> parts;
		while (ss >> part) {
			parts.push_back(part);
		}

		// 现在 parts 中每个元素代表了行中的一个数据项

		// 读取时间戳（周内秒）
		Rawdata->TimeStamp = stod(parts[1]);

		// 读取位置数据（纬度、经度、高程）
		Rawdata->blh.latitude = stod(parts[2]);
		Rawdata->blh.longitude = stod(parts[3]);
		Rawdata->blh.H = stod(parts[4]);

		// 读取速度数据（Vn, Ve, Vd）
		Rawdata->vel.Vn = stod(parts[5]);
		Rawdata->vel.Ve = stod(parts[6]);
		Rawdata->vel.Vd = stod(parts[7]);

		// 读取姿态角数据（roll, pitch, yaw）
		Rawdata->pos.roll = stod(parts[8]);
		Rawdata->pos.pitch = stod(parts[9]);
		Rawdata->pos.yaw = stod(parts[10]);

		//// 将航向角转化为-180 到 180
		//if (Rawdata->pos.yaw > 180.0) {
		//	Rawdata->pos.yaw -= 360;
		//}

		return true; // 读取成功
	}

	return false; // 文件结束或读取失败
}