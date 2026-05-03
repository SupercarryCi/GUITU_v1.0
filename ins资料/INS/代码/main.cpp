#include"IMU_Structs.h"

int main()
{
    int mode = 0;   // Solution mode: 0 for example data, 1 for self-collected data
    cout << "Please choose the mode of solution:\n  0 ExampleData\n  1 SelfData\n";
    cin >> mode;
    if (!(mode == 0 || mode == 1)) { cout << "Error type of the solution!\n"; return 1; }

    switch (mode)
    {
    case 0:
    {
        cout << "\nNow is the solution of Example! Calculating...\n";

        // Open example input and output files
        FILE* m_imu = fopen("Data/Example_Data/IMU.bin", "rb");                 // Example IMU raw data
        FILE* m_ins = fopen("Data/Example_Data/PureIns.bin", "rb");             // Example reference truth data

        FILE* result_example = fopen("Result/ExampleResult/INS_Result.txt", "wb");  // Save navigation result
        FILE* true_example = fopen("Result/ExampleResult/INS_True.txt", "wb");      // Save reference truth
        FILE* diff_result_example = fopen("Result/ExampleResult/INS_Diff_Result.txt", "wb");    // Save error result
        FILE* denu_result = fopen("Result/ExampleResult/denu_Result.txt", "wb");    // Save dENU track result

        if (!(m_imu)) { cerr << "Cannot open the file to read rawdata!\n"; return -1; }
        if (!(m_ins)) { cerr << "Cannot open the file to read rawdata!\n"; return -1; }
        if (!(result_example)) { cerr << "Cannot open the file to save result!\n"; return -1; }
        if (!(true_example)) { cerr << "Cannot open the file to save result!\n"; return -1; }
        if (!(diff_result_example)) { cerr << "Cannot open the file to save diff result!\n"; return -1; }
        if (!(denu_result)) { cerr << "Cannot open the file to save denu result!\n"; return -1; }
        fprintf(result_example, "# TimeStamp,BLH(deg),Vel_xyz(m/s),Pos_roll_pitch_yaw(deg)\n");    // Result file header
        fprintf(true_example, "# TimeStamp,BLH(deg),Vel_xyz(m/s),Pos_roll_pitch_yaw(deg)\n");      // Truth file header
        fprintf(diff_result_example, "# TimeStamp,dVel(m/s),dBlh(deg),dPos_roll_pitch_yaw(deg)\n");    // Diff file header
        fprintf(denu_result, "# TimeStamp,truedata dneu(m),ourdata dneu(m)\n");    // dENU file header

        // Cached states from previous epochs
        IMUDataEpoch mImuDataEpoch_prv, mImuDataEpoch_pprv; // Previous and pre-previous IMU epochs
        INSDataEpoch result_prv;    // Previous navigation result
        INSDataEpoch result_pprv;   // Pre-previous navigation result
        Quater Q_prv;   // Previous attitude quaternion

        POSITION ZeroPoint; // Reference origin for dENU computation
        double basexyz[3] = { 0.0,0.0,0.0 };
        ZeroPoint.latitude = initial_latitude; ZeroPoint.longitude = initial_longitude; ZeroPoint.H = initial_elevation;
        BLHToXYZ(ZeroPoint, basexyz, R_WGS84, F_WGS84);

        bool flag_imu = true;       // IMU read loop flag
        bool Is_prv = false;        // Whether previous epoch is ready
        bool Is_pprv = false;       // Whether pre-previous epoch is ready

        double caltime = 0.0;   // Accumulated processed time
        int calnum = 0;         // Number of processed 10-minute blocks

        // Epoch-by-epoch solution
        while (flag_imu == true)
        {
            // Current epoch variables
            IMUDataEpoch mImuDataEpoch_cur; // Current IMU epoch
            INSDataEpoch mInsDataEpoch_cur; // Current truth epoch
            INSDataEpoch result_cur;        // Current navigation result
            Quater Q_cur;                   // Current quaternion

            // Read current IMU epoch
            if (flag_imu) flag_imu = ReadExamplePureIMUData(m_imu, &mImuDataEpoch_cur);
            if (mImuDataEpoch_cur.TimeStamp < 91619.995)continue;   // Skip data before the alignment point

            if (!Is_prv)    // Cache the first previous epoch
            {
                mImuDataEpoch_prv = mImuDataEpoch_cur;
                Is_prv = true;
                continue;
            }
            else if (!Is_pprv)   // Cache the second previous epoch and initialize state
            {
                mImuDataEpoch_pprv = mImuDataEpoch_prv;
                mImuDataEpoch_prv = mImuDataEpoch_cur;
                Is_pprv = true;

                // Initialize navigation state
                // 1. Velocity is initialized to zero
                // 2. Position initialization
                result_prv.blh.latitude = initial_latitude; result_prv.blh.longitude = initial_longitude; result_prv.blh.H = initial_elevation;
                result_pprv.blh.latitude = initial_latitude; result_pprv.blh.longitude = initial_longitude; result_pprv.blh.H = initial_elevation;
                // 3. Attitude initialization
                result_prv.pos.roll = initial_roll;  result_prv.pos.pitch = initial_pitch;  result_prv.pos.yaw = initial_heading;
                Q_prv.SetQbn(result_prv.pos);  // Build quaternion from the initial attitude
                continue;
            }

            caltime += (mImuDataEpoch_cur.TimeStamp - mImuDataEpoch_prv.TimeStamp); // Accumulate processed duration
            if (caltime > 600.0)    // Print once every 10 minutes of data
            {
                caltime = 0.0;
                calnum += 1;

                printf("[%d] Processed 10 minutes of example data\n", calnum);
            }

            /*** Mechanization starts here ***/

            // Read truth data for the current epoch
            ReadExamplePureINSData(m_ins, &mInsDataEpoch_cur);

            // 1. Update velocity
            VelocityUpdate(mImuDataEpoch_cur, mImuDataEpoch_prv, mImuDataEpoch_pprv,
                result_prv, result_pprv, &result_cur);

            // 2. Update position
            PositionUpdate(mImuDataEpoch_cur, mImuDataEpoch_prv, mImuDataEpoch_pprv,
                result_prv, result_pprv, &result_cur);

            double blh[3] = { 0.0,0.0,0.0 };
            blh[0] = result_cur.blh.latitude * Deg;
            blh[1] = result_cur.blh.longitude * Deg;
            blh[2] = result_cur.blh.H;

            // 3. Update attitude
            PostureUpdate(mImuDataEpoch_cur, mImuDataEpoch_prv,
                result_prv, Q_prv, &Q_cur, &result_cur);

            double POS[3] = { 0.0,0.0,0.0 };
            POS[0] = result_cur.pos.roll * Deg;
            POS[1] = result_cur.pos.pitch * Deg;
            POS[2] = result_cur.pos.yaw * Deg;

            // Save navigation result and truth data
            SaveOurResult(result_example, mImuDataEpoch_cur, result_cur.vel, result_cur.blh, result_cur.pos);

            if (mInsDataEpoch_cur.pos.yaw < 0)mInsDataEpoch_cur.pos.yaw += 360;
            SaveTrueResult(true_example, mInsDataEpoch_cur);

            // Save error result
            double dvel[3] = { result_cur.vel.Vn - mInsDataEpoch_cur.vel.Vn, result_cur.vel.Ve - mInsDataEpoch_cur.vel.Ve, result_cur.vel.Vd - mInsDataEpoch_cur.vel.Vd };
            double dblh[3] = { blh[0] - mInsDataEpoch_cur.blh.latitude, blh[1] - mInsDataEpoch_cur.blh.longitude, blh[2] - mInsDataEpoch_cur.blh.H };
            if (POS[2] < 0)POS[2] += 360;
            double dpos[3] = { POS[0] - mInsDataEpoch_cur.pos.roll, POS[1] - mInsDataEpoch_cur.pos.pitch, POS[2] - mInsDataEpoch_cur.pos.yaw };
            if (fabs(mInsDataEpoch_cur.TimeStamp - mInsDataEpoch_cur.TimeStamp) < 1e-3)
                SaveDiffResult(diff_result_example, mImuDataEpoch_cur, dvel, dblh, dpos);
            else {
                cout << "The timestamp of truedata and ours isnt syn!\n";
            }

            // Compute and save dENU track
            double truexyz[3] = { 0.0,0.0,0.0 };
            double ourxyz[3] = { 0.0,0.0,0.0 };
            dENU truedenu, ourdenu;
            mInsDataEpoch_cur.blh.latitude *= Rad; mInsDataEpoch_cur.blh.longitude *= Rad;

            BLHToXYZ(mInsDataEpoch_cur.blh, truexyz, R_WGS84, F_WGS84);
            BLHToXYZ(result_cur.blh, ourxyz, R_WGS84, F_WGS84);
            CompEnudPos(basexyz, truexyz, &ZeroPoint, &truedenu);
            CompEnudPos(basexyz, ourxyz, &ZeroPoint, &ourdenu);

            SavedENUResult(denu_result, mImuDataEpoch_cur, truedenu, ourdenu);

            // 4. Refresh cached epochs
            mImuDataEpoch_pprv = mImuDataEpoch_prv; // Update IMU cache
            mImuDataEpoch_prv = mImuDataEpoch_cur;

            result_pprv = result_prv;   // Update navigation cache
            result_prv = result_cur;
            Q_prv = Q_cur;      // Update quaternion cache
        }

        cout << "Example processing finished. Results are stored in the result folder.\n";
        // Close files
        fclose(m_imu);
        fclose(m_ins);
        fclose(result_example);
        fclose(true_example);
        fclose(diff_result_example);
        fclose(denu_result);
        break;  // End case
    }
    case 1:
    {
        cout << "Now is solution of SelfData! Calculating...\n";

        // Open self-collected data and truth files
        string ourdata_file_path = "Data\\GroupOne.ASC";    // Self-collected IMU data
        string truth_file_path = "Data\\TruthOne.nav";      // Reference truth

        ifstream ourdata_file(ourdata_file_path);
        ifstream truth_file(truth_file_path);
        if (!ourdata_file) { cerr << "Cannot open the file to read our data!\n"; return -1; }
        if (!truth_file) { cerr << "Cannot open the file to read truth data!\n"; return -1; }

        FILE* result_file = fopen("Result/result.txt", "wb");   // Save navigation result
        if(!result_file) { cerr << "Cannot open the file to save results!\n"; return -1; }
        fprintf(result_file, "# TimeStamp,BLH(deg),Vel(m/s),Pos(deg)\n");    // Result file header
        
        FILE* true_file = fopen("Result/true.txt", "wb");   // Save reference truth
        if(!true_file) { cerr << "Cannot open the file to save true data!\n"; return -1; }
        fprintf(true_file, "# TimeStamp,BLH(deg),Vel(m/s),Pos(deg)\n");    // Truth file header

        FILE* diff_result_file = fopen("Result/diff_result.txt", "wb");   // Save error result
        if(!diff_result_file) { cerr << "Cannot open the file to save diff results!\n"; return -1; }
        fprintf(diff_result_file, "# TimeStamp,dVel(m/s),dBLH(deg),dPos(deg)\n");    // Diff file header

        FILE* denu_result_file = fopen("Result/denu_result.txt", "wb");   // Save dENU track result
        if(!denu_result_file) { cerr << "Cannot open the file to save denu results!\n"; return -1; }
        fprintf(denu_result_file, "# TimeStamp,truedata dneu(m),ourdata dneu(m)\n");    // dENU file header

        // Cached states from previous epochs
        IMUDataEpoch ourdata_prv;   // Previous raw data epoch
        IMUDataEpoch ourdata_pprv;  // Pre-previous raw data epoch
        INSDataEpoch result_prv;    // Previous navigation result
        INSDataEpoch result_pprv;   // Pre-previous navigation result
        Quater Q_prv;   // Previous quaternion

        INSDataEpoch truedata;    // Current truth data

        TimeIntervalsArray ZeroSpeed;   // Zero-velocity interval table

        POSITION ZeroPoint; // Reference origin for dENU computation
        double basexyz[3] = { 0.0,0.0,0.0 };
        //ZeroPoint.latitude = ours_initial_latitude; ZeroPoint.longitude = ours_initial_longitude; ZeroPoint.H = ours_initial_height;
        ZeroPoint.latitude = 30.527908355 * Rad; ZeroPoint.longitude = 114.355645940 * Rad; ZeroPoint.H = 23.3442;  // Reference point from rtkplot
        BLHToXYZ(ZeroPoint, basexyz, R_WGS84, F_WGS84);

        bool Is_prv = false;    // Whether previous epoch is available
        bool Is_pprv = false;   // Whether pre-previous epoch is available
        bool flag_imu = true;   // IMU read loop flag
        bool Is_Cali = false;   // Whether static calibration is enabled
        bool Is_Zero = false;   // Whether zero-velocity update is enabled

        double epochnum = 0.0;  // Number of epochs used in averaging
        double accmean[3] = { 0.0,0.0,0.0 }, gyrmean[3] = { 0.0,0.0,0.0 };  // Mean acceleration and angular rate

        double caltime = 0.0;   // Accumulated processed time
        int calnum = 0;         // Number of processed 10-minute blocks

        // Epoch-by-epoch solution
        while (flag_imu == true)
        {
            // Current epoch variables
            IMUDataEpoch ourdata_cur;   // Current raw data epoch
            INSDataEpoch result_cur;    // Current navigation result
            Quater Q_cur;   // Current quaternion

            // Read the current IMU epoch
            DeviceType PureIMU_One = CGI;   // Device type used in this flow
            DeviceType PureIMU_Five = XWGI; // Reserved alternative device type
            if (flag_imu)flag_imu = ReadIMURawData_CGI(ourdata_file, &ourdata_cur, PureIMU_One);    // Read self-collected data
            if (ourdata_cur.TimeStamp > endtime)break;

            // Accumulate static mean before truth starts, used for calibration if needed
            if (ourdata_cur.TimeStamp < starttime - 0.02)
            {
                // Update mean value
                CalAvgAcc_Gyr(ourdata_cur, &epochnum, accmean, gyrmean);

                continue;
            }

            // Apply static calibration
            if (Is_Cali == true)
            {
                //printf("%0.8f %0.8f %0.8f %0.8f %0.8f %0.8f", ourdata_cur.Acc.X, ourdata_cur.Acc.Y, ourdata_cur.Acc.Z, ourdata_cur.Gyr.X, ourdata_cur.Gyr.Y, ourdata_cur.Gyr.Z);
                AccCalibration(accmean, &ourdata_cur);
                //GyrCalibration(gyrmean, &ourdata_cur);
                //printf(" || %0.8f %0.8f %0.8f %0.8f %0.8f %0.8f\n", ourdata_cur.Acc.X, ourdata_cur.Acc.Y, ourdata_cur.Acc.Z, ourdata_cur.Gyr.X, ourdata_cur.Gyr.Y, ourdata_cur.Gyr.Z);
            }

            // Wait until two historical IMU epochs are available
            if (!Is_prv)    // First cache fill
            {
                ourdata_prv = ourdata_cur;
                Is_prv = true;
                continue;
            }
            else if(!Is_pprv)   // Second cache fill and state initialization
            {
                ourdata_pprv = ourdata_prv;
                ourdata_prv = ourdata_cur;
                Is_pprv = true;

                // Initialize navigation state before formal solution starts
                /* 1. Velocity initialization */
                /* 2. Position initialization */
                result_prv.blh.latitude = ours_initial_latitude; result_prv.blh.longitude = ours_initial_longitude; result_prv.blh.H = ours_initial_height;
                result_pprv.blh.latitude = ours_initial_latitude; result_pprv.blh.longitude = ours_initial_longitude; result_pprv.blh.H = ours_initial_height;
                /* 3. Attitude initialization */
                result_prv.pos.roll = ours_initial_roll; result_prv.pos.pitch = ours_initial_pitch; result_prv.pos.yaw = ours_initial_yaw;
                /* 4. Build the previous quaternion */
                Q_prv.SetQbn(result_prv.pos);

                continue;
            }

            // Accumulate processed time
            caltime += (ourdata_cur.TimeStamp - ourdata_prv.TimeStamp); // Count solution time
            if (caltime >= 600.0)   // Print once every 10 minutes
            {
                caltime = 0.0;
                calnum += 1;

                printf("[%d] Processed 10 minutes of self data\n", calnum);
            }

            /*** Mechanization starts here ***/
            ReadTruthData(truth_file, &truedata);   // Read truth data
            while (truedata.TimeStamp < starttime)  // Synchronize to the start time
            {
                ReadTruthData(truth_file, &truedata);   // Keep reading truth data
            }

            result_cur.TimeStamp = ourdata_cur.TimeStamp;   // Set result timestamp

            // 1. Update velocity
            VelocityUpdate(ourdata_cur, ourdata_prv, ourdata_pprv,
                result_prv, result_pprv, &result_cur);

            if (Is_Zero == true)    // Apply zero-velocity update if enabled
            {
                // Check whether current time falls inside any zero-velocity interval
                for (int i = 0; i < zero_time_intervals_num; i++)
                {
                    if (ourdata_cur.TimeStamp > ZeroSpeed.getInterval(i).end)
                    {
                        ZeroSpeed.getInterval(i).used = true;   // This interval has been consumed
                        continue;
                    }

                    if (i < zero_time_intervals_num - 1 &&
                        ourdata_cur.TimeStamp > ZeroSpeed.getInterval(i).end &&
                        ourdata_cur.TimeStamp < ZeroSpeed.getInterval(i + 1).start) // Between two intervals, exit early
                        break;

                    if (ourdata_cur.TimeStamp >= ZeroSpeed.getInterval(i).start &&
                        ourdata_cur.TimeStamp <= ZeroSpeed.getInterval(i).end)  // Inside a zero-velocity interval
                    {
                        result_cur.vel.Vn = 0.0;
                        result_cur.vel.Ve = 0.0;
                        result_cur.vel.Vd = 0.0;
                        break;
                    }
                }
            }

            // 2. Update position
            PositionUpdate(ourdata_cur, ourdata_prv, ourdata_pprv,
                result_prv, result_pprv, &result_cur);

            double blh[3] = { 0.0,0.0,0.0 };
            blh[0] = result_cur.blh.latitude * Deg;
            blh[1] = result_cur.blh.longitude * Deg;
            blh[2] = result_cur.blh.H;

            // 3. Update attitude
            PostureUpdate(ourdata_cur, ourdata_prv,
                result_prv, Q_prv, &Q_cur, &result_cur);

            double POS[3] = { 0.0,0.0,0.0 };
            POS[0] = result_cur.pos.roll * Deg;
            POS[1] = result_cur.pos.pitch * Deg;
            POS[2] = result_cur.pos.yaw * Deg;
            if (POS[2] < 0)POS[2] += 360;

            // Save navigation result and truth data
            if (fabs(ourdata_cur.TimeStamp - truedata.TimeStamp) < 1e-3 && ourdata_cur.TimeStamp <= endtime)
            {
                SaveOurResult(result_file, ourdata_cur, result_cur.vel, result_cur.blh, result_cur.pos);
                SaveTrueResult(true_file, truedata);
            }
            else {
                printf("Current time: %0.3f Truth time: %0.3f Not synchronized\n", ourdata_cur.TimeStamp, truedata.TimeStamp);
            }

            // Save difference result
            if (fabs(truedata.TimeStamp - result_cur.TimeStamp) < 1e-3 && ourdata_cur.TimeStamp <= endtime)
            {
                double dvel[3] = { result_cur.vel.Vn - truedata.vel.Vn,result_cur.vel.Ve - truedata.vel.Ve, result_cur.vel.Vd - truedata.vel.Vd };
                double dblh[3] = { blh[0] - truedata.blh.latitude,blh[1] - truedata.blh.longitude,blh[2] - truedata.blh.H };
                double dpos[3] = { POS[0] - truedata.pos.roll,POS[1] - truedata.pos.pitch,POS[2] - truedata.pos.yaw };

                if (dpos[2] > 180)dpos[2] -= 360;
                if (dpos[2] < -180)dpos[2] += 360;
                SaveDiffResult(diff_result_file, ourdata_cur, dvel, dblh, dpos);
            }
            else {
                cout << "The timestamp of truedata and ours isnt syn!\n";
            }

            // Save dENU track result
            double truexyz[3] = { 0.0,0.0,0.0 };
            double ourxyz[3] = { 0.0,0.0,0.0 };
            dENU truedenu, ourdenu;

            truedata.blh.latitude *= Rad; truedata.blh.longitude *= Rad;
            BLHToXYZ(truedata.blh, truexyz, R_WGS84, F_WGS84);
            BLHToXYZ(result_cur.blh, ourxyz, R_WGS84, F_WGS84);
            CompEnudPos(basexyz, truexyz, &ZeroPoint, &truedenu);
            CompEnudPos(basexyz, ourxyz, &ZeroPoint, &ourdenu);

            SavedENUResult(denu_result_file, ourdata_cur, truedenu, ourdenu);

            // 4. Refresh cached epochs
            ourdata_pprv = ourdata_prv; // Update raw data cache
            ourdata_prv = ourdata_cur;

            result_pprv = result_prv;   // Update navigation cache
            result_prv = result_cur;
            Q_prv = Q_cur;      // Update quaternion cache
        }

        // Print used zero-velocity intervals
        for (int i = 0; i < zero_time_intervals_num; i++)
        {
            if (ZeroSpeed.getInterval(i).used == true)
                printf("Zero-velocity interval: %d %0.f - %0.f duration %0.f\n", i + 1,
                    ZeroSpeed.getInterval(i).start, ZeroSpeed.getInterval(i).end,
                    ZeroSpeed.getInterval(i).end - ZeroSpeed.getInterval(i).start);
        }

        cout << "Self data processing finished. Results are stored in the result folder.\n";
        // Close files
        ourdata_file.close();
        truth_file.close();
        fclose(result_file);
        fclose(true_file);
        fclose(diff_result_file);
        fclose(denu_result_file);

        break;  // End case
    }
    }

    return 0;
}
