function RmseWithTime(data1, data2, window_size) 
    % data1 包含 velocity 和姿态数据
    % data2 包含 ENU 位置信息

    % 提取数据
    second1 = table2array(data1(:,1));
    Vn = table2array(data1(:,2));
    Ve = table2array(data1(:,3));
    Vd = table2array(data1(:,4));

    roll = table2array(data1(:,8));
    pitch = table2array(data1(:,9));
    yaw = table2array(data1(:,10));

    second2 = table2array(data2(:,1));
    dN_true = table2array(data2(:,2));
    dE_true = table2array(data2(:,3));
    dU_true = table2array(data2(:,4));  % dU数据未使用

    dN_our = table2array(data2(:,5));
    dE_our = table2array(data2(:,6));
    dU_our = table2array(data2(:,7));  % dU数据未使用

    % 计算速度、位置、姿态的 RMSE
    RMSE_vn = calculate_rmse_window(second1, Vn, window_size);
    RMSE_ve = calculate_rmse_window(second1, Ve, window_size);
    RMSE_vd = calculate_rmse_window(second1, Vd, window_size);

    RMSE_n = calculate_rmse_window(second2, dN_our - dN_true, window_size);
    RMSE_e = calculate_rmse_window(second2, dE_our - dE_true, window_size);
    RMSE_u = calculate_rmse_window(second2, dU_our - dU_true, window_size);

    RMSE_roll = calculate_rmse_window(second1, roll, window_size);
    RMSE_pitch = calculate_rmse_window(second1, pitch, window_size);
    RMSE_yaw = calculate_rmse_window(second1, yaw, window_size);
    
    % 最后时刻的误差（评定惯导导航级别）
    mile = 1852; % 1海里等于1852米
    dN_last = dN_our(end) - dN_true(end);
    dE_last = dE_our(end) - dE_true(end);
    dU_last = dU_our(end) - dU_true(end);

    dVn_last = Vn(end);
    dVe_last = Ve(end);
    dVd_last = Vd(end);

    droll_last = roll(end);
    dpitch_last = pitch(end);
    dyaw_last = yaw(end);

    % 计算总时间
    total_time1 = (second1(end) - second1(1)) / 3600.0;
    total_time2 = (second2(end) - second2(1)) / 3600.0;    

    fprintf("Total time is: %0.4f\n\n", total_time1);
    position_3d_error = sqrt(dN_last^2 + dE_last^2 + dU_last^2);
    fprintf("Position Error(m/h): %0.15f %0.15f %0.15f %0.15f\n", ...
        abs(dN_last/total_time2), abs(dE_last/total_time2), abs(dU_last/total_time2), ...
        position_3d_error/total_time2);
    fprintf("Position Error(mile/h): %0.15f %0.15f %0.15f %0.15f\n\n", ...
        abs(dN_last/total_time2)/mile, abs(dE_last/total_time2)/mile, abs(dU_last/total_time2)/mile, ...
        position_3d_error/total_time2/mile);

    velocity_3d_error = sqrt(dVn_last^2 + dVe_last^2 + dVd_last^2);
    fprintf("Velocity Error(m/h2): %0.15f %0.15f %0.15f %0.15f\n", ...
        abs(dVn_last/total_time1), abs(dVe_last/total_time1), abs(dVd_last/total_time1), ...
        velocity_3d_error/total_time1);
    fprintf("Velocity Error(mile/h2): %0.15f %0.15f %0.15f %0.15f\n\n", ...
        abs(dVn_last/total_time1)/mile, abs(dVe_last/total_time1)/mile, abs(dVd_last/total_time1)/mile, ...
        velocity_3d_error/total_time1/mile);
    
    posture_3d_error = sqrt(droll_last^2 + dpitch_last^2 + dyaw_last^2);
    fprintf("Posture Error(deg/s): %0.15f %0.15f %0.15f %0.15f\n", ...
        abs(droll_last/total_time1), abs(dpitch_last/total_time1), abs(dyaw_last/total_time1), ...
        posture_3d_error/total_time1);

    % 绘制结果
    figure;

    % 绘制速度RMSE
    subplot(3,3,1);  
    plot(second1(1:window_size:end), RMSE_vn, 'r-', 'LineWidth', 1.5);
    ylabel('Vn RMSE (m/s)');
    xlabel('Time (s)');  % 添加X轴标签
    grid on;
    xlim([second1(1), second1(end)]);  % 设置X轴范围为整个数据时间段

    subplot(3,3,2);  
    plot(second1(1:window_size:end), RMSE_ve, 'r-', 'LineWidth', 1.5);
    ylabel('Ve RMSE (m/s)');
    xlabel('Time (s)');  % 添加X轴标签
    grid on;
    xlim([second1(1), second1(end)]);  % 设置X轴范围为整个数据时间段

    subplot(3,3,3);  
    plot(second1(1:window_size:end), RMSE_vd, 'r-', 'LineWidth', 1.5);
    ylabel('Vd RMSE (m/s)');
    xlabel('Time (s)');  % 添加X轴标签
    grid on;
    xlim([second1(1), second1(end)]);  % 设置X轴范围为整个数据时间段

    % 绘制位置RMSE
    subplot(3,3,4);  
    plot(second2(1:window_size:end), RMSE_n, 'g-', 'LineWidth', 1.5);
    ylabel('dN RMSE (m)');
    xlabel('Time (s)');  % 添加X轴标签
    grid on;
    xlim([second2(1), second2(end)]);  % 设置X轴范围为整个数据时间段

    subplot(3,3,5);  
    plot(second2(1:window_size:end), RMSE_e, 'g-', 'LineWidth', 1.5);
    ylabel('dE RMSE (m)');
    xlabel('Time (s)');  % 添加X轴标签
    grid on;
    xlim([second2(1), second2(end)]);  % 设置X轴范围为整个数据时间段

    subplot(3,3,6);  
    plot(second2(1:window_size:end), RMSE_u, 'g-', 'LineWidth', 1.5);
    ylabel('dU RMSE (m)');
    xlabel('Time (s)');  % 添加X轴标签
    grid on;
    xlim([second2(1), second2(end)]);  % 设置X轴范围为整个数据时间段

    % 绘制姿态RMSE
    subplot(3,3,7);  
    plot(second1(1:window_size:end), RMSE_roll, 'b-', 'LineWidth', 1.5);
    ylabel('Roll RMSE (deg)');
    xlabel('Time (s)');  % 添加X轴标签
    grid on;
    xlim([second1(1), second1(end)]);  % 设置X轴范围为整个数据时间段

    subplot(3,3,8);  
    plot(second1(1:window_size:end), RMSE_pitch, 'b-', 'LineWidth', 1.5);
    ylabel('Pitch RMSE (deg)');
    xlabel('Time (s)');  % 添加X轴标签
    grid on;
    xlim([second1(1), second1(end)]);  % 设置X轴范围为整个数据时间段

    subplot(3,3,9);  
    plot(second1(1:window_size:end), RMSE_yaw, 'b-', 'LineWidth', 1.5);
    ylabel('Yaw RMSE (deg)');
    xlabel('Time (s)');  % 添加X轴标签
    grid on;
    xlim([second1(1), second1(end)]);  % 设置X轴范围为整个数据时间段

end

function RMSE_values = calculate_rmse_window(time, errors, window_size)
    % 获取数据长度
    N = length(errors);
    RMSE_values = [];
    
    % 循环计算每十秒（窗口大小为 window_size）一个RMSE
    for start_idx = 1:window_size:N - window_size + 1
        end_idx = start_idx + window_size - 1;  % 确定窗口的结束位置
        
        % 计算当前窗口的RMSE
        current_rmse = sqrt(mean(errors(start_idx:end_idx).^2));
        RMSE_values = [RMSE_values, current_rmse];  % 将计算出的RMSE添加到结果中
    end
end

