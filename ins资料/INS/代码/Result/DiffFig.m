function DiffFig(data, mode)
    % 将数据转换为表格
    second = table2array(data(:,1));
    Vn = table2array(data(:,2));
    Ve = table2array(data(:,3));
    Vd = table2array(data(:,4));

    B = table2array(data(:,5));
    L = table2array(data(:,6));
    H = table2array(data(:,7));

    roll = table2array(data(:,8));
    pitch = table2array(data(:,9));
    yaw = table2array(data(:,10));

    fprintf("  dB_mean = %0.15f dL_mean = %0.15f dH_mean = %0.15f\n", ...
        mean(B), mean(L), mean(H));
    fprintf("  dB_RMSE = %0.15f dL_RMSE = %0.15f dH_RMSE = %0.15f\n\n", ...
        calculate_rmse(B), calculate_rmse(L), calculate_rmse(H));

    fprintf("  dVn_mean = %0.15f dVe_mean = %0.15f dVd_mean = %0.15f\n", ...
        mean(Vn), mean(Ve), mean(Vd));
    fprintf("  dVn_RMSE = %0.15f dVe_RMSE = %0.15f dVd_RMSE = %0.15f\n\n", ...
        calculate_rmse(Vn), calculate_rmse(Ve), calculate_rmse(Vd));

    fprintf("  droll_mean = %0.15f dpitch_mean = %0.15f dyaw_mean = %0.15f\n", ...
        mean(roll), mean(pitch), mean(yaw));
    fprintf("  droll_RMSE = %0.15f dpitch_RMSE = %0.15f dyaw_RMSE = %0.15f\n\n", ...
        calculate_rmse(roll), calculate_rmse(pitch), calculate_rmse(yaw));

    % 绘制第一张图：速度、位置、姿态
    figure;
    % 速度图
    subplot(3,3,1);    
    plot(second, Vn, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Vn');  % 设置线条粗细
    ylabel('Vn (m/s)');
    legend('show');
    if mode == 0
        hold on;
        plot([min(second), max(second)], [1e-4 + 1e-5, 1e-4 + 1e-5], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off');  % 添加虚线，并隐藏legend
        plot([min(second), max(second)], [-1e-4 - 1e-5, -1e-4 - 1e-5], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off'); % 添加虚线，并隐藏legend
        hold off;
    end
    xlim([min(second), max(second)]);  % 确保横轴没有空白
    grid on;

    subplot(3,3,2);
    plot(second, Ve, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Ve');  % 设置线条粗细
    if mode == 0
        hold on;
        plot([min(second), max(second)], [1e-4 + 1e-5, 1e-4 + 1e-5], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off');  % 添加虚线，并隐藏legend
        plot([min(second), max(second)], [-1e-4 - 1e-5, -1e-4 - 1e-5], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off'); % 添加虚线，并隐藏legend
        hold off;
    end    
    ylabel('Ve (m/s)');
    legend('show');    
    xlim([min(second), max(second)]);  % 确保横轴没有空白
    grid on;

    subplot(3,3,3);
    plot(second, Vd, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Vd');  % 设置线条粗细
    if mode == 0
        hold on;
        plot([min(second), max(second)], [1e-3 + 1e-4, 1e-3 + 1e-4], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off');  % 添加虚线，并隐藏legend
        plot([min(second), max(second)], [-1e-3 - 1e-4, -1e-3 - 1e-4], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off'); % 添加虚线，并隐藏legend
        hold off;
    end
    ylabel('Vd (m/s)');
    legend('show');
    grid on;
    % 设置横轴范围
    xlim([min(second), max(second)]);  % 确保横轴没有空白

    % 位置图
    subplot(3,3,4);
    plot(second, B, 'g-', 'LineWidth', 1.5, 'DisplayName', 'B');  % 设置线条粗细
    if mode == 0
        hold on;
        plot([min(second), max(second)], [1e-6 + 1e-7, 1e-6 + 1e-7], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off');  % 添加虚线，并隐藏legend
        plot([min(second), max(second)], [-1e-6 - 1e-7, -1e-6 - 1e-7], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off'); % 添加虚线，并隐藏legend
        hold off;
    end
    xlim([min(second), max(second)]);  % 确保横轴没有空白
    ylabel('Latitude (deg)');
    legend('show');
    grid on;

    subplot(3,3,5);
    plot(second, L, 'g-', 'LineWidth', 1.5, 'DisplayName', 'L');  % 设置线条粗细
    if mode == 0
        hold on;
        plot([min(second), max(second)], [1e-6 + 1e-7, 1e-6 + 1e-7], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off');  % 添加虚线，并隐藏legend
        plot([min(second), max(second)], [-1e-6 - 1e-7, -1e-6 - 1e-7], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off'); % 添加虚线，并隐藏legend
        hold off;
    end
    xlim([min(second), max(second)]);  % 确保横轴没有空白
    ylabel('Longitude (deg)');
    legend('show');
    grid on;

    subplot(3,3,6);
    plot(second, H, 'g-', 'LineWidth', 1.5, 'DisplayName', 'H');  % 设置线条粗细
    if mode == 0
        hold on;
        plot([min(second), max(second)], [2.1, 2.1], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off');  % 添加虚线，并隐藏legend
        plot([min(second), max(second)], [-2.1, -2.1], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off'); % 添加虚线，并隐藏legend
        hold off;
    end
    xlim([min(second), max(second)]);  % 确保横轴没有空白
    ylabel('Height (m)');
    legend('show');
    grid on;

    % 姿态图
    subplot(3,3,7);
    plot(second, roll, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Roll');  % 设置线条粗细
    if mode == 0
        hold on;
        plot([min(second), max(second)], [1e-6 + 1e-7, 1e-6 + 1e-7], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off');  % 添加虚线，并隐藏legend
        plot([min(second), max(second)], [-1e-6 - 1e-7, -1e-6 - 1e-7], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off'); % 添加虚线，并隐藏legend
        hold off;
    end
    xlim([min(second), max(second)]);  % 确保横轴没有空白
    xlabel('Time (s)');
    legend('show');
    ylabel('Roll (deg)');
    grid on;

    subplot(3,3,8);
    plot(second, pitch, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Pitch');  % 设置线条粗细
    if mode == 0
        hold on;
        plot([min(second), max(second)], [1e-6 + 1e-7, 1e-6 + 1e-7], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off');  % 添加虚线，并隐藏legend
        plot([min(second), max(second)], [-1e-6 - 1e-7, -1e-6 - 1e-7], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off'); % 添加虚线，并隐藏legend
        hold off;
    end
    xlim([min(second), max(second)]);  % 确保横轴没有空白
    xlabel('Time (s)');
    legend('show');
    ylabel('Pitch (deg)');
    grid on;

    subplot(3,3,9);
    plot(second, yaw, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Yaw');  % 设置线条粗细
    if mode == 0
        hold on;
        plot([min(second), max(second)], [1e-6 + 1e-7, 1e-6 + 1e-7], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off');  % 添加虚线，并隐藏legend
        plot([min(second), max(second)], [-1e-6 - 1e-7, -1e-6 - 1e-7], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off'); % 添加虚线，并隐藏legend
        hold off;
    end
    xlim([min(second), max(second)]);  % 确保横轴没有空白
    legend('show');
    xlabel('Time (s)');
    ylabel('Yaw (deg)');

    grid on;

end

% 自定义RMSE计算函数
function rmse_value = calculate_rmse(errors)
    % 计算 RMSE: 平方差的均值的平方根
    rmse_value = sqrt(mean(errors.^2));  % 计算误差的平方和的均值
end

% function DiffFig(data1, data2, mode)
%     % 将数据转换为表格
%     second = table2array(data1(:,1));
%     Vn = table2array(data1(:,2));
%     Ve = table2array(data1(:,3));
%     Vd = table2array(data1(:,4));
% 
%     B = table2array(data1(:,5));
%     L = table2array(data1(:,6));
%     H = table2array(data1(:,7));
% 
%     roll = table2array(data1(:,8));
%     pitch = table2array(data1(:,9));
%     yaw = table2array(data1(:,10));
% 
%     Vn_raw = table2array(data2(:,2));
%     Ve_raw = table2array(data2(:,3));
%     Vd_raw = table2array(data2(:,4));
% 
%     B_raw = table2array(data2(:,5));
%     L_raw = table2array(data2(:,6));
%     H_raw = table2array(data2(:,7));
% 
%     roll_raw = table2array(data2(:,8));
%     pitch_raw = table2array(data2(:,9));
%     yaw_raw = table2array(data2(:,10));
% 
%     fprintf("  dB_mean = %0.15f dL_mean = %0.15f dH_mean = %0.15f\n", ...
%         mean(B), mean(L), mean(H));
%     fprintf("  dB_RMSE = %0.15f dL_RMSE = %0.15f dH_RMSE = %0.15f\n\n", ...
%         calculate_rmse(B), calculate_rmse(L), calculate_rmse(H));
% 
%     fprintf("  dVn_mean = %0.15f dVe_mean = %0.15f dVd_mean = %0.15f\n", ...
%         mean(Vn), mean(Ve), mean(Vd));
%     fprintf("  dVn_RMSE = %0.15f dVe_RMSE = %0.15f dVd_RMSE = %0.15f\n\n", ...
%         calculate_rmse(Vn), calculate_rmse(Ve), calculate_rmse(Vd));
% 
%     fprintf("  droll_mean = %0.15f dpitch_mean = %0.15f dyaw_mean = %0.15f\n", ...
%         mean(roll), mean(pitch), mean(yaw));
%     fprintf("  droll_RMSE = %0.15f dpitch_RMSE = %0.15f dyaw_RMSE = %0.15f\n\n", ...
%         calculate_rmse(roll), calculate_rmse(pitch), calculate_rmse(yaw));
% 
%     % 绘制第一张图：速度、位置、姿态
%     figure;
%     % 速度图
%     subplot(3,3,1);    
%     hold on;
%     plot(second, Vn, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Vn Zero');  % 设置线条粗细
%     plot(second, Vn_raw, 'c-', 'LineWidth', 1.5, 'DisplayName', 'Vn None');  % 设置线条粗细
%     ylabel('Vn (m/s)');
%     legend('show');
%     if mode == 0
%         hold on;
%         plot([min(second), max(second)], [1e-4 + 1e-5, 1e-4 + 1e-5], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off');  % 添加虚线，并隐藏legend
%         plot([min(second), max(second)], [-1e-4 - 1e-5, -1e-4 - 1e-5], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off'); % 添加虚线，并隐藏legend
%         hold off;
%     end
%     xlim([min(second), max(second)]);  % 确保横轴没有空白
%     grid on;
%     hold off;
% 
%     subplot(3,3,2);
%     hold on;
%     plot(second, Ve, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Ve Zero');  % 设置线条粗细
%     plot(second, Ve_raw, 'c-', 'LineWidth', 1.5, 'DisplayName', 'Ve None');  % 设置线条粗细
%     if mode == 0
%         hold on;
%         plot([min(second), max(second)], [1e-4 + 1e-5, 1e-4 + 1e-5], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off');  % 添加虚线，并隐藏legend
%         plot([min(second), max(second)], [-1e-4 - 1e-5, -1e-4 - 1e-5], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off'); % 添加虚线，并隐藏legend
%         hold off;
%     end    
%     ylabel('Ve (m/s)');
%     legend('show');    
%     xlim([min(second), max(second)]);  % 确保横轴没有空白
%     grid on;
%     hold off;
% 
%     subplot(3,3,3);
%     hold on;
%     plot(second, Vd, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Vd Zero');  % 设置线条粗细
%     plot(second, Vd_raw, 'c-', 'LineWidth', 1.5, 'DisplayName', 'Vd None');  % 设置线条粗细
%     if mode == 0
%         hold on;
%         plot([min(second), max(second)], [1e-3 + 1e-4, 1e-3 + 1e-4], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off');  % 添加虚线，并隐藏legend
%         plot([min(second), max(second)], [-1e-3 - 1e-4, -1e-3 - 1e-4], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off'); % 添加虚线，并隐藏legend
%         hold off;
%     end
%     ylabel('Vd (m/s)');
%     legend('show');
%     grid on;
%     % 设置横轴范围
%     xlim([min(second), max(second)]);  % 确保横轴没有空白
%     hold off;
% 
%     % 位置图
%     subplot(3,3,4);
%     hold on;
%     plot(second, B, 'g-', 'LineWidth', 1.5, 'DisplayName', 'B Zero');  % 设置线条粗细
%     plot(second, B_raw, 'k-', 'LineWidth', 1.5, 'DisplayName', 'B None');  % 设置线条粗细
%     if mode == 0
%         hold on;
%         plot([min(second), max(second)], [1e-6 + 1e-7, 1e-6 + 1e-7], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off');  % 添加虚线，并隐藏legend
%         plot([min(second), max(second)], [-1e-6 - 1e-7, -1e-6 - 1e-7], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off'); % 添加虚线，并隐藏legend
%         hold off;
%     end
%     xlim([min(second), max(second)]);  % 确保横轴没有空白
%     ylabel('Latitude (deg)');
%     legend('show');
%     grid on;
%     hold off;
% 
%     subplot(3,3,5);
%     hold on;
%     plot(second, L, 'g-', 'LineWidth', 1.5, 'DisplayName', 'L Zero');  % 设置线条粗细
%     plot(second, L_raw, 'k-', 'LineWidth', 1.5, 'DisplayName', 'L None');  % 设置线条粗细
%     if mode == 0
%         hold on;
%         plot([min(second), max(second)], [1e-6 + 1e-7, 1e-6 + 1e-7], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off');  % 添加虚线，并隐藏legend
%         plot([min(second), max(second)], [-1e-6 - 1e-7, -1e-6 - 1e-7], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off'); % 添加虚线，并隐藏legend
%         hold off;
%     end
%     xlim([min(second), max(second)]);  % 确保横轴没有空白
%     ylabel('Longitude (deg)');
%     legend('show');
%     grid on;
%     hold off;
% 
%     subplot(3,3,6);
%     hold on;
%     plot(second, H, 'g-', 'LineWidth', 1.5, 'DisplayName', 'H Zero');  % 设置线条粗细
%     plot(second, H_raw, 'k-', 'LineWidth', 1.5, 'DisplayName', 'H None');  % 设置线条粗细
%     if mode == 0
%         hold on;
%         plot([min(second), max(second)], [2.1, 2.1], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off');  % 添加虚线，并隐藏legend
%         plot([min(second), max(second)], [-2.1, -2.1], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off'); % 添加虚线，并隐藏legend
%         hold off;
%     end
%     xlim([min(second), max(second)]);  % 确保横轴没有空白
%     ylabel('Height (m)');
%     legend('show');
%     grid on;
%     hold off;
% 
%     % 姿态图
%     subplot(3,3,7);
%     hold on;
%     plot(second, roll, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Roll Zero');  % 设置线条粗细
%     plot(second, roll_raw, 'm-', 'LineWidth', 1.5, 'DisplayName', 'Roll None');  % 设置线条粗细
%     if mode == 0
%         hold on;
%         plot([min(second), max(second)], [1e-6 + 1e-7, 1e-6 + 1e-7], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off');  % 添加虚线，并隐藏legend
%         plot([min(second), max(second)], [-1e-6 - 1e-7, -1e-6 - 1e-7], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off'); % 添加虚线，并隐藏legend
%         hold off;
%     end
%     xlim([min(second), max(second)]);  % 确保横轴没有空白
%     xlabel('Time (s)');
%     legend('show');
%     ylabel('Roll (deg)');
%     grid on;
%     hold off;
% 
%     subplot(3,3,8);
%     hold on;
%     plot(second, pitch, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Pitch Zero');  % 设置线条粗细
%     plot(second, pitch_raw, 'm-', 'LineWidth', 1.5, 'DisplayName', 'Pitch None');  % 设置线条粗细
%     if mode == 0
%         hold on;
%         plot([min(second), max(second)], [1e-6 + 1e-7, 1e-6 + 1e-7], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off');  % 添加虚线，并隐藏legend
%         plot([min(second), max(second)], [-1e-6 - 1e-7, -1e-6 - 1e-7], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off'); % 添加虚线，并隐藏legend
%         hold off;
%     end
%     xlim([min(second), max(second)]);  % 确保横轴没有空白
%     xlabel('Time (s)');
%     legend('show');
%     ylabel('Pitch (deg)');
%     grid on;
%     hold off;
% 
%     subplot(3,3,9);
%     hold on;
%     plot(second, yaw, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Yaw Zero');  % 设置线条粗细
%     plot(second, yaw_raw, 'm-', 'LineWidth', 1.5, 'DisplayName', 'Yaw None');  % 设置线条粗细
%     if mode == 0
%         hold on;
%         plot([min(second), max(second)], [1e-6 + 1e-7, 1e-6 + 1e-7], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off');  % 添加虚线，并隐藏legend
%         plot([min(second), max(second)], [-1e-6 - 1e-7, -1e-6 - 1e-7], 'k--', 'LineWidth', 3, 'HandleVisibility', 'off'); % 添加虚线，并隐藏legend
%         hold off;
%     end
%     xlim([min(second), max(second)]);  % 确保横轴没有空白
%     legend('show');
%     xlabel('Time (s)');
%     ylabel('Yaw (deg)');
%     hold off;   
%     grid on;
% 
% end
% 
% % 自定义RMSE计算函数
% function rmse_value = calculate_rmse(errors)
%     % 计算 RMSE: 平方差的均值的平方根
%     rmse_value = sqrt(mean(errors.^2));  % 计算误差的平方和的均值
% end
