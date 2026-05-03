% function DataFig(data_our, data_true)
%     % 将数据转换为数组
%     second_our = table2array(data_our(:,1));
%     B_our = table2array(data_our(:,2));
%     L_our = table2array(data_our(:,3));
%     H_our = table2array(data_our(:,4));
% 
%     Vn_our = table2array(data_our(:,5));
%     Ve_our = table2array(data_our(:,6));
%     Vd_our = table2array(data_our(:,7));
% 
%     roll_our = table2array(data_our(:,8));
%     pitch_our = table2array(data_our(:,9));
%     yaw_our = table2array(data_our(:,10));
% 
%     % 参考真值数据转换
%     B_true = table2array(data_true(:,2));
%     L_true = table2array(data_true(:,3));
%     H_true = table2array(data_true(:,4));
% 
%     Vn_true = table2array(data_true(:,5));
%     Ve_true = table2array(data_true(:,6));
%     Vd_true = table2array(data_true(:,7));
% 
%     roll_true = table2array(data_true(:,8));
%     pitch_true = table2array(data_true(:,9));
%     yaw_true = table2array(data_true(:,10));
% 
%     %% 绘制第一张图：速度（NED）
%     figure;
%     subplot(3,1,1);
%     hold on;
%     plot(second_our, Vn_our, 'r', 'LineWidth', 1.5, 'DisplayName', 'Vn (Our)');  % 设置线条粗细
%     plot(second_our, Vn_true, 'b--', 'LineWidth', 1, 'DisplayName', 'Vn (True)');  % 设置线条粗细
%     ylabel('Vn (m/s)');
%     legend('show');
%     xlim([min(second_our), max(second_our)]);  % 确保横轴没有空白
%     grid on;
%     hold off;
% 
%     subplot(3,1,2);
%     hold on;
%     plot(second_our, Ve_our, 'g', 'LineWidth', 1.5, 'DisplayName', 'Ve (Our)');  % 设置线条粗细
%     plot(second_our, Ve_true, 'k--', 'LineWidth', 1, 'DisplayName', 'Ve (True)');  % 设置线条粗细
%     ylabel('Ve (m/s)');
%     legend('show');
%     xlim([min(second_our), max(second_our)]);  % 确保横轴没有空白
%     grid on;
%     hold off;
% 
%     subplot(3,1,3);
%     hold on;
%     plot(second_our, Vd_our, 'm', 'LineWidth', 1.5, 'DisplayName', 'Vd (Our)');  % 设置线条粗细
%     plot(second_our, Vd_true, 'c--', 'LineWidth', 1, 'DisplayName', 'Vd (True)');  % 设置线条粗细
%     xlabel('Time (s)');
%     ylabel('Vd (m/s)');
%     legend('show');
%     xlim([min(second_our), max(second_our)]);  % 确保横轴没有空白
%     grid on;
%     hold off;
% 
%     %% 绘制第二张图：位置（B, L, H）
%     figure;
%     subplot(3,1,1);
%     hold on;
%     plot(second_our, B_our, 'r', 'LineWidth', 1.5, 'DisplayName', 'B (Our)');  % 设置线条粗细
%     plot(second_our, B_true, 'b--', 'LineWidth', 1, 'DisplayName', 'B (True)');  % 设置线条粗细
%     ylabel('Latitude (deg)');
%     legend('show');
%     xlim([min(second_our), max(second_our)]);  % 确保横轴没有空白
%     grid on;
%     hold off;
% 
%     subplot(3,1,2);
%     hold on;
%     plot(second_our, L_our, 'g', 'LineWidth', 1.5, 'DisplayName', 'L (Our)');  % 设置线条粗细
%     plot(second_our, L_true, 'k--', 'LineWidth', 1, 'DisplayName', 'L (True)');  % 设置线条粗细
%     ylabel('Longitude (deg)');
%     legend('show');
%     xlim([min(second_our), max(second_our)]);  % 确保横轴没有空白
%     grid on;
%     hold off;
% 
%     subplot(3,1,3);
%     hold on;
%     plot(second_our, H_our, 'm', 'LineWidth', 1.5, 'DisplayName', 'H (Our)');  % 设置线条粗细
%     plot(second_our, H_true, 'c--', 'LineWidth', 1, 'DisplayName', 'H (True)');  % 设置线条粗细
%     xlabel('Time (s)');
%     ylabel('Height (m)');
%     legend('show');
%     xlim([min(second_our), max(second_our)]);  % 确保横轴没有空白
%     grid on;
%     hold off;
% 
%     %% 绘制第三张图：姿态（Roll, Pitch, Yaw）
%     figure;
%     subplot(3,1,1);
%     hold on;
%     plot(second_our, roll_our, 'r', 'LineWidth', 1.5, 'DisplayName', 'Roll (Our)');  % 设置线条粗细
%     plot(second_our, roll_true, 'b--', 'LineWidth', 1, 'DisplayName', 'Roll (True)');  % 设置线条粗细
%     ylabel('Roll (deg)');
%     legend('show');
%     xlim([min(second_our), max(second_our)]);  % 确保横轴没有空白
%     grid on;
%     hold off;
% 
%     subplot(3,1,2);
%     hold on;
%     plot(second_our, pitch_our, 'g', 'LineWidth', 1.5, 'DisplayName', 'Pitch (Our)');  % 设置线条粗细
%     plot(second_our, pitch_true, 'k--', 'LineWidth', 1, 'DisplayName', 'Pitch (True)');  % 设置线条粗细
%     ylabel('Pitch (deg)');
%     legend('show');
%     xlim([min(second_our), max(second_our)]);  % 确保横轴没有空白
%     grid on;
%     hold off;
% 
%     subplot(3,1,3);
%     hold on;
%     plot(second_our, yaw_our, 'm', 'LineWidth', 1.5, 'DisplayName', 'Yaw (Our)');  % 设置线条粗细
%     plot(second_our, yaw_true, 'c--', 'LineWidth', 1, 'DisplayName', 'Yaw (True)');  % 设置线条粗细
%     xlabel('Time (s)');
%     ylabel('Yaw (deg)');
%     legend('show');
%     xlim([min(second_our), max(second_our)]);  % 确保横轴没有空白
%     grid on;
%     hold off;
% end

function DataFig(data_our, data_true)
    % 将数据转换为数组
    second_our = table2array(data_our(:,1));
    B_our = table2array(data_our(:,2));
    L_our = table2array(data_our(:,3));
    H_our = table2array(data_our(:,4));

    Vn_our = table2array(data_our(:,5));
    Ve_our = table2array(data_our(:,6));
    Vd_our = table2array(data_our(:,7));

    roll_our = table2array(data_our(:,8));
    pitch_our = table2array(data_our(:,9));
    yaw_our = table2array(data_our(:,10));

    % 参考真值数据转换
    B_true = table2array(data_true(:,2));
    L_true = table2array(data_true(:,3));
    H_true = table2array(data_true(:,4));

    Vn_true = table2array(data_true(:,5));
    Ve_true = table2array(data_true(:,6));
    Vd_true = table2array(data_true(:,7));

    roll_true = table2array(data_true(:,8));
    pitch_true = table2array(data_true(:,9));
    yaw_true = table2array(data_true(:,10));

    %% 绘制第一张图：速度（NED）
    figure;
    subplot(3,3,1);  % 调整为较小的子图布局
    hold on;
    plot(second_our, Vn_our, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Vn (Our)');  % 设置线条粗细
    plot(second_our, Vn_true, 'b-', 'LineWidth', 1, 'DisplayName', 'Vn (True)');  % 设置线条粗细
    ylabel('Vn (m/s)');
    legend('show');
    xlim([min(second_our), max(second_our)]);  % 确保横轴没有空白
    grid on;
    hold off;

    subplot(3,3,2);
    hold on;
    plot(second_our, Ve_our, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Ve (Our)');  % 设置线条粗细
    plot(second_our, Ve_true, 'b-', 'LineWidth', 1, 'DisplayName', 'Ve (True)');  % 设置线条粗细
    ylabel('Ve (m/s)');
    legend('show');
    xlim([min(second_our), max(second_our)]);  % 确保横轴没有空白
    grid on;
    hold off;

    subplot(3,3,3);
    hold on;
    plot(second_our, Vd_our, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Vd (Our)');  % 设置线条粗细
    plot(second_our, Vd_true, 'b-', 'LineWidth', 1, 'DisplayName', 'Vd (True)');  % 设置线条粗细
    ylabel('Vd (m/s)');
    legend('show');
    xlim([min(second_our), max(second_our)]);  % 确保横轴没有空白
    grid on;
    hold off;

    %% 绘制第二张图：位置（B, L, H）
    subplot(3,3,4);
    hold on;
    plot(second_our, B_our, 'g-', 'LineWidth', 1.5, 'DisplayName', 'B (Our)');  % 设置线条粗细
    plot(second_our, B_true, 'k-', 'LineWidth', 1, 'DisplayName', 'B (True)');  % 设置线条粗细
    ylabel('Latitude (deg)');
    legend('show');
    xlim([min(second_our), max(second_our)]);  % 确保横轴没有空白
    grid on;
    hold off;

    subplot(3,3,5);
    hold on;
    plot(second_our, L_our, 'g-', 'LineWidth', 1.5, 'DisplayName', 'L (Our)');  % 设置线条粗细
    plot(second_our, L_true, 'k-', 'LineWidth', 1, 'DisplayName', 'L (True)');  % 设置线条粗细
    ylabel('Longitude (deg)');
    legend('show');
    xlim([min(second_our), max(second_our)]);  % 确保横轴没有空白
    grid on;
    hold off;

    subplot(3,3,6);
    hold on;
    plot(second_our, H_our, 'g-', 'LineWidth', 1.5, 'DisplayName', 'H (Our)');  % 设置线条粗细
    plot(second_our, H_true, 'k-', 'LineWidth', 1, 'DisplayName', 'H (True)');  % 设置线条粗细
    ylabel('Height (m)');
    legend('show');
    xlim([min(second_our), max(second_our)]);  % 确保横轴没有空白
    grid on;
    hold off;

    %% 绘制第三张图：姿态（Roll, Pitch, Yaw）
    subplot(3,3,7);
    hold on;
    plot(second_our, roll_our, 'm-', 'LineWidth', 1.5, 'DisplayName', 'Roll (Our)');  % 设置线条粗细
    plot(second_our, roll_true, 'c-', 'LineWidth', 1, 'DisplayName', 'Roll (True)');  % 设置线条粗细
    ylabel('Roll (deg)');
    legend('show');
    xlim([min(second_our), max(second_our)]);  % 确保横轴没有空白
    xlabel('Time (s)');    
    grid on;
    hold off;

    subplot(3,3,8);
    hold on;
    plot(second_our, pitch_our, 'm-', 'LineWidth', 1.5, 'DisplayName', 'Pitch (Our)');  % 设置线条粗细
    plot(second_our, pitch_true, 'c-', 'LineWidth', 1, 'DisplayName', 'Pitch (True)');  % 设置线条粗细
    ylabel('Pitch (deg)');
    legend('show');
    xlim([min(second_our), max(second_our)]);  % 确保横轴没有空白
    xlabel('Time (s)');
    grid on;
    hold off;

    subplot(3,3,9);
    hold on;
    plot(second_our, yaw_our, 'm-', 'LineWidth', 1.5, 'DisplayName', 'Yaw (Our)');  % 设置线条粗细
    plot(second_our, yaw_true, 'c-', 'LineWidth', 1, 'DisplayName', 'Yaw (True)');  % 设置线条粗细
    xlabel('Time (s)');
    ylabel('Yaw (deg)');
    legend('show');
    xlim([min(second_our), max(second_our)]);  % 确保横轴没有空白
    grid on;
    hold off;
end
