function dEnuFig(data)
    % 将数据转换为数组
    second = table2array(data(:,1));
    dN_true = table2array(data(:,2));
    dE_true = table2array(data(:,3));
    dU_true = table2array(data(:,4));  % dU数据未使用

    dN_our = table2array(data(:,5));
    dE_our = table2array(data(:,6));
    dU_our = table2array(data(:,7));  % dU数据未使用
    
    ddN = dN_true - dN_our;
    ddE = dE_true - dE_our;
    ddU = dU_true - dU_our;

    fprintf("  dN_mean = %0.15f dE_mean = %0.15f dU_mean = %0.15f\n", ...
        mean(ddN), mean(ddE), mean(ddU));
    fprintf("  dN_RMSE = %0.15f dE_RMSE = %0.15f dU_RMSE = %0.15f\n", ...
        calculate_rmse(ddN), calculate_rmse(ddE), calculate_rmse(ddU));

    % 绘制 dN 和 dE 的二维图
    figure;
    hold on;

    % 画解算结果 (dN_our, dE_our) 与参考真值 (dN_true, dE_true)
    % plot(dE_true, dN_true, 'm-', 'LineWidth', 5, 'DisplayName', 'True Data');  % 参考数据，蓝色圆点线
    plot(dE_our, dN_our, 'r-', 'LineWidth', 5, 'DisplayName', 'Our Data');    % 解算数据，红色星号线

    % 设置图形属性
    xlabel('East (m)');  % x 轴标签
    ylabel('North (m)'); % y 轴标签
    legend('show');  % 显示图例
    grid on;  % 显示网格

    % 设置轴范围（根据数据范围动态调整）
    xlim([min(dE_true) - 0.1, max(dE_true) + 0.1]);
    ylim([min(dN_true) - 0.1, max(dN_true)]);

    % 保证 x 轴和 y 轴的分度值一致
    axis equal;  % 使得 x 轴和 y 轴的单位长度相等

    hold off;

    %% 绘制：位置差分
    figure;
    subplot(3,1,1);
    hold on;
    plot(second, ddN, 'r-', 'LineWidth', 1.5, 'DisplayName', 'dN');  % 设置线条粗细
    ylabel('dN (m)');
    legend('show');
    xlim([min(second), max(second)]);  % 确保横轴没有空白
    grid on;
    hold off;

    subplot(3,1,2);
    hold on;
    plot(second, ddE, 'g-', 'LineWidth', 1.5, 'DisplayName', 'dE');  % 设置线条粗细
    ylabel('dE (m)');
    legend('show');
    xlim([min(second), max(second)]);  % 确保横轴没有空白
    grid on;
    hold off;

    subplot(3,1,3);
    hold on;
    plot(second, ddU, 'b-', 'LineWidth', 1.5, 'DisplayName', 'dU');  % 设置线条粗细
    xlabel('Time (s)');
    ylabel('dU (m)');
    legend('show');
    xlim([min(second), max(second)]);  % 确保横轴没有空白
    grid on;
    hold off;

end

% 自定义RMSE计算函数
function rmse_value = calculate_rmse(errors)
    % 计算 RMSE: 平方差的均值的平方根
    rmse_value = sqrt(mean(errors.^2));  % 计算误差的平方和的均值
end