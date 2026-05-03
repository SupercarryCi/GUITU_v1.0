% BLE与PDR定位，UKF与PF的融合、对比
% 作者联系VX:matlabfilter（除前期达成一致外，付费咨询）
% 2026-03-05/Ver1

clc; clear; close all
rng(0)  % 固定随机种子，保证结果可复现

%% 初始化
% 环境参数设置
area_x = 200;   % 区域X方向范围（米）
area_y = 50;    % 区域Y方向范围（米）
% AP（接入点）布置
% 每行为一个AP的 [x, y] 坐标
AP = [
  0   -10;   % AP1：左下角
 60   0;   % AP2：右下角
 50  20;   % AP3：右上角
  0  20;   % AP4：左上角
 10  10;   % AP5：中心
];

nAP = size(AP, 1);  % AP数量

% 信号传播模型参数
RSSI0      = -45;   % 参考距离1m处的RSSI（dBm）
pathloss   = 2.2;   % 路径损耗指数
noise_rssi = 1;   % RSSI测量噪声标准差（dBm）


%% 构建指纹库
% 对区域内均匀网格点，计算每个AP的理论RSSI值
grids = 0.2;  % 网格分辨率（米）
[xg, yg] = meshgrid(-area_x:grids:area_x, -area_y:grids:area_y);  % 生成网格
num_grid = numel(xg);  % 网格点总数
% 指纹库：[x, y, RSSI_AP1, RSSI_AP2, ..., RSSI_APn]
fingerprint = zeros(num_grid, 2 + nAP);

for i = 1:num_grid
    x = xg(i);
    y = yg(i);
    fingerprint(i, 1:2) = [x, y];  % 存储网格点坐标

    for j = 1:nAP
        d = norm([x, y] - AP(j, :));  % 计算到第j个AP的距离
        % 对数距离路径损耗模型：RSSI = RSSI0 - 10*n*log10(d)
        rssi = RSSI0 - 10 * pathloss * log10(d + 0.1);  % +0.1防止log(0)
        fingerprint(i, 2 + j) = rssi;
    end
end

% 生成真实轨迹（随机游走模型）
N = 150;    % 总步数
L = 0.7;    % 每步步长（米）
yaw = 0.01;               % 初始朝向角（弧度）
x_true = zeros(3, N);  % 状态量：[x; y; yaw]
x_true(:,1) = [-20,0,yaw]';
for k = 2:N
    yaw = yaw + 0.01 * randn;                    % 朝向角随机微扰
    x_true(1, k) = x_true(1, k-1) + L * cos(yaw);  % X方向位移
    x_true(2, k) = x_true(2, k-1) + L * sin(yaw);  % Y方向位移
    x_true(3, k) = yaw;                              % 更新朝向角
end

%% 生成PDR（行人航位推算）轨迹
% 在真实步长和朝向上叠加噪声，模拟IMU累积误差
x_pdr = zeros(3, N);
x_pdr(:,1) = x_true(:,1);
yaw = 0.01;
for k = 2:N
    step = L + 0.01 * randn;                       % 步长加噪声
    yaw  = yaw + 0.01 * randn;                     % 朝向加噪声
    x_pdr(1, k) = x_pdr(1, k-1) + step * cos(yaw);
    x_pdr(2, k) = x_pdr(2, k-1) + step * sin(yaw);
    x_pdr(3, k) = yaw;
end

% 在线RSSI测量（模拟真实场景中的BLE测量）
rssi_measure = zeros(nAP, N);
for k = 1:N
    pos = x_true(1:2, k)';  % 当前真实位置
    for j = 1:nAP
        d = norm(pos - AP(j, :));
        % 加入高斯噪声模拟实测RSSI
        rssi_measure(j, k) = RSSI0 - 10 * pathloss * log10(d + 0.1) + noise_rssi * randn;
    end
end

% KNN指纹定位
% 将当前RSSI向量与指纹库逐条比对，取欧氏距离最小的网格点坐标
pos_ble = zeros(2, N);
for k = 1:N
    rssi = rssi_measure(:, k)';               % 当前RSSI观测（行向量）
    diff = fingerprint(:, 3:end) - rssi;      % 与指纹库所有条目的差值
    dist = sum(diff.^2, 2);                   % 各网格点的匹配距离（平方和）
    [~, id] = min(dist);                      % 找最近邻
    pos_ble(:, k) = fingerprint(id, 1:2)';    % 输出对应坐标
end

%% UKF（无迹卡尔曼滤波）融合PDR与BLE定位
% 状态量：[x; y; yaw]，观测量：BLE定位坐标 [x; y]
n = 3;  % 状态维度
ukf_est = zeros(3, N);        % UKF估计状态
ukf_est(:,1) = x_true(:,1);
P = diag([1, 1, 0.5]);      % 初始协方差矩阵
% 过程噪声协方差（对应位置x/y和朝向yaw）
Q = diag([0.01, 0.01, 0.01]);
% 观测噪声协方差（对应BLE定位x/y误差，单位m^2）
R = diag([2, 2]);
% UKF参数设置
alpha  = 1e-3;               % 控制sigma点扩散程度（通常取小值）
beta   = 2;                  % 先验分布参数（高斯分布取2最优）
kappa  = 0;                  % 二级比例参数
lambda = alpha^2 * (n + kappa) - n;  % 复合比例参数
gamma  = sqrt(n + lambda);           % sigma点扩展系数
% 均值权重和协方差权重
Wm = [lambda/(n+lambda), repmat(1/(2*(n+lambda)), 1, 2*n)];
Wc = Wm;
Wc(1) = Wc(1) + (1 - alpha^2 + beta);  % 第0个协方差权重修正
for k = 2:N
    % 生成Sigma点 ---
    A = chol(P, 'lower');  % Cholesky分解
    X = [ukf_est(:, k-1), ...
         ukf_est(:, k-1) + gamma * A, ...
         ukf_est(:, k-1) - gamma * A];  % 共2n+1个sigma点
    % Sigma点传播（过程模型：匀速直线运动） ---
    X_pred = zeros(3, 2*n+1);
    for i = 1:2*n+1
        yaw = X(3, i);
        X_pred(:, i) = [
            X(1, i) + L * cos(yaw);
            X(2, i) + L * sin(yaw);
            yaw
        ];
    end
    % 计算预测均值和协方差 ---
    x_pred = sum(X_pred .* Wm, 2);
    P_pred = Q;
    for i = 1:2*n+1
        dx = X_pred(:, i) - x_pred;
        P_pred = P_pred + Wc(i) * dx * dx';
    end
    % 观测模型（直接观测位置x/y） ---
    Z = zeros(2, 2*n+1);
    Z(1, :) = X_pred(1, :);
    Z(2, :) = X_pred(2, :);
    z_pred = sum(Z .* Wm, 2);
    % 计算观测协方差和互协方差 ---
    S = R;
    for i = 1:2*n+1
        dz = Z(:, i) - z_pred;
        S = S + Wc(i) * dz * dz';
    end
    Pxz = zeros(3, 2);
    for i = 1:2*n+1
        Pxz = Pxz + Wc(i) * (X_pred(:, i) - x_pred) * (Z(:, i) - z_pred)';
    end
    % 卡尔曼增益与状态更新 ---
    K = Pxz / S;                         % 卡尔曼增益
    z = pos_ble(:, k);                   % 当前BLE观测值
    ukf_est(:, k) = x_pred + K * (z - z_pred);  % 状态更新
    P = P_pred - K * S * K';             % 协方差更新
end
%% 粒子滤波（PF）融合PDR与BLE定位
Np       = 200;                        % 粒子数（建议>=100，20太少容易退化）
weights  = ones(Np, 1) / Np;          % 初始权重均匀分布
pf_est   = zeros(3, N);               % PF估计结果 [x;y;yaw]
pf_est(:, 1) = x_true(:, 1);         % 初始状态取真值

particles = pf_est(:, 1) .* ones(1, Np);

for k = 2:N
    %  粒子传播（过程模型 + 噪声）---
    for i = 1:Np
        yaw = particles(3, i) + 0.01 * randn;          % 朝向加噪声
        particles(1, i) = particles(1, i) + L * cos(yaw) + 0.01 * randn;
        particles(2, i) = particles(2, i) + L * sin(yaw) + 0.01 * randn;
        particles(3, i) = yaw;
    end

    % 权重更新（BLE观测似然）---
    z = pos_ble(:, k);
    for i = 1:Np
        err = z - particles(1:2, i);
        weights(i) = exp(-err' * err / 2);
    end

    % 防止权重全零（数值下溢）导致除零NaN
    w_sum = sum(weights);
    if w_sum < 1e-300 || isnan(w_sum)
        weights = ones(Np, 1) / Np;   % 退化时重置为均匀权重
    else
        weights = weights / w_sum;
    end

    %- 系统重采样（替代randsample，无需工具箱）---
    % 使用系统重采样（Systematic Resampling），方差更小
    cumW = cumsum(weights);
    u0   = rand / Np;
    u    = u0 + (0:Np-1)' / Np;     % 等间距采样点
    index = zeros(Np, 1);
    j = 1;
    for i = 1:Np
        while u(i) > cumW(j) && j < Np
            j = j + 1;
        end
        index(i) = j;
    end
    particles = particles(:, index);
    weights   = ones(Np, 1) / Np;

    %--- Step4: 粒子均值作为位置估计 ---
    pf_est(:, k) = mean(particles, 2);
end

%% 误差计算（欧氏距离误差）
err_pdr = sqrt(sum((x_pdr(1:2, :) - x_true(1:2, :)).^2));
err_ble = sqrt(sum((pos_ble        - x_true(1:2, :)).^2));
err_ukf = sqrt(sum((ukf_est(1:2, :) - x_true(1:2, :)).^2));
err_pf  = sqrt(sum((pf_est(1:2, :) - x_true(1:2, :)).^2));


% 命令行输出误差统计特性对比

fprintf('定位误差统计特性对比（单位：米）\n');
fprintf('%-8s %8s %8s %8s %8s\n', '方法', '均值', '中位数', '标准差', '最大值');
fprintf('----------------------------------------------\n');

methods   = {'PDR', 'BLE(KNN)', 'UKF', 'PF'};
err_all   = {err_pdr, err_ble, err_ukf, err_pf};

for i = 1:4
    e = err_all{i};
    fprintf('%-10s %10.3f %10.3f %10.3f %10.3f\n', ...
        methods{i}, mean(e), median(e), std(e), max(e));
end

% 计算RMSE（均方根误差）
fprintf('\n%-10s %10s\n', '方法', 'RMSE(m)');
fprintf('--------------------\n');
for i = 1:4
    e = err_all{i};
    fprintf('%-10s %10.3f\n', methods{i}, sqrt(mean(e.^2)));
end
fprintf('--------------------\n\n');

%% 绘图
% 轨迹对比图（含AP位置）
figure;
% 绘制各方法轨迹
plot(x_true(1,:), x_true(2,:), 'k',   'LineWidth', 2,   'DisplayName', '轨迹真值');
hold on
plot(x_pdr(1,:),  x_pdr(2,:),  'r--', 'LineWidth', 1.2, 'DisplayName', 'PDR');
plot(pos_ble(1,:),pos_ble(2,:), 'c.',  'MarkerSize', 4,  'DisplayName', 'BLE KNN');
plot(ukf_est(1,:),  ukf_est(2,:),  'b',   'LineWidth', 1.5, 'DisplayName', 'UKF融合');
plot(pf_est(1,:), pf_est(2,:), 'g',   'LineWidth', 1.5, 'DisplayName', 'PF融合');
% 绘制AP位置（红色五角星标注）
h_ap = plot(AP(:,1), AP(:,2), 'r^', ...
    'MarkerSize', 10, 'MarkerFaceColor', 'red', 'DisplayName', 'AP');
% 在AP旁标注编号
for j = 1:nAP
    text(AP(j,1) + 0.3, AP(j,2) + 0.5, sprintf('AP%d', j), ...
        'FontSize', 8, 'Color', 'red', 'FontWeight', 'bold');
end
legend('Location', 'best');
xlabel('X轴 (m)');
ylabel('Y轴(m)');
title('轨迹对比');
axis equal


% XY位移对比图
figure;
% 绘制各方法轨迹
subplot(2,1,1);
plot(x_true(1,:), 'k',   'LineWidth', 2,   'DisplayName', '轨迹真值');
hold on
plot(x_pdr(1,:),  'r--', 'LineWidth', 1.2, 'DisplayName', 'PDR');
plot(pos_ble(1,:), 'c.',  'MarkerSize', 4,  'DisplayName', 'BLE KNN');
plot(ukf_est(1,:),  'b',   'LineWidth', 1.5, 'DisplayName', 'UKF融合');
plot(pf_est(1,:), 'g',   'LineWidth', 1.5, 'DisplayName', 'PF融合');
legend('Location', 'best');
xlabel('时间');
ylabel('X轴位移(m)');
subplot(2,1,2);
plot(x_true(2,:), 'k',   'LineWidth', 2,   'DisplayName', '轨迹真值');
hold on
plot(x_pdr(2,:),  'r--', 'LineWidth', 1.2, 'DisplayName', 'PDR');
plot(pos_ble(2,:), 'c.',  'MarkerSize', 4,  'DisplayName', 'BLE KNN');
plot(ukf_est(2,:),  'b',   'LineWidth', 1.5, 'DisplayName', 'UKF融合');
plot(pf_est(2,:), 'g',   'LineWidth', 1.5, 'DisplayName', 'PF融合');
legend('Location', 'best');
xlabel('时间');
ylabel('Y轴位移(m)');
sgtitle('各方法估计的XY轴位移对比（两个轴分别显示）');


% 逐步误差对比图
figure;
plot(err_pdr, 'r--', 'LineWidth', 1.2, 'DisplayName','PDR');
hold on
plot(err_ble, 'c',   'LineWidth', 1.0, 'DisplayName','BLE KNN');
plot(err_ukf, 'b',   'LineWidth', 1.5, 'DisplayName','UKF融合');
plot(err_pf,  'g',   'LineWidth', 1.5, 'DisplayName','PF融合');
legend('Location', 'best');
xlabel('仿真时间');
ylabel('位置误差 (m)');
title('位置误差曲线');


% 逐步误差对比图
figure;
subplot(2,1,1);
plot(x_pdr(1,:) - x_true(1,:), 'r--', 'LineWidth', 1.2, 'DisplayName','PDR');
hold on
plot(pos_ble(1,:) - x_true(1,:), 'c',   'LineWidth', 1.0, 'DisplayName','BLE KNN');
plot(ukf_est(1,:) - x_true(1,:), 'b',   'LineWidth', 1.5, 'DisplayName','UKF融合');
plot(pf_est(1,:) - x_true(1,:),  'g',   'LineWidth', 1.5, 'DisplayName','PF融合');
legend('Location', 'best');
xlabel('仿真时间');
ylabel('X轴位置误差 (m)');
subplot(2,1,2);
plot(x_pdr(2,:) - x_true(2,:), 'r--', 'LineWidth', 1.2, 'DisplayName','PDR');
hold on
plot(pos_ble(2,:) - x_true(2,:), 'c',   'LineWidth', 1.0, 'DisplayName','BLE KNN');
plot(ukf_est(2,:) - x_true(2,:), 'b',   'LineWidth', 1.5, 'DisplayName','UKF融合');
plot(pf_est(2,:) - x_true(2,:),  'g',   'LineWidth', 1.5, 'DisplayName','PF融合');
xlabel('仿真时间');
ylabel('Y轴位置误差 (m)');
sgtitle('XY轴误差曲线（分别显示）');

