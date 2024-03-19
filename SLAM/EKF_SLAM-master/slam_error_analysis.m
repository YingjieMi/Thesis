% 这个文件用来对比同一个地图的不同丢包概率下 EKF SLAM的误差
clc;
clear;
close all;

% 原始随机丢包数据集
loaded_data = load('SLAM_Error_Data.mat');

% 修改KF算法的丢包数据集
% loaded_data = load('Int_SLAM_Error_Data.mat');

% 选择一个特定的数据集前缀，例如 'm3'
dataset_prefix = 'm3';

% 获取数据集中所有运行的名字
runs = fieldnames(loaded_data.(dataset_prefix).e_ekf);
num_run = length(runs);

% 获取所有运行的名字并升序排序
[sorted_runs, sortIdx] = sort(runs); % sortIdx是排序后的索引

% 准备绘图
fig1 = figure;
subplot(3, 1, 1);
hold on;

% 遍历每个运行，绘制 x 方向的误差
for i = 1:num_run
    run_name = runs{i};
    e_ekf = loaded_data.(dataset_prefix).e_ekf.(run_name);
    ekf_eN2 = loaded_data.(dataset_prefix).ekf_eN2.(run_name);

    % 假设 ticks 是一个与 e_ekf 中数据量相同长度的向量
    ticks = 1:length(e_ekf(1, :));

    plot(ticks, abs(e_ekf(1, :)), 'linewidth', 2);
end

% 设置图例
title("EKF SLAM x, y, φ error in different packet loss");
xlabel("ticks");
ylabel("|x-error|");
legend(runs, 'Location', 'northeast'); % 显示图例，每个run的名字
hold off; % 结束x方向的误差图的绘制

% 继续绘制y方向和角度的误差对比图，接下来是y方向的误差对比图
subplot(3, 1, 2);
hold on;
for i = 1:num_run
    run_name = runs{i};
    e_ekf = loaded_data.(dataset_prefix).e_ekf.(run_name);
    plot(ticks, abs(e_ekf(2, :)), 'linewidth', 2);
end
xlabel("ticks");
ylabel("|y-error|");
hold off; % 结束y方向的误差图的绘制

% 接下来是角度误差的对比图
subplot(3, 1, 3);
hold on;
for i = 1:num_run
    run_name = runs{i};
    e_ekf = loaded_data.(dataset_prefix).e_ekf.(run_name);
    plot(ticks, wrapToPi(e_ekf(3, :)), 'linewidth', 2);
end
xlabel("ticks");
ylabel("|\phi-error|");
hold off; % 结束角度误差图的绘制

% 如果您还想绘制 ekf_eN2 的误差范数图，您可以创建另一个图形窗口并绘制
fig2 = figure;
hold on;

% 创建一个矩阵来存储所有运行的误差范数
ekf_eN2_matrix = [];

for i = 1:num_run
    run_name = runs{i};
    ekf_eN2 = loaded_data.(dataset_prefix).ekf_eN2.(run_name);
    ekf_eN2_matrix(:, i) = ekf_eN2;  % 假设ekf_eN2是列向量
    plot(ticks, ekf_eN2, 'linewidth', 2);
end
xlabel("ticks");
ylabel("error norm");
title("EKF SLAM Error Norm for different packet loss");
legend(runs); % 显示图例，每个run的名字
hold off; % 结束误差范数图的绘制

%% 创建升序排列的箱型图

% 根据排序索引重新排列误差范数矩阵
sorted_ekf_eN2_matrix = ekf_eN2_matrix(:, sortIdx);
figure;
boxplot(sorted_ekf_eN2_matrix, 'Labels', sorted_runs);
title('EKF SLAM Error Norm box plot for Different Packet Loss Rates (Sorted)');
ylabel('Error Norm');
xlabel('Packet Loss Rate Runs (Sorted)');


