clc;
clear;
close all;

% 加载数据集
int_data = load('Int_SLAM_Error_Data.mat');
orig_data = load('SLAM_Error_Data.mat');

% 选择一个特定的数据集前缀，例如 'm3'
dataset_prefix = 'm3';

% 这里是修正后的运行名数组
common_runs = {'m3_p005', 'm3_p01', 'm3_p02', 'm3_p03', 'm3_p04'};

% 初始化误差中位数数组和丢包率数组
int_median_errors = zeros(1, length(common_runs));
orig_median_errors = zeros(1, length(common_runs));
packet_loss_rates = [5, 10, 20, 30, 40];

% 遍历每个共有的运行
for i = 1:length(common_runs)
    run_name = common_runs{i};
    
    % 检查并计算每个运行的误差中位数
    if isfield(orig_data.(dataset_prefix).ekf_eN2, run_name)
        orig_median_errors(i) = median(orig_data.(dataset_prefix).ekf_eN2.(run_name));
    else
        warning('Run %s does not exist in the original dataset.', run_name);
    end
    
    if isfield(int_data.(dataset_prefix).ekf_eN2, run_name)
        int_median_errors(i) = median(int_data.(dataset_prefix).ekf_eN2.(run_name));
    else
        warning('Run %s does not exist in the intermittent dataset.', run_name);
    end
end

% 绘制误差中位数的折线图
figure;
plot(packet_loss_rates, orig_median_errors, 'o-', 'DisplayName', 'Original');
hold on;
plot(packet_loss_rates, int_median_errors, 's-', 'DisplayName', 'Intermittent');
hold off;

% 添加图例、标题和轴标签
legend('show','Location','northwest');
title('EKF SLAM Error Median Comparison');
xlabel('Packet Loss Rate (%)');
ylabel('Error Median (m)');
