close all;
clear;
clc;

% 定义常量
dataset_name = 'm2';
specific_run = 'm2_p005';

% 原始随机丢包数据集
loss_data = load('SLAM_Error_Data.mat');
e_ekf = loss_data.(dataset_name).e_ekf.(specific_run);
ekf_eN2 = loss_data.(dataset_name).ekf_eN2.(specific_run);

% 修改KF算法的丢包数据集
intermittent_data = load('Int_SLAM_Error_Data.mat');
e_ekf_int = intermittent_data.(dataset_name).e_ekf.(specific_run);
ekf_int_eN2 = intermittent_data.(dataset_name).ekf_eN2.(specific_run);


length = size(e_ekf_int,2);

% 创建误差图形
fig2 = figure;
hold on
ticks = 1:1:length;

% 绘制x方向的误差
subplot(3,1,1)
hold on
plot(ticks,e_ekf(1,:),'linewidth',2);
plot(ticks,e_ekf_int(1,:),'linewidth',2);
xlabel("ticks")
ylabel("|x-error|")
legend('lossy-ekf-error','ekf-error-int');

% 绘制y方向的误差
subplot(3,1,2)
hold on
plot(ticks,e_ekf(2,:),'linewidth',2);
plot(ticks,e_ekf_int(2,:),'linewidth',2);
xlabel("ticks")
ylabel("|y-error|")

% 绘制角度的误差
subplot(3,1,3)
hold on
plot(ticks,pi_to_pi(e_ekf(3,:)),'linewidth',2);
plot(ticks,pi_to_pi(e_ekf_int(3,:)),'linewidth',2);
xlabel("ticks")
ylabel("|\phi-error|")
hold off

%% 绘制总误差
fig3 = figure;
hold on
plot(ticks,ekf_eN2,'linewidth',2);
plot(ticks,ekf_int_eN2,'linewidth',2);
xlabel("ticks")
ylabel("error")
legend('lossy-ekf-error','ekf-error-int');


%% 箱型图
fig4 = figure;
% 将两个误差范数数据集组合到一个矩阵中
combined_errors = [ekf_eN2(:) ekf_int_eN2(:)];
% 创建一个用于标识数据集的组向量
group = [ones(size(ekf_eN2(:))); 2 * ones(size(ekf_int_eN2(:)))];
% 生成箱型图
boxplot(combined_errors, group, 'Labels', {'EKF without Intermittent', 'EKF with Intermittent'}, 'Whisker', 2);
title('Boxplot Comparison of EKF Error Norms');
ylabel('Error Norm');
xlabel('EKF Implementation');
% 展示图形
hold off


% 计算误差范数的差分，来观察误差是如何快速减少的
ekf_error_diff = diff(ekf_eN2);
ekf_int_error_diff = diff(ekf_int_eN2);

% 可视化误差范数的变化速率
fig5 = figure;
plot(ticks(1:end-1), abs(ekf_error_diff), 'LineWidth', 2);
hold on;
plot(ticks(1:end-1), abs(ekf_int_error_diff), 'LineWidth', 2);
xlabel('Time Steps');
ylabel('Change in Error Norm');
title('Convergence Speed of EKF Error Norms');
legend('EKF without Intermittent', 'EKF with Intermittent');
grid on;

%% 打印误差范数数据
% 计算统计数据
mean_error_norm = mean(ekf_eN2);
std_error_norm = std(ekf_eN2);
max_error_norm = max(ekf_eN2);
mean_change_rate = mean(abs(diff(ekf_eN2)));

% 输出结果
fprintf('平均误差范数: %f\n', mean_error_norm);
fprintf('误差范数标准差: %f\n', std_error_norm);
fprintf('最大误差范数: %f\n', max_error_norm);
fprintf('误差范数变化的平均速度: %f\n\n', mean_change_rate);


% 计算统计数据
mean_error_norm_int = mean(ekf_int_eN2);
std_error_norm_int = std(ekf_int_eN2);
max_error_norm_int = max(ekf_int_eN2);
mean_change_rate_int = mean(abs(diff(ekf_int_eN2)));

% 输出结果
fprintf('平均误差范数: %f\n', mean_error_norm_int);
fprintf('误差范数标准差: %f\n', std_error_norm_int);
fprintf('最大误差范数: %f\n', max_error_norm_int);
fprintf('误差范数变化的平均速度: %f\n', mean_change_rate_int);

