close all;
clear;
clc;

original = load('sim_result_original_3.mat');
lossy = load('sim_result.mat');
intermit = load('sim_result_int.mat');

specific_run = '30%';

EKF_trajectory = lossy.sim_result.EKF_pre_trajectory;
int_trajectory = intermit.sim_result_int.EKF_pre_trajectory;
ground_truth = original.sim_result.ture_trajectory;

% 仅使用x和y坐标（假设第一行是x，第二行是y）
error_distances_EKF = sqrt(sum((ground_truth(1:2, :) - EKF_trajectory(1:2, :)).^2, 1));
error_distances_int = sqrt(sum((ground_truth(1:2, :) - int_trajectory(1:2, :)).^2, 1));

% 计算ATE
ate_ekf = sqrt(mean(error_distances_EKF.^2));
ate_int = sqrt(mean(error_distances_int.^2));

fprintf('The EKF ATE for run %s is %.3f meters.\n', specific_run, ate_ekf);
fprintf('The intermittent ATE for run %s is %.3f meters.\n', specific_run, ate_int);

% 可视化误差
figure;
plot(error_distances_EKF, 'b-', 'LineWidth', 2); % 使用蓝色实线
hold on; % 保持图像，继续在同一图上绘图
plot(error_distances_int, 'r--', 'LineWidth', 2); % 使用红色虚线
title(sprintf('Error Distance Over Time for %s loss rate', specific_run));
xlabel('Time (Index)');
ylabel('Error Distance (meters)');
legend('EKF ATE', 'Intermittent ATE', 'Location', 'northwest'); % 正确添加图例
grid on;
