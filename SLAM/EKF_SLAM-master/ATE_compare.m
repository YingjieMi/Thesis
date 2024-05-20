% ATE data plot
close all
clear
clc;


% 数据
loss_rates = [5, 10, 20, 30, 40];
ate_no_intermittent = [0.658, 0.69, 1.609, 5.322, 9.006];
ate_with_intermittent = [0.614, 0.621, 0.814, 0.961, 3.566];

% 绘制折线图
figure;
plot(loss_rates, ate_no_intermittent, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 5);
hold on;
plot(loss_rates, ate_with_intermittent, 'r-s', 'LineWidth', 1.5, 'MarkerSize', 5);
hold off;

% 图表设置
xlabel('Pocket loss rate (%)');
ylabel('Absolute Trajectory Error (m)');
title('ATE Comparison between normal EKF and intermitent EKF');
legend('Normal EKF', 'Intermittent EKF', 'Location', 'northwest');
