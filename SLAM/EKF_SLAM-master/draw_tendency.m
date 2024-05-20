clc;
close all;
clear;

% Assuming the Excel file is in the current directory, replace with full path if not
filename = 'C:\Users\Microsoft微软\Desktop\improvement.xlsx'; 

% Read the specified range from the Excel file
data = readtable(filename);

% Extract Pocket Loss Rate and Improvement Rates
pocket_loss_rate = data{1:2:height(data), 'PocketLossRate'};
avg_error_norm_improvement_rate = data{1:2:height(data), 'AverageErrorNormImprovementRate'};
error_norm_std_dev_improvement_rate = data{1:2:height(data), 'ErrorNormStandardDeviationImprovementRate'};
max_error_norm_improvement_rate = data{1:2:height(data), 'MaximumErrorNormImprovementRate'};

% Create the plots
figure;
hold on; % Hold on to the current figure

plot(pocket_loss_rate, avg_error_norm_improvement_rate, 'o-', 'DisplayName', 'Average Error Norm Improvement Rate');
plot(pocket_loss_rate, error_norm_std_dev_improvement_rate, 's-', 'DisplayName', 'Error Norm Standard Deviation Improvement Rate');
plot(pocket_loss_rate, max_error_norm_improvement_rate, 'd-', 'DisplayName', 'Maximum Error Norm Improvement Rate');

% Customize the plot
xlabel('Pocket Loss Rate');
ylabel('Improvement Rate');
title('Improvement Rates in increasing Pocket Loss Rate');
legend('Location', 'southeast');
grid on; % Turn on grid

% 设置坐标轴标签为百分比
% 获取当前刻度
xticks = get(gca, 'XTick');
yticks = get(gca, 'YTick');
% 设置新的刻度标签
set(gca, 'XTickLabel', arrayfun(@(v) sprintf('%.0f%%', v*100), xticks, 'UniformOutput', false));
set(gca, 'YTickLabel', arrayfun(@(v) sprintf('%.0f%%', v*100), yticks, 'UniformOutput', false));

hold off; % Release the figure

