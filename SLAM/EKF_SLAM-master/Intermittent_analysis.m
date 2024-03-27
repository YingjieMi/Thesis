close all;
clear;
clc;

% 定义常量
dataset_name = 'm3';
specific_run = 'm3_p035';

% 原始随机丢包数据集
loss_data = load('SLAM_Error_Data.mat');
e_ekf = loss_data.(dataset_name).e_ekf.(specific_run);
ekf_eN2 = loss_data.(dataset_name).ekf_eN2.(specific_run);

% 修改KF算法的丢包数据集
intermittent_data = load('Int_SLAM_Error_Data.mat');
e_ekf_int = intermittent_data.(dataset_name).e_ekf.(specific_run);
ekf_int_eN2 = intermittent_data.(dataset_name).ekf_eN2.(specific_run);


length = size(e_ekf_int,2);

%% 创建误差图形
fig1 = figure;
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
title("Comparison of X, Y, and φ Errors Before and After Applying the Intermittent Algorithm");

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
fig2 = figure;
hold on
plot(ticks,ekf_eN2,'linewidth',2);
plot(ticks,ekf_int_eN2,'linewidth',2);
xlabel("ticks")
ylabel("error")
legend('lossy-ekf-error','ekf-error-int');
title("Comparison of Error Norm whether Applying the Intermittent Algorithm");


%% 箱型图
fig3 = figure;
% 将两个误差范数数据集组合到一个矩阵中
combined_errors = [ekf_eN2(:) ekf_int_eN2(:)];
% 创建一个用于标识数据集的组向量
group = [ones(size(ekf_eN2(:))); 2 * ones(size(ekf_int_eN2(:)))];
% 生成箱型图
boxplot(combined_errors, group, 'Labels', {'EKF without Intermittent', 'EKF with Intermittent'}, 'Whisker', 2);
title('Boxplot Comparison of Error Norms whether Applying the Intermittent Algorithm');
ylabel('Error Norm');
xlabel('EKF Implementation');
% 展示图形
hold off

%% 打印误差范数数据
% 计算统计数据
mean_error_norm = mean(ekf_eN2);
std_error_norm = std(ekf_eN2);
max_error_norm = max(ekf_eN2);

% % Output results
% fprintf('Before interruption:\n');
% fprintf('Average error norm: %f\n', mean_error_norm);
% fprintf('Standard deviation of error norm: %f\n', std_error_norm);
% fprintf('Maximum error norm: %f\n\n', max_error_norm);

% Calculate statistics
mean_error_norm_int = mean(ekf_int_eN2);
std_error_norm_int = std(ekf_int_eN2);
max_error_norm_int = max(ekf_int_eN2);

% % Output results
% fprintf('After interruption:\n');
% fprintf('Average error norm: %f\n', mean_error_norm_int);
% fprintf('Standard deviation of error norm: %f\n', std_error_norm_int);
% fprintf('Maximum error norm: %f\n', max_error_norm_int);

disp(' ');
disp('----------------------------------------------------------------------');
fprintf('|                          %s                                    |\n', specific_run);
disp('----------------------------------------------------------------------');
disp('|                          | Before Intermittent | After Intermittent |');
disp('|---------------------------------------------------------------------|');
fprintf('| Average Error Norm       |  %18.4f |  %17.4f |\n', mean_error_norm, mean_error_norm_int);
fprintf('| Error Norm Std Dev       |  %18.4f |  %17.4f |\n', std_error_norm, std_error_norm_int);
fprintf('| Maximum Error Norm       |  %18.4f |  %17.4f |\n', max_error_norm, max_error_norm_int);
disp('----------------------------------------------------------------------');


improvement_rate_mean = ((mean_error_norm - mean_error_norm_int) / mean_error_norm) * 100;
improvement_rate_std = ((std_error_norm - std_error_norm_int) / std_error_norm) * 100;
improvement_rate_max = ((max_error_norm - max_error_norm_int) / max_error_norm) * 100;

disp(['Average error norm improvement rate: ', num2str(improvement_rate_mean), '%']);
disp(['Error norm standard deviation improvement rate: ', num2str(improvement_rate_std), '%']);
disp(['Maximum error norm improvement rate: ', num2str(improvement_rate_max), '%']);

%% 保存数据
% % 定义保存图片和文本的文件夹路径
% saveFolderPath = fullfile('C:\Users\Microsoft微软\Desktop\Thesis\SLAM\EKF_SLAM-master\pictrues\Intermittent algorithm comparison\m2\', 'p03');
% if ~exist(saveFolderPath, 'dir')
%     mkdir(saveFolderPath); % 如果文件夹不存在，则创建文件夹
% end
% 
% % 定义保存图形的文件名（基于特定运行）
% fig1Filename = fullfile(saveFolderPath, 'Comparison_X_Y_phi_Errors.fig');
% fig2Filename = fullfile(saveFolderPath, 'Comparison_Error_Norm.fig');
% fig3Filename = fullfile(saveFolderPath, 'Boxplot_Comparison_Error_Norms.fig');
% 
% % 保存图形
% savefig(fig1, fig1Filename);
% savefig(fig2, fig2Filename);
% savefig(fig3, fig3Filename);
% 
% % 定义保存文本信息的文件名
% textFilename = fullfile(saveFolderPath, 'Error_Statistics.txt');
% 
% % 打开文件用于写入
% fileID = fopen(textFilename, 'w');
% 
% % 写入文本信息
% fprintf(fileID, '----------------------------------------------------------------------\n');
% fprintf(fileID, '|                                 %s                                    |\n', specific_run);
% fprintf(fileID, '----------------------------------------------------------------------\n');
% fprintf(fileID, '|                                 | Before Intermittent | After Intermittent |\n');
% fprintf(fileID, '|---------------------------------------------------------------------|\n');
% fprintf(fileID, '| Average Error Norm      |  %18.4f |  %17.4f |\n', mean_error_norm, mean_error_norm_int);
% fprintf(fileID, '| Error Norm Std Dev       |  %18.4f |  %17.4f |\n', std_error_norm, std_error_norm_int);
% fprintf(fileID, '| Maximum Error Norm   |  %18.4f |  %17.4f |\n', max_error_norm, max_error_norm_int);
% fprintf(fileID, '----------------------------------------------------------------------\n\n');
% 
% fprintf(fileID, 'Average error norm improvement rate: %0.2f%%\n', improvement_rate_mean);
% fprintf(fileID, 'Error norm standard deviation improvement rate: %0.2f%%\n', improvement_rate_std);
% fprintf(fileID, 'Maximum error norm improvement rate: %0.2f%%\n', improvement_rate_max);
% 
% % 关闭文件
% fclose(fileID);
