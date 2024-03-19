close all
clear

% Addpath
addpath('C:\Users\Microsoft微软\Desktop\SLAM\EKF_SLAM-master\functions\EKF_SLAM_core_functions\');
addpath('C:\Users\Microsoft微软\Desktop\SLAM\EKF_SLAM-master\functions\data_gen_functions\');
addpath('C:\Users\Microsoft微软\Desktop\SLAM\EKF_SLAM-master\functions\draw_functions\');
addpath('C:\Users\Microsoft微软\Desktop\SLAM\EKF_SLAM-master\functions\models\');

% Add loss result
load('sim_result.mat')              % 未改动算法数据
% load('sim_result_int.mat')          % 中断算法数据
EKF_pre_trajectory = sim_result.EKF_pre_trajectory;

% Add original result
load('sim_result_original.mat')
EKF_pre_trajectory_original = sim_result_original.EKF_pre_trajectory;

% Assessment
true_trajectory = sim_result.ture_trajectory;
true_trajectory_original = sim_result_original.ture_trajectory;
model_pre_trajectory = sim_result.model_pre_trajectory;

length = size(EKF_pre_trajectory,2);


% 计算EKF和模型预测的误差
e_ekf = EKF_pre_trajectory - true_trajectory;
e_ekf(3,:) = pi_to_pi(e_ekf(3,:));
e_ekf_original = EKF_pre_trajectory_original - true_trajectory_original;
e_ekf_original(3,:) = pi_to_pi(e_ekf_original(3,:));

% 取绝对误差
e_ekf = abs(e_ekf);
e_ekf_original = abs(e_ekf_original);

% 创建误差图形
fig2 = figure;
hold on
ticks = 1:1:length;

% 绘制x方向的误差
subplot(3,1,1)
hold on
plot(ticks,e_ekf(1,:),'linewidth',2);
plot(ticks,e_ekf_original(1,:),'linewidth',2);
xlabel("ticks")
ylabel("|x-error|")
legend('lossy-ekf-error','ekf-error');

% 绘制y方向的误差
subplot(3,1,2)
hold on
plot(ticks,e_ekf(2,:),'linewidth',2);
plot(ticks,e_ekf_original(2,:),'linewidth',2);
xlabel("ticks")
ylabel("|y-error|")

% 绘制角度的误差
subplot(3,1,3)
hold on
plot(ticks,pi_to_pi(e_ekf(3,:)),'linewidth',2);
plot(ticks,pi_to_pi(e_ekf_original(3,:)),'linewidth',2);
xlabel("ticks")
ylabel("|\phi-error|")

% 计算误差范数
ekf_eN2 = zeros(1,length);
ekf_original_eN2 = zeros(1,length);
for i=1:1:length
    ekf_eN2(i) = norm(e_ekf(:,i),2);
    ekf_original_eN2(i) = norm(e_ekf_original(:,i),2);
end
hold off

%% 绘制总误差
fig3 = figure;
hold on
plot(ticks,ekf_eN2,'linewidth',2);
plot(ticks,ekf_original_eN2,'linewidth',2);
xlabel("ticks")
ylabel("error")
legend('lossy-ekf-error','ekf-error');

%% 保存数据
% 定义保存结果的基本文件名
% 原始随机丢包的文件名
base_filename = 'SLAM_Error_Data.mat';

% 修改KF算法的丢包的文件名
% base_filename = 'Int_SLAM_Error_Data.mat';

% 创建一个独特的文件名标识
dataname = 'm2_p01';  % 每次运行时需要确保这个名称是独特的
prefix = dataname(1:2);  % 提取前缀 "m3"

% 检查文件是否存在
if exist(base_filename, 'file')
    % 加载现有的数据文件
    loaded_data = load(base_filename);
else
    % 如果文件不存在，则初始化一个空的结构体
    loaded_data = struct();
end

% 检查是否已经有了该前缀的分类，如果没有则创建
if ~isfield(loaded_data, prefix)
    loaded_data.(prefix) = struct('e_ekf', struct(), 'ekf_eN2', struct());
end

% 保存当前运行的误差数据
loaded_data.(prefix).e_ekf.(dataname) = e_ekf;
loaded_data.(prefix).ekf_eN2.(dataname) = ekf_eN2;

% 保存更新后的数据
save(base_filename, '-struct', 'loaded_data');
