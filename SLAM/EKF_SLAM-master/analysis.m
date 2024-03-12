close all
clear

% 添加文件夹
addpath('C:\Users\UchihaYJ\Desktop\SLAM\EKF_SLAM-master\functions\EKF_SLAM_core_functions\');
addpath('C:\Users\UchihaYJ\Desktop\SLAM\EKF_SLAM-master\functions\data_gen_functions\');
addpath('C:\Users\UchihaYJ\Desktop\SLAM\EKF_SLAM-master\functions\draw_functions\');
addpath('C:\Users\UchihaYJ\Desktop\SLAM\EKF_SLAM-master\functions\models\');

% load('sim_result_iekf_map3.mat')
load('sim_result.mat')
IEKF_pre_trajectory = sim_result.EKF_pre_trajectory;

% load('sim_result_color_noise_map3.mat')
EKF_pre_trajectory_color = sim_result.EKF_pre_trajectory;

% load('sim_result_asy_map3.mat')
EKF_pre_trajectory_asy = sim_result.EKF_pre_trajectory;

% load('sim_result_less_landmark_map3.mat')
EKF_pre_trajectory_ls = sim_result.EKF_pre_trajectory;

% load('sim_result_ekf_map3.mat')
% load('sim_result_less_landmark_map3.mat')
% load('sim_result_color_noise_map3.mat')
ture_trajectory = sim_result.ture_trajectory;
model_pre_trajectory = sim_result.model_pre_trajectory;
EKF_pre_trajectory = sim_result.EKF_pre_trajectory;
landmarks = sim_result.landmarks;
wp = sim_result.wp;

length = size(ture_trajectory,2);

figure
axis equal
hold on;
% 画出landmarks
scatter( landmarks(1, :), landmarks(2, :), 'b*' )

% 画出历史轨迹
truep = plot( ture_trajectory(1, :), ture_trajectory(2, :), 'k--','linewidth',3);
    
% 画出历史EKF预测轨迹
ekfp = plot( EKF_pre_trajectory(1, :), EKF_pre_trajectory(2, :), 'r','linewidth',3 );
    
% 画出历史model预测轨迹
modelp = plot( model_pre_trajectory(1, :), model_pre_trajectory(2, :), 'b-.','linewidth',3 );

% 画出车的位姿
xtrue = ture_trajectory(:,length-1);
draw_car(xtrue,5,'k');
    
% EKF预测位姿
x = EKF_pre_trajectory(:,length-1);
draw_car(x,5,'r');
    
% 模型预测位姿
x_model_pre = model_pre_trajectory(:,length-1);
draw_car(x_model_pre,5,'g');

%图例
legend([truep ekfp,modelp],'true','ekf','model');

% 保持图形
hold off;

% 计算EKF和模型预测的误差
e_ekf = EKF_pre_trajectory - ture_trajectory;
e_ekf(3,:) = pi_to_pi(e_ekf(3,:));
e_model = model_pre_trajectory - ture_trajectory;
e_model(3,:) = pi_to_pi(e_model(3,:));

% 取绝对误差
e_ekf = abs(e_ekf);
e_model = abs(e_model);

% 创建误差图形
fig2 = figure;
hold on
ticks = 1:1:length;

% 绘制x方向的误差
subplot(3,1,1)
hold on
plot(ticks,e_ekf(1,:),'linewidth',2);
plot(ticks,e_model(1,:),'linewidth',2);
xlabel("ticks")
ylabel("|x-error|")
legend('ekf-error','model-error');

% 绘制y方向的误差
subplot(3,1,2)
hold on
plot(ticks,e_ekf(2,:),'linewidth',2);
plot(ticks,e_model(2,:),'linewidth',2);
xlabel("ticks")
ylabel("|y-error|")

% 绘制角度的误差
subplot(3,1,3)
hold on
plot(ticks,pi_to_pi(e_ekf(3,:)),'linewidth',2);
plot(ticks,pi_to_pi(e_model(3,:)),'linewidth',2);
xlabel("ticks")
ylabel("|\phi-error|")

% 计算误差范数
ekf_eN2 = zeros(1,length);
model_eN2 = zeros(1,length);
for i=1:1:length
    ekf_eN2(i) = norm(e_ekf(:,i),2);
    model_eN2(i) = norm(e_model(:,i),2);
end
hold off

% 绘制总误差
fig3 = figure;
hold on
plot(ticks,ekf_eN2,'linewidth',2);
plot(ticks,model_eN2,'linewidth',2);
xlabel("ticks")
ylabel("error")
legend('ekf-error','model-error');   