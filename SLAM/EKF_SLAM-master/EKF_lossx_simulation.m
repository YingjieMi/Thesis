% EKF_SLAM x,P 丢失 仿真文件
close all
clear
clc;

% 加载模型参数
simulation_config

% 添加文件夹
addpath('C:\Users\Microsoft微软\Desktop\Thesis\SLAM\EKF_SLAM-master\data');
addpath('C:\Users\Microsoft微软\Desktop\Thesis\SLAM\EKF_SLAM-master\functions');
addpath('C:\Users\Microsoft微软\Desktop\Thesis\SLAM\EKF_SLAM-master\pictrues');

% 加载数据
MAP_DATA = 'data/map3.mat';
load(MAP_DATA)

% 设置可变参数：丢包率和协方差scale
packet_loss_prob = 0.35;
scaleFactor = 1.05;        %1.05
    
% 加载关键点
key_points = data_original.key_points;
% 加载路标
landmarks = data_original.landmarks;
% 加载状态
states = data_original.states;
% 加载路径关键点
wp = data_original.key_points;

% 获取序列长度
length = size(states,2);

true_trajectory = zeros(3,length);
model_pre_trajectory = zeros(3,length);
EKF_pre_trajectory = zeros(3,length);
EKF_pre_trajectory_int = zeros(3,length);

% 不使用中断初始化
x= states(1).xtrue; % 状态向量
P= zeros(3); % 协方差矩阵
QE= 2*Q; % 控制变量的协方差矩阵
RE= 2*R; % 传感器协方差矩阵
ftag= 1:size(landmarks,2);
da_table= zeros(1,size(landmarks,2));
dt = DT_CONTROLS;

% 使用中断初始化
x_int= states(1).xtrue; % 状态向量
P_int= zeros(3); % 协方差矩阵
da_table_int= zeros(1,size(landmarks,2));

x_model_pre = x;

if SLAM_SAVE_GIF == 1

    if exist('ekf_slam.avi','file') == 2
        delete('ekf_slam.avi');
    end
    
    if exist('ekf_slam.gif','file') == 2
        delete('ekf_slam.gif');
    end
    
    %创建avi文件对象
    aviobj = VideoWriter('ekf_slam.avi','Uncompressed AVI');
    open(aviobj)
end

%%
% 循环算法仿真
fig = figure;
hold on;

% 设置种子为1以获得可复现的随机数序列
rng(1); %1

for k = 1:1:length
    % 获取控制量
    Vn = states(k).Vn;
    Gn = states(k).Gn;   
    % 存储控制量
    sim_result.controls(k).Vn = Vn;
    sim_result.controls(k).Gn = Gn;   

    % 随机决定是否丢包
    if rand < packet_loss_prob && k ~= 1
        packetLost = 1;
    else
        packetLost = 0;
    end
    
    % 预测阶段
    % 不使用中断
    [x,P] = EKF_predict (x,P, Vn,Gn,QE, WHEELBASE,dt);
    % 使用中断
    [x_int,P_int] = EKF_predict (x_int,P_int, Vn,Gn,QE, WHEELBASE,dt);

    if states(k).observation_update == 1
        if packetLost == 0
            % 获取观测值
            z = states(k).zn;
            ftag_visible = states(k).ftag_visible;
        else
            P = P * scaleFactor;
            P_int = P_int * scaleFactor;
        end
        
        % 数据关联
        [zf,idf, zn]= data_associate(x,P,z,RE, GATE_REJECT, GATE_AUGMENT); 
        [zf_int,idf_int, zn_int]= data_associate(x_int,P_int,z,RE, GATE_REJECT, GATE_AUGMENT); 

        
        % ---------------------------------------------------------
        % 更新状态向量
        if SWITCH_USE_IEKF == 1 
            [x_int,P_int]= IEKF_update_Intermittent(x_int,P_int,zf_int,RE,idf_int, 3, ~packetLost);
            [x,P]= update_iekf(x,P,zf,RE,idf, 3);
        else
            [x_int,P_int]= EKF_update_Intermittent(x_int,P_int,zf_int,RE,idf_int, 1, ~packetLost);
            [x,P]= EKF_update(x,P,zf,RE,idf, 1); 
        end
        % ---------------------------------------------------------

        % 添加新的landmark到状态向量中
        [x,P]= augment(x,P, zn,RE); 
        [x_int,P_int]= augment(x_int,P_int, zn_int,RE);
    end
    
    xtrue = states(k).xtrue;
    iwp = states(k).next_keypoint;
    
    % 清除图像
    cla;
    axis equal
   
    true_trajectory(:,k) = xtrue(1:3);
    EKF_pre_trajectory(:,k) = x(1:3);
    EKF_pre_trajectory_int(:,k) = x_int(1:3);
    
    % 画出历史轨迹
    plot( true_trajectory(1, 1:k), true_trajectory(2, 1:k), 'k--','linewidth',3);
    
    % 画出历史EKF预测轨迹
    plot( EKF_pre_trajectory(1, 1:k), EKF_pre_trajectory(2, 1:k), 'r','linewidth',2 );

    % 画出历史中断EKF预测轨迹
    plot( EKF_pre_trajectory_int(1, 1:k), EKF_pre_trajectory_int(2, 1:k), 'Color', [0.68, 0.85, 0.9],'linewidth',2 );
    
    % 画出landmarks
    scatter( landmarks(1, :), landmarks(2, :), 'b*' );
    
    % 画出路径关键点
    plot( wp(1,:), wp(2, :), 'r.','markersize',26 );
    
    % 画出目标点的位置
    if iwp~=0
       plot(wp(1,iwp),wp(2,iwp),'bo','markersize',13,'linewidth',1);
    end
    
    % 画出车的位姿
    draw_car(xtrue,5,'k');
    
    % EKF预测位姿
    draw_car(x,5,'r');

    % EKF预测位姿
    draw_car(x_int,5,'b');
    
    legend('true','ekf','int-ekf');

    pause(0.00000001)
    
    if SLAM_SAVE_GIF == 1
        %获取当前画面
        F = getframe(fig);
        %加入avi对象中
        writeVideo(aviobj,F);
        
        %转成gif图片,只能用256色
        im = frame2im(F);
        [I,map] = rgb2ind(im,256);
        %写入 GIF89a 格式文件   
        if k == 1
            imwrite(I,map,'ekf_slam.gif','GIF', 'Loopcount',inf,'DelayTime',0.1);
        else
            imwrite(I,map,'ekf_slam.gif','GIF','WriteMode','append','DelayTime',0.1);
        end
    end 
    
    sim_result.states(k).xtrue = xtrue;
    sim_result.states(k).x_model_pre = x_model_pre;
    sim_result.states(k).x = x;
    sim_result.states(k).P = P;
    
end


%% Save results and data
sim_result.landmarks = landmarks;
sim_result.ture_trajectory = true_trajectory;
sim_result.EKF_pre_trajectory = EKF_pre_trajectory;
sim_result.wp = wp;
save sim_result sim_result;

sim_result_int.landmarks = landmarks;
sim_result_int.ture_trajectory = true_trajectory;
sim_result_int.EKF_pre_trajectory = EKF_pre_trajectory_int;
sim_result_int.wp = wp;
save sim_result_int sim_result_int;