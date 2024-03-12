% EKF_SLAM 仿真文件
close all
clear
clc;

% 加载模型参数
simulation_config

% 添加文件夹
addpath('C:\Users\Microsoft微软\Desktop\SLAM\EKF_SLAM-master\data');
addpath('C:\Users\Microsoft微软\Desktop\SLAM\EKF_SLAM-master\functions');
addpath('C:\Users\Microsoft微软\Desktop\SLAM\EKF_SLAM-master\pictrues');

% 加载数据
MAP_DATA = 'data/map3.mat';
load(MAP_DATA)
    
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

if ADD_COLOR_NOISE == 1
    noise_V = gen_color_noise(length,Q(1,1),c);
    noise_G = gen_color_noise(length,Q(2,2),c);
%     noise_r = gen_color_noise(length,R(1,1));
%     noise_t = gen_color_noise(length,R(2,2));
    for i = 1:1:length
        states(i).Vn = states(i).V + noise_V(i);
        states(i).Gn = states(i).G + noise_G(i);
%         states(i).Vn = states(i).V + noise_r(i);
%         states(i).Vn = states(i).V + noise_t(i);
    end
end

true_trajectory = zeros(3,length);
model_pre_trajectory = zeros(3,length);
EKF_pre_trajectory = zeros(3,length);

x= states(1).xtrue; % 状态向量
P= zeros(3); % 协方差矩阵
QE= 2*Q; % 控制变量的协方差矩阵
RE= 2*R; % 传感器协方差矩阵
ftag= 1:size(landmarks,2);
da_table= zeros(1,size(landmarks,2));
dt = DT_CONTROLS;

if ASYNCHRONOUS == 1
    dt = DT_OBSERVE;
end

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

% 循环算法仿真
fig = figure;
hold on;

% 初始化数据存储数组
dataToSave = zeros(length, 3); % 假设length是迭代次数

% 设置 概率丢失通信
packet_loss_prob = 0.2;
last_x = 0;
last_P = 0;
last_Vn = 0;
last_Gn = 0;

for k = 1:1:length
    % 随机决定是否丢包
    if rand >= packet_loss_prob || k == 1 % 不丢包
        % 获取控制量
        Vn = states(k).Vn;
        Gn = states(k).Gn;
        packetLost = 0; % 未丢包
    else  % 丢包
        % 使用上一次的状态和协方差
        x = last_x;
        P = last_P;
        % 可能需要设置一个默认的控制量
        Vn = last_Vn; % 使用上一次的控制量
        Gn = last_Gn; % 使用上一次的控制量
        packetLost = 1; % 丢包
    end

    last_Vn = Vn;
    last_Gn = Gn;
     
    if ASYNCHRONOUS == 0
        % EKF更新状态预测值和协方差
        [x,P] = EKF_predict (x,P, Vn,Gn,QE, WHEELBASE,dt);
        % 获取仅通过模型预测的位姿
        x_model_pre = vehicle_model(x_model_pre, Vn,Gn, WHEELBASE,dt);
    end
    

    if states(k).observation_update == 1
        
        if ASYNCHRONOUS == 1
            % EKF更新状态预测值和协方差
            [x,P] = EKF_predict (x,P, Vn,Gn,QE, WHEELBASE,dt);
            % 获取仅通过模型预测的位姿
            x_model_pre = vehicle_model(x_model_pre, Vn,Gn, WHEELBASE,dt);
        end
        % 获取观测值
        z = states(k).zn;
        ftag_visible = states(k).ftag_visible;
        
        if REDUCE_OB_FEATURES == 1
            % 削减观测到的landmark数目
            if size(z,2) > 1
                z = z(:,1);
                ftag_visible = ftag_visible(1);
            end
        end
        
        % 数据关联
        if SWITCH_ASSOCIATION_KNOWN == 1
            [zf,idf,zn, da_table]= data_associate_known(x,z,ftag_visible, da_table);
        else
            [zf,idf, zn]= data_associate(x,P,z,RE, GATE_REJECT, GATE_AUGMENT); 
        end
        
        % 更新状态向量
        if SWITCH_USE_IEKF == 1
            [x,P]= update_iekf(x,P,zf,RE,idf, 5);
        else
            [x,P]= EKF_update(x,P,zf,RE,idf, 1); 
        end
        
        % 添加新的landmark到状态向量中
        [x,P]= augment(x,P, zn,RE); 
    end

    % 保存当前状态和协方差，以便下一次循环使用
    last_x = x;
    last_P = P;

    % 保存数据
    traceP = trace(P); % 计算P的迹
    detP = det(P); % 计算P的行列式
    dataToSave(k, :) = [packetLost, traceP, detP];
    
    xtrue = states(k).xtrue;
    iwp = states(k).next_keypoint;
    
    % 清除图像
    cla;
    axis equal
   
    true_trajectory(:,k) = xtrue(1:3);
    model_pre_trajectory(:,k) = x_model_pre(1:3);
    EKF_pre_trajectory(:,k) = x(1:3);
    
    % 画出历史轨迹
    plot( true_trajectory(1, 1:k), true_trajectory(2, 1:k), 'k--','linewidth',3);
    
    % 画出历史EKF预测轨迹
    plot( EKF_pre_trajectory(1, 1:k), EKF_pre_trajectory(2, 1:k), 'r','linewidth',3 );
    
    % 画出历史model预测轨迹
    plot( model_pre_trajectory(1, 1:k), model_pre_trajectory(2, 1:k), 'b-.','linewidth',3);
    
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
    
    % 模型预测位姿
    draw_car(x_model_pre,5,'g');

    % 画出激光雷达观测范围
    % draw_circle(xtrue(1), xtrue(2),MAX_RANGE);

    if ~isempty(z)
        % 画出激光雷达观测线
        plines = make_laser_lines(z,xtrue);
        % plot(plines(1,:),plines(2,:));
        
%         pellipses = make_covariance_ellipses(x,P);
%         plot(pellipses(1,:),pellipses(2,:));
    end
    
    % legend([truep ekfp,modelp],'true','ekf','model');
    legend('true','ekf','model');

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

sim_result.landmarks = landmarks;
sim_result.ture_trajectory = true_trajectory;
sim_result.EKF_pre_trajectory = EKF_pre_trajectory;
sim_result.model_pre_trajectory = model_pre_trajectory;
sim_result.wp = wp;

save sim_result sim_result;

% 将数据保存到 Excel 文件中
% 注意：如果 P 是矩阵，你可能需要将它转换为适合 Excel 单元格的格式
% xlswrite('packet_loss_and_covariance.xlsx', dataToSave);