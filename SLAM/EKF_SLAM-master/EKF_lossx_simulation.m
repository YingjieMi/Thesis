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
MAP_DATA = 'data/map2.mat';
load(MAP_DATA)

% 设置可变参数：丢包率和协方差scale
packet_loss_prob = 0.1;
set_scaleFactor = 1.00;        %1.05
    
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

%%
% 循环算法仿真
fig = figure;
hold on;

% 初始化数据存储数组
dataToSave = zeros(length, 3); % 假设length是迭代次数

last_x = 0;
last_P = 0;

% 初始化FIFO队列
historySize = 10; % 定义历史记录的大小
arraySize = size(x);
n = arraySize(1);
historyX = zeros(n, historySize); % 用于x的FIFO队列
historyX(:,1) = states(1).xtrue;
historyV = zeros(1, historySize); % 用于v的FIFO队列
historyV(1) = states(1).Vn;
historyG = zeros(1, historySize); % 用于g的FIFO队列
historyG(1) = states(1).Gn;


for k = 1:1:length
    % 获取控制量
    Vn = states(k).Vn;
    Gn = states(k).Gn;   
    % 存储控制量
    sim_result.controls(k).Vn = Vn;
    sim_result.controls(k).Gn = Gn;
     
    if ASYNCHRONOUS == 0
        % EKF更新状态预测值和协方差
        [x,P] = EKF_predict (x,P, Vn,Gn,QE, WHEELBASE,dt);
        % 获取仅通过模型预测的位姿
        x_model_pre = vehicle_model(x_model_pre, Vn,Gn, WHEELBASE,dt);
    end
    
    % 随机决定是否丢包
    if rand < packet_loss_prob && k ~= 1 % 丢包
        % 使用上一次的状态和协方差
        % 增加协方差
        scaleFactor = set_scaleFactor;
        x = last_x;
        P = last_P * scaleFactor;
        packetLost = 1; % 丢包
    else
        packetLost = 0; % 不丢包
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


    %
    % 如果检测为丢包，则使用加权平均
    %
    % 当丢包发生时，使用FIFO队列中的数据
    if packetLost == 1
        % 计算历史数据的平均值
        % 初始化avgX为零数组，大小与historyX的行数相同
        avgX = zeros(size(historyX, 1), 1);
        
        % 对于historyX的每一行（即每一个变量或维度），单独计算非零平均值
        for i = 1:size(historyX, 1)
            % 提取当前行的所有值
            currentRowValues = historyX(i, :);
    
            % 生成一个逻辑索引，标识哪些值是非零的
            validXIndices = currentRowValues ~= 0;
    
            % 使用逻辑索引计算非零值的平均值
            if sum(validXIndices) > 0 % 确保分母不为零
                avgX(i) = sum(currentRowValues(validXIndices)) / sum(validXIndices);
            else
                avgX(i) = 0; % 如果当前行全为零，则平均值为零
            end
        end
        % 
        % 
        % 
        % 对于historyV和historyG，因为它们是1D数组，处理稍有不同
        validVIndices = historyV ~= 0;
        avgV = sum(historyV(validVIndices)) / sum(validVIndices);
    
        validGIndices = historyG ~= 0;
        avgG = sum(historyG(validGIndices)) / sum(validGIndices);
    
        % 计算基于avgV的最大预期位移范围
        maxDisplacement = avgV * 5; % 假设5个时刻
    
        % 计算预测位姿与平均位姿之间的实际位移
        actualDisplacement = sqrt((x(1) - avgX(1))^2 + (x(2) - avgX(2))^2);
    
        % % 判断实际位移是否超出最大预期位移范围
        % if actualDisplacement > maxDisplacement
        %     % 如果超出范围，认为预测是错误的，使用currentX作为当前位姿
        %     x = predictCurrentPose(avgX, avgV, avgG);
        %     disp(k);
        % end
    end

    % 更新FIFO队列
    historyX = [historyX(:,2:end), x(1:3)]; % 更新x的FIFO队列
    historyV = [historyV(2:end), Vn]; % 更新v的FIFO队列
    historyG = [historyG(2:end), Gn]; % 更新g的FIFO队列


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


%% Save results and data
sim_result.landmarks = landmarks;
sim_result.ture_trajectory = true_trajectory;
sim_result.EKF_pre_trajectory = EKF_pre_trajectory;
sim_result.model_pre_trajectory = model_pre_trajectory;
sim_result.wp = wp;

save sim_result sim_result;

% 将数据保存到 Excel 文件中
% 注意：如果 P 是矩阵，你可能需要将它转换为适合 Excel 单元格的格式
% xlswrite('packet_loss_and_covariance.xlsx', dataToSave);