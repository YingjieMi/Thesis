% EKF_SLAM x,P ��ʧ �����ļ�
close all
clear
clc;

% ����ģ�Ͳ���
simulation_config

% ����ļ���
addpath('C:\Users\Microsoft΢��\Desktop\Thesis\SLAM\EKF_SLAM-master\data');
addpath('C:\Users\Microsoft΢��\Desktop\Thesis\SLAM\EKF_SLAM-master\functions');
addpath('C:\Users\Microsoft΢��\Desktop\Thesis\SLAM\EKF_SLAM-master\pictrues');

% ��������
MAP_DATA = 'data/map3.mat';
load(MAP_DATA)

% ���ÿɱ�����������ʺ�Э����scale
packet_loss_prob = 0.35;
scaleFactor = 1.05;        %1.05
    
% ���عؼ���
key_points = data_original.key_points;
% ����·��
landmarks = data_original.landmarks;
% ����״̬
states = data_original.states;
% ����·���ؼ���
wp = data_original.key_points;

% ��ȡ���г���
length = size(states,2);

true_trajectory = zeros(3,length);
model_pre_trajectory = zeros(3,length);
EKF_pre_trajectory = zeros(3,length);
EKF_pre_trajectory_int = zeros(3,length);

% ��ʹ���жϳ�ʼ��
x= states(1).xtrue; % ״̬����
P= zeros(3); % Э�������
QE= 2*Q; % ���Ʊ�����Э�������
RE= 2*R; % ������Э�������
ftag= 1:size(landmarks,2);
da_table= zeros(1,size(landmarks,2));
dt = DT_CONTROLS;

% ʹ���жϳ�ʼ��
x_int= states(1).xtrue; % ״̬����
P_int= zeros(3); % Э�������
da_table_int= zeros(1,size(landmarks,2));

x_model_pre = x;

if SLAM_SAVE_GIF == 1

    if exist('ekf_slam.avi','file') == 2
        delete('ekf_slam.avi');
    end
    
    if exist('ekf_slam.gif','file') == 2
        delete('ekf_slam.gif');
    end
    
    %����avi�ļ�����
    aviobj = VideoWriter('ekf_slam.avi','Uncompressed AVI');
    open(aviobj)
end

%%
% ѭ���㷨����
fig = figure;
hold on;

% ��������Ϊ1�Ի�ÿɸ��ֵ����������
rng(1); %1

for k = 1:1:length
    % ��ȡ������
    Vn = states(k).Vn;
    Gn = states(k).Gn;   
    % �洢������
    sim_result.controls(k).Vn = Vn;
    sim_result.controls(k).Gn = Gn;   

    % ��������Ƿ񶪰�
    if rand < packet_loss_prob && k ~= 1
        packetLost = 1;
    else
        packetLost = 0;
    end
    
    % Ԥ��׶�
    % ��ʹ���ж�
    [x,P] = EKF_predict (x,P, Vn,Gn,QE, WHEELBASE,dt);
    % ʹ���ж�
    [x_int,P_int] = EKF_predict (x_int,P_int, Vn,Gn,QE, WHEELBASE,dt);

    if states(k).observation_update == 1
        if packetLost == 0
            % ��ȡ�۲�ֵ
            z = states(k).zn;
            ftag_visible = states(k).ftag_visible;
        else
            P = P * scaleFactor;
            P_int = P_int * scaleFactor;
        end
        
        % ���ݹ���
        [zf,idf, zn]= data_associate(x,P,z,RE, GATE_REJECT, GATE_AUGMENT); 
        [zf_int,idf_int, zn_int]= data_associate(x_int,P_int,z,RE, GATE_REJECT, GATE_AUGMENT); 

        
        % ---------------------------------------------------------
        % ����״̬����
        if SWITCH_USE_IEKF == 1 
            [x_int,P_int]= IEKF_update_Intermittent(x_int,P_int,zf_int,RE,idf_int, 3, ~packetLost);
            [x,P]= update_iekf(x,P,zf,RE,idf, 3);
        else
            [x_int,P_int]= EKF_update_Intermittent(x_int,P_int,zf_int,RE,idf_int, 1, ~packetLost);
            [x,P]= EKF_update(x,P,zf,RE,idf, 1); 
        end
        % ---------------------------------------------------------

        % ����µ�landmark��״̬������
        [x,P]= augment(x,P, zn,RE); 
        [x_int,P_int]= augment(x_int,P_int, zn_int,RE);
    end
    
    xtrue = states(k).xtrue;
    iwp = states(k).next_keypoint;
    
    % ���ͼ��
    cla;
    axis equal
   
    true_trajectory(:,k) = xtrue(1:3);
    EKF_pre_trajectory(:,k) = x(1:3);
    EKF_pre_trajectory_int(:,k) = x_int(1:3);
    
    % ������ʷ�켣
    plot( true_trajectory(1, 1:k), true_trajectory(2, 1:k), 'k--','linewidth',3);
    
    % ������ʷEKFԤ��켣
    plot( EKF_pre_trajectory(1, 1:k), EKF_pre_trajectory(2, 1:k), 'r','linewidth',2 );

    % ������ʷ�ж�EKFԤ��켣
    plot( EKF_pre_trajectory_int(1, 1:k), EKF_pre_trajectory_int(2, 1:k), 'Color', [0.68, 0.85, 0.9],'linewidth',2 );
    
    % ����landmarks
    scatter( landmarks(1, :), landmarks(2, :), 'b*' );
    
    % ����·���ؼ���
    plot( wp(1,:), wp(2, :), 'r.','markersize',26 );
    
    % ����Ŀ����λ��
    if iwp~=0
       plot(wp(1,iwp),wp(2,iwp),'bo','markersize',13,'linewidth',1);
    end
    
    % ��������λ��
    draw_car(xtrue,5,'k');
    
    % EKFԤ��λ��
    draw_car(x,5,'r');

    % EKFԤ��λ��
    draw_car(x_int,5,'b');
    
    legend('true','ekf','int-ekf');

    pause(0.00000001)
    
    if SLAM_SAVE_GIF == 1
        %��ȡ��ǰ����
        F = getframe(fig);
        %����avi������
        writeVideo(aviobj,F);
        
        %ת��gifͼƬ,ֻ����256ɫ
        im = frame2im(F);
        [I,map] = rgb2ind(im,256);
        %д�� GIF89a ��ʽ�ļ�   
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