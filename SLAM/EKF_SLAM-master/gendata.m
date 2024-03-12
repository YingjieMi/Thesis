function data_original = gendata( do_vis )
%
% ���ڲ���EKF_SLAM��������Ҫ�����ݡ�
%
% Inputs:
%   do_vis - �Ƿ���п��ӻ�
%
% Outputs: 
%   data_original - ���������ݣ����������
%       key_points - ��ͼ�ؼ���
%       lamdmarks - landmark������
%       states.G - ��ʵ���
%       states.V - ��ʵ�ٶ�
%       states.Gn - ���������Ķ��
%       states.Vn - �����������ٶ�
%       states.xtrue - ��ʵ��λ��
%       states.ftag_visible - �ɹ۲⵽��landmark
%       states.ztrue - ��ʵ��landmark�۲�ֵ
%       states.zn - ����������landmark�۲�ֵ
%       states.observation_update - �Ƿ�����˹۲����ݸ���
%       states.next_keypoint - ��һĿ���
%
% ������ 2019

% �������
clear all

% ����ģ�Ͳ���
simulation_config

% �޲�����Ĭ�Ͽ��ӻ�
if nargin < 1
    do_vis = 1;
end

% �����˶��켣�Ĺؼ���
t = 0:0.8:6.28;
key_points = gen_trajectory2d( t );
data_original.key_points = key_points;

% ����landmarks
n_landmarks = N_LANDMARKS;
border = [BORDER_LENGTH, BORDER_LENGTH];
minpos = min((key_points'));
maxpos = max((key_points'));
minlm = minpos - border;
maxlm = maxpos + border;
landmarks(1, :) = minlm(1)+rand(n_landmarks, 1)*(maxlm(1)-minlm(1));
landmarks(2, :) = minlm(2)+rand(n_landmarks, 1)*(maxlm(2)-minlm(2));

data_original.landmarks = landmarks;

% ����Ƶ��
dt = DT_CONTROLS;
% ��ʼĿ��ؼ���
iwp= 1;
% ��ʼ���
G= 0;
% ��ʼλ��
xtrue = [key_points(:,1);135*pi/180];

true_states = zeros(5,1000000);
wp = key_points;
dtsum= DT_OBSERVE+0.0001;
ftag= 1:size(landmarks,2);

if SAVE_GIF == 1

    if exist('gen_data.avi','file') == 2
        delete(gen_data.avi);
    end
    
    if exist('gen_data.gif','file') == 2
        delete(gen_data.avi);
    end
    
    %����avi�ļ�����
    aviobj = VideoWriter('gen_data.avi','Uncompressed AVI');
    open(aviobj)
end



fig=figure;
i = 1;
k = 1;
while iwp ~= 0
    
    % ������ʵ���
    [G,iwp]= compute_steering(xtrue, wp, iwp, AT_WAYPOINT, G, RATEG, MAXG, dt);
    
    % �ж��Ƿ���������Ȧ��
    if iwp==0 && NUMBER_LOOPS > 1 
        iwp=1; 
        NUMBER_LOOPS= NUMBER_LOOPS-1; 
    end 
    
    % ���³�����ʵλ��
    xtrue= vehicle_model(xtrue, V,G, WHEELBASE,dt);
    
    % ���²ɼ����Ŀ�����
    [Vn,Gn]= add_control_noise(V,G,Q, SWITCH_CONTROL_NOISE);
    
    ob = 0;
    %��ʵlandmark�۲�ֵ
    dtsum= dtsum + dt;
    if dtsum >= DT_OBSERVE
        dtsum= 0;
        % �����Χlandmark
        [z,ftag_visible]= get_observations(xtrue, landmarks, ftag, MAX_RANGE);
        ztrue = z;
        % ��ӹ۲�����
        z = add_observation_noise(z,R, SWITCH_SENSOR_NOISE);
        
        % ��ʵ����
        true_state = [xtrue;V;G];
        true_states(:,i) = true_state;

        ob = 1;
        
        
        i = i + 1;   
    end
    
    % �ɼ�����
    state.G = G;
    state.V = V;
    state.xtrue = xtrue;
    state.Vn = Vn;
    state.Gn = Gn;
    state.ztrue = ztrue;
    state.ftag_visible = ftag_visible;
    state.zn = z;
    state.observation_update = ob;
    state.next_keypoint = iwp;
    data_original.states(k) = state;
    
    % ���ӻ�����
    if do_vis == 1
        % ���ͼ��
        cla;
        hold on;
        scatter( landmarks(1, :), landmarks(2, :), 'b*' ); 
        axis equal;
        plot( wp(1,:), wp(2, :), 'r.','markersize',26 );
    
        % ��������λ��
        draw_car(xtrue,5,'k');
    
        % ������һĿ����λ��
        if iwp~=0
            plot(wp(1,iwp),wp(2,iwp),'bo','markersize',13,'linewidth',1);
        end
    
        % ���������״�۲ⷶΧ
        draw_circle(xtrue(1), xtrue(2),MAX_RANGE);
    
        % ���������״�۲���
        plines = make_laser_lines(z,xtrue);
        if  ~isempty(plines)
            plot(plines(1,:),plines(2,:));
        end
    
        % ������ʷ�켣
        plot( true_states(1, 1:i-1), true_states(2, 1:i-1), 'k--','linewidth',3 );
 
        pause(0.00001); 
        
        if SAVE_GIF == 1
            %��ȡ��ǰ����
            F = getframe(fig);
            %����avi������
            writeVideo(aviobj,F);
            
            %ת��gifͼƬ,ֻ����256ɫ
            im = frame2im(F);
            [I,map] = rgb2ind(im,256);
            %д�� GIF89a ��ʽ�ļ�   
            if k == 1
                imwrite(I,map,'gen_data.gif','GIF', 'Loopcount',inf,'DelayTime',0.1);
            else
                imwrite(I,map,'gen_data.gif','GIF','WriteMode','append','DelayTime',0.1);
            end
        end
        
        k = k+1;
    end
end
save data data_original 