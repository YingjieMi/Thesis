close all
clear

% Addpath
addpath('C:\Users\Microsoft΢��\Desktop\SLAM\EKF_SLAM-master\functions\EKF_SLAM_core_functions\');
addpath('C:\Users\Microsoft΢��\Desktop\SLAM\EKF_SLAM-master\functions\data_gen_functions\');
addpath('C:\Users\Microsoft΢��\Desktop\SLAM\EKF_SLAM-master\functions\draw_functions\');
addpath('C:\Users\Microsoft΢��\Desktop\SLAM\EKF_SLAM-master\functions\models\');

% Add loss result
load('sim_result.mat')              % δ�Ķ��㷨����
% load('sim_result_int.mat')          % �ж��㷨����
EKF_pre_trajectory = sim_result.EKF_pre_trajectory;

% Add original result
load('sim_result_original.mat')
EKF_pre_trajectory_original = sim_result_original.EKF_pre_trajectory;

% Assessment
true_trajectory = sim_result.ture_trajectory;
true_trajectory_original = sim_result_original.ture_trajectory;
model_pre_trajectory = sim_result.model_pre_trajectory;

length = size(EKF_pre_trajectory,2);


% ����EKF��ģ��Ԥ������
e_ekf = EKF_pre_trajectory - true_trajectory;
e_ekf(3,:) = pi_to_pi(e_ekf(3,:));
e_ekf_original = EKF_pre_trajectory_original - true_trajectory_original;
e_ekf_original(3,:) = pi_to_pi(e_ekf_original(3,:));

% ȡ�������
e_ekf = abs(e_ekf);
e_ekf_original = abs(e_ekf_original);

% �������ͼ��
fig2 = figure;
hold on
ticks = 1:1:length;

% ����x��������
subplot(3,1,1)
hold on
plot(ticks,e_ekf(1,:),'linewidth',2);
plot(ticks,e_ekf_original(1,:),'linewidth',2);
xlabel("ticks")
ylabel("|x-error|")
legend('lossy-ekf-error','ekf-error');

% ����y��������
subplot(3,1,2)
hold on
plot(ticks,e_ekf(2,:),'linewidth',2);
plot(ticks,e_ekf_original(2,:),'linewidth',2);
xlabel("ticks")
ylabel("|y-error|")

% ���ƽǶȵ����
subplot(3,1,3)
hold on
plot(ticks,pi_to_pi(e_ekf(3,:)),'linewidth',2);
plot(ticks,pi_to_pi(e_ekf_original(3,:)),'linewidth',2);
xlabel("ticks")
ylabel("|\phi-error|")

% ��������
ekf_eN2 = zeros(1,length);
ekf_original_eN2 = zeros(1,length);
for i=1:1:length
    ekf_eN2(i) = norm(e_ekf(:,i),2);
    ekf_original_eN2(i) = norm(e_ekf_original(:,i),2);
end
hold off

%% ���������
fig3 = figure;
hold on
plot(ticks,ekf_eN2,'linewidth',2);
plot(ticks,ekf_original_eN2,'linewidth',2);
xlabel("ticks")
ylabel("error")
legend('lossy-ekf-error','ekf-error');

%% ��������
% ���屣�����Ļ����ļ���
% ԭʼ����������ļ���
base_filename = 'SLAM_Error_Data.mat';

% �޸�KF�㷨�Ķ������ļ���
% base_filename = 'Int_SLAM_Error_Data.mat';

% ����һ�����ص��ļ�����ʶ
dataname = 'm2_p01';  % ÿ������ʱ��Ҫȷ����������Ƕ��ص�
prefix = dataname(1:2);  % ��ȡǰ׺ "m3"

% ����ļ��Ƿ����
if exist(base_filename, 'file')
    % �������е������ļ�
    loaded_data = load(base_filename);
else
    % ����ļ������ڣ����ʼ��һ���յĽṹ��
    loaded_data = struct();
end

% ����Ƿ��Ѿ����˸�ǰ׺�ķ��࣬���û���򴴽�
if ~isfield(loaded_data, prefix)
    loaded_data.(prefix) = struct('e_ekf', struct(), 'ekf_eN2', struct());
end

% ���浱ǰ���е��������
loaded_data.(prefix).e_ekf.(dataname) = e_ekf;
loaded_data.(prefix).ekf_eN2.(dataname) = ekf_eN2;

% ������º������
save(base_filename, '-struct', 'loaded_data');
