% ���������ļ�
% 
% ������ 2019

% �������ɿ��Ʋ���
N_LANDMARKS = 60; % �������landmark����Ŀ
BORDER_LENGTH = 30; % �������landmark�ı߽��С��Χ
SAVE_GIF = 0; % 0������gif������1����gif����
NUMBER_LOOPS= 1; % �ܵ�Ȧ��
AT_WAYPOINT= 0.5; % �ж��л���һ��ľ���
SWITCH_CONTROL_NOISE = 1; % �Ƿ���ӿ�������
SWITCH_SENSOR_NOISE = 1; % �Ƿ���ӹ۲�����
ADD_COLOR_NOISE = 0;
% c = [1 0.8 -0.6 0.5 -0.4 0.2];
c = [1 0.8 -0.6];

% �����ļ����Ʋ���
SLAM_SAVE_GIF = 1; % 0������gif������1����gif����
MAP_DATA = 'map3.mat';
REDUCE_OB_FEATURES = 0;
ASYNCHRONOUS = 0; % �Ƿ�ͬ��
SWITCH_ASSOCIATION_KNOWN= 0; % id�Ƿ���֪
GATE_REJECT= 4.0; % �ж�Ϊ��֪������������
GATE_AUGMENT= 25.0; % �ж�Ϊ����������С����
SWITCH_USE_IEKF = 1;

% ���Ʋ���
V= 8; % m/s���ٶ�
MAXG= 30*pi/180; % rad�����ķ����
RATEG= 20*pi/180; % rad/s, ���ת������
WHEELBASE= 4; % metres, �־�
DT_CONTROLS= 0.025; % ����Ƶ��

% ������������
sigmaV= 2; % m/s
sigmaG= (10.0*pi/180); % radians
Q= [sigmaV^2 0; 0 sigmaG^2];

% �۲����
MAX_RANGE= 30.0; % metres ���۲����
DT_OBSERVE= 8*DT_CONTROLS; % seconds, �۲�Ƶ��

% �۲�����
sigmaR= 0.1; % metres
sigmaB= (1.0*pi/180); % radians
R= [sigmaR^2 0; 0 sigmaB^2];

