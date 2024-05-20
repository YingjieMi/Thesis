% EKF_SLAM x, P Loss Simulation File
close all
clear
clc;

% Load simulation configuration parameters
simulation_config

% Add folders to the path
addpath('C:\Users\Microsoft微软\Desktop\Thesis\SLAM\EKF_SLAM-master\data');
addpath('C:\Users\Microsoft微软\Desktop\Thesis\SLAM\EKF_SLAM-master\functions');
addpath('C:\Users\Microsoft微软\Desktop\Thesis\SLAM\EKF_SLAM-master\pictures');

% Load map data
MAP_DATA = 'data/map3.mat';
load(MAP_DATA)

% Set variable parameters: packet loss rate and covariance scale
packet_loss_prob = 0.3;
scaleFactor = 1.05;             % Scale factor for covariance

% Load key points
key_points = data_original.key_points;
% Load landmarks
landmarks = data_original.landmarks;
% Load states
states = data_original.states;
% Load waypoints
wp = data_original.key_points;

% Get the length of the sequence
length = size(states, 2);

true_trajectory = zeros(3, length);
model_pre_trajectory = zeros(3, length);
EKF_pre_trajectory = zeros(3, length);
EKF_pre_trajectory_int = zeros(3, length);

% Initialize without intermittent
x = states(1).xtrue; % State vector
P = zeros(3); % Covariance matrix
QE = 2 * Q; % Covariance matrix for control variables
RE = 2 * R; % Sensor covariance matrix
ftag = 1:size(landmarks, 2);
da_table = zeros(1, size(landmarks, 2));
dt = DT_CONTROLS;

% Initialize with intermittent
x_int = states(1).xtrue; % State vector
P_int = zeros(3); % Covariance matrix
da_table_int = zeros(1, size(landmarks, 2));

x_model_pre = x;

if SLAM_SAVE_GIF == 1
    if exist('ekf_slam.avi', 'file') == 2
        delete('ekf_slam.avi');
    end

    if exist('ekf_slam.gif', 'file') == 2
        delete('ekf_slam.gif');
    end

    % Create AVI file object
    aviobj = VideoWriter('ekf_slam.avi', 'Uncompressed AVI');
    open(aviobj)
end

%%
% Loop for simulation
fig = figure;
hold on;

% Set the seed to 1 for reproducible random number sequence
rng(1); % 1

for k = 1:length
    % Get control inputs
    Vn = states(k).Vn;
    Gn = states(k).Gn;   
    % Store control inputs
    sim_result.controls(k).Vn = Vn;
    sim_result.controls(k).Gn = Gn;   

    % Randomly decide whether to lose packet
    if rand < packet_loss_prob && k ~= 1
        packetLost = 1;
    else
        packetLost = 0;
    end
    
    % Prediction phase
    % Without intermittent
    [x, P] = EKF_predict(x, P, Vn, Gn, QE, WHEELBASE, dt);
    % With intermittent
    [x_int, P_int] = EKF_predict(x_int, P_int, Vn, Gn, QE, WHEELBASE, dt);

    if states(k).observation_update == 1
        if packetLost == 0
            % Get observations
            z = states(k).zn;
            ftag_visible = states(k).ftag_visible;
        else
            P = P * scaleFactor;
            P_int = P_int * scaleFactor;
        end
        
        % Data association
        [zf, idf, zn] = data_associate(x, P, z, RE, GATE_REJECT, GATE_AUGMENT); 
        [zf_int, idf_int, zn_int] = data_associate(x_int, P_int, z, RE, GATE_REJECT, GATE_AUGMENT); 

        % ---------------------------------------------------------
        % Update state vector
        if SWITCH_USE_IEKF == 1 
            [x_int, P_int] = IEKF_update_Intermittent(x_int, P_int, zf_int, RE, idf_int, 3, ~packetLost);
            [x, P] = update_iekf(x, P, zf, RE, idf, 3);
        else
            [x_int, P_int] = EKF_update_Intermittent(x_int, P_int, zf_int, RE, idf_int, 1, ~packetLost);
            [x, P] = EKF_update(x, P, zf, RE, idf, 1); 
        end
        % ---------------------------------------------------------

        % Add new landmarks to the state vector
        [x, P] = augment(x, P, zn, RE); 
        [x_int, P_int] = augment(x_int, P_int, zn_int, RE);
    end
    
    xtrue = states(k).xtrue;
    iwp = states(k).next_keypoint;
    
    % Clear the plot
    cla;
    axis equal
   
    true_trajectory(:, k) = xtrue(1:3);
    EKF_pre_trajectory(:, k) = x(1:3);
    EKF_pre_trajectory_int(:, k) = x_int(1:3);
    
    % Plot the historical trajectory
    plot(true_trajectory(1, 1:k), true_trajectory(2, 1:k), 'k--', 'linewidth', 3);
    
    % Plot the historical EKF predicted trajectory
    plot(EKF_pre_trajectory(1, 1:k), EKF_pre_trajectory(2, 1:k), 'r', 'linewidth', 2);

    % Plot the historical intermittent EKF predicted trajectory
    plot(EKF_pre_trajectory_int(1, 1:k), EKF_pre_trajectory_int(2, 1:k), 'Color', [0.68, 0.85, 0.9], 'linewidth', 2);
    
    % Plot the landmarks
    scatter(landmarks(1, :), landmarks(2, :), 'b*');
    
    % Plot the waypoints
    plot(wp(1, :), wp(2, :), 'r.', 'markersize', 26);
    
    % Plot the position of the target point
    if iwp ~= 0
       plot(wp(1, iwp), wp(2, iwp), 'bo', 'markersize', 13, 'linewidth', 1);
    end
    
    % Plot the vehicle pose
    draw_car(xtrue, 5, 'k');
    
    % EKF predicted pose
    draw_car(x, 5, 'r');

    % Intermittent EKF predicted pose
    draw_car(x_int, 5, 'b');
    
    legend('true', 'ekf', 'int-ekf');

    pause(0.00000001)
    
    if SLAM_SAVE_GIF == 1
        % Get the current frame
        F = getframe(fig);
        % Add to AVI object
        writeVideo(aviobj, F);
        
        % Convert to GIF image, only 256 colors can be used
        im = frame2im(F);
        [I, map] = rgb2ind(im, 256);
        % Write to GIF89a format file   
        if k == 1
            imwrite(I, map, 'ekf_slam.gif', 'GIF', 'Loopcount', inf, 'DelayTime', 0.1);
        else
            imwrite(I, map, 'ekf_slam.gif', 'GIF', 'WriteMode', 'append', 'DelayTime', 0.1);
        end
    end 
    
    sim_result.states(k).xtrue = xtrue;
    sim_result.states(k).x_model_pre = x_model_pre;
    sim_result.states(k).x = x;
    sim_result.states(k).P = P;
end

%% Save results and data
sim_result.landmarks = landmarks;
sim_result.true_trajectory = true_trajectory;
sim_result.EKF_pre_trajectory = EKF_pre_trajectory;
sim_result.wp = wp;
save sim_result sim_result;

sim_result_int.landmarks = landmarks;
sim_result_int.true_trajectory = true_trajectory;
sim_result_int.EKF_pre_trajectory = EKF_pre_trajectory_int;
sim_result_int.wp = wp;
save sim_result_int sim_result_int;
