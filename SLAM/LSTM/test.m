clear; clc;

% 加载和准备数据
data = load('sim_result.mat');
pose = data.sim_result.EKF_pre_trajectory';  % 假设位姿数据为 [x, y, theta]
control = zeros(numel(data.sim_result.controls), 2); % 初始化控制信号数组
for i = 1:numel(data.sim_result.controls)
    control(i, :) = [data.sim_result.controls(i).Vn, data.sim_result.controls(i).Gn];
end

% 组织数据为序列
num_samples = size(pose, 1);
kim = 10;  % 历史步数
num_features = size(pose, 2) + size(control, 2); % 特征数量
X = [];
Y = [];
for i = 1:num_samples - kim - 1  % 确保有足够的数据用于预测
    X{i} = [pose(i:i+kim-1, :), control(i:i+kim-1, :)]';
    Y{i} = pose(i+kim, :)';  % 预测下一个时间步的位姿
end

YMatrix = zeros(length(Y), 3); % 假设有3个预测目标维度

for i = 1:length(Y)
    YMatrix(i, :) = Y{i}';
end


% 定义网络结构
layers = [
    sequenceInputLayer(num_features)
    lstmLayer(100, 'OutputMode', 'last')
    fullyConnectedLayer(3)  % 假设位姿维度为3
    regressionLayer
];


% 训练选项
options = trainingOptions('adam', ...
    'MaxEpochs',100, ...
    'MiniBatchSize', 20, ...
    'Shuffle','never', ...
    'Plots','training-progress');

% 训练网络
net = trainNetwork(X, YMatrix, layers, options);

% 使用网络进行预测
YPred = predict(net, X);
