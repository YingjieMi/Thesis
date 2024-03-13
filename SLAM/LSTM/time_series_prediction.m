% 关闭所有图形，清除变量和控制台
close all;
clear;
clc;

% 加载数据
data = load('sim_result.mat');

% 假设位姿数据和控制数据已经提前准备好
pose = data.sim_result.EKF_pre_trajectory'; % 位姿数据为 [x, y, theta]
control = zeros(numel(data.sim_result.controls), 2); % 初始化控制信号数组

% 转换控制信号为矩阵形式
for i = 1:numel(data.sim_result.controls)
    control(i, :) = [data.sim_result.controls(i).Vn, data.sim_result.controls(i).Gn];
end

% 准备数据
num_samples = size(pose, 1); % 总样本数
kim = 10; % 时间步数
data_dim = size(pose, 2) + size(control, 2); % 每个时间点的特征数

% 预分配空间
X = zeros(data_dim, kim, num_samples-kim);
Y = zeros(size(pose, 2), num_samples-kim); % 假设我们只预测位姿的下一个时间点

for i = 1:num_samples-kim
    X(:, :, i) = [pose(i:i+kim-1, :), control(i:i+kim-1, :)]'; % 转置以匹配 [特征数, 时间步]
    Y(:, i) = pose(i+kim, :); % 下一个时间点的位姿
end

%% 划分训练集和测试集
% 计算训练集和测试集的分割点
split_point = floor(0.8 * (num_samples - kim));

% 直接根据索引顺序划分训练集和测试集
trainIdx = 1:split_point;
testIdx = split_point+1:num_samples - kim;

% 提取训练集和测试集的数据
XTrain = X(:, :, trainIdx);
YTrain = Y(:, trainIdx)';
XTest = X(:, :, testIdx);
YTest = Y(:, testIdx);

% 归一化处理（可选）
numFeatures = size(XTrain, 1);
numTimeStepsTrain = size(XTrain, 2);
numSamplesTrain = size(XTrain, 3);
numTimeStepsTest = size(XTest, 2);
numSamplesTest = size(XTest, 3);

% 重塑数据以适配 mapminmax
XTrainReshaped = reshape(XTrain, numFeatures, []);
XTestReshaped = reshape(XTest, numFeatures, []);

% 应用归一化
[XTrainNorm, PS] = mapminmax(XTrainReshaped);
XTestNorm = mapminmax('apply', XTestReshaped, PS);

% 数据重塑以适配 LSTM 网络
XTrain = reshape(XTrainNorm, numFeatures, numTimeStepsTrain, numSamplesTrain);
XTest = reshape(XTestNorm, numFeatures, numTimeStepsTest, numSamplesTest);


% 假设 XTrain 和 XTest 的形状是 [特征数, 时间步长, 样本数]
numSamplesTrain = size(XTrain, 3);
numSamplesTest = size(XTest, 3);

% 将 XTrain 和 XTest 转换为单元数组
XTrainCell = cell(1, numSamplesTrain);
for i = 1:numSamplesTrain
    XTrainCell{i} = squeeze(XTrain(:, :, i));
end

XTestCell = cell(1, numSamplesTest);
for i = 1:numSamplesTest
    XTestCell{i} = squeeze(XTest(:, :, i));
end


%% 创建 LSTM 网络结构
layers = [
    sequenceInputLayer(numFeatures)
    lstmLayer(20, 'OutputMode', 'sequence')
    reluLayer                            % Relu 激活层
    fullyConnectedLayer(2)
    regressionLayer
];

% 训练选项
options = trainingOptions('adam', ...
    'MaxEpochs',100, ...
    'MiniBatchSize', 20, ...
    'InitialLearnRate', 0.01, ...
    'GradientThreshold', 1, ...
    'Shuffle', 'never', ...
    'Verbose', 0, ...
    'Plots', 'training-progress');

% 训练 LSTM 网络
net = trainNetwork(XTrainCell, YTrain, layers, options);

% 进行预测
YPred = predict(net, XTest);

%% 显示预测结果和实际结果
figure;
subplot(3,1,1);
plot(YTest(:,1));
hold on;
plot(YPred(:,1)');
ylabel('X');
title('Prediction vs. Actual');

subplot(3,1,2);
plot(YTest(:,2));
hold on;
plot(YPred(:,2)');
ylabel('Y');

subplot(3,1,3);
plot(YTest(:,3));
hold on;
plot(YPred(:,3)');
ylabel('Theta');
xlabel('Time Step');
