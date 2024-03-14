% 关闭所有图形，清除变量和控制台
close all;
clear;
clc;

% 加载数据
data = load('sim_result_m2.mat');

% 位姿数据和控制数据准备
pose = data.sim_result.EKF_pre_trajectory';  % 位姿数据为 [x, y, theta]
control = zeros(numel(data.sim_result.controls), 2);  % 初始化控制信号数组

% 转换控制信号为矩阵形式
for i = 1:numel(data.sim_result.controls)
    control(i, :) = [data.sim_result.controls(i).Vn, data.sim_result.controls(i).Gn];
end

% 准备数据
num_samples = size(pose, 1);  % 总样本数
kim = 10;  % 时间步数
num_features = size(pose, 2) + size(control, 2);  % 每个时间点的特征数

% 归一化处理
poseMin = min(pose);
poseMax = max(pose);
controlMin = min(control);
controlMax = max(control);

poseNorm = (pose - poseMin) ./ (poseMax - poseMin);
controlNorm = (control - controlMin) ./ (controlMax - controlMin);

% 重塑数据以匹配之前的格式
X = [];
Y = [];
for i = 1:num_samples - kim - 1
    X{i} = [poseNorm(i:i+kim-1, :), controlNorm(i:i+kim-1, :)]';
    Y{i} = poseNorm(i+kim, :)';  % 预测下一个时间步的位姿
end

% 将Y转换为矩阵形式
YMatrix = zeros(length(Y), 3); % 假设有3个预测目标维度
for i = 1:length(Y)
    YMatrix(i, :) = Y{i}';
end

%% 数据划分比例
trainRatio = 0.8;
splitIdx = floor(trainRatio * length(X));  % 计算训练集和测试集的分界线

% 划分训练集和测试集
XTrain = X(1:splitIdx);
YTrain = YMatrix(1:splitIdx, :);
XTest = X(splitIdx+1:end);
YTest = YMatrix(splitIdx+1:end, :);

% 独立的验证集数据
numObservations = size(XTrain, 2);
numValidationSamples = floor(0.1 * numObservations); % 10%作为验证集

% 打乱数据
idx = randperm(numObservations);
XTrainShuffled = XTrain(:, idx);
YTrainShuffled = YTrain(idx, :);

% 划分验证集
XVal = XTrainShuffled(:, 1:numValidationSamples);
YVal = YTrainShuffled(1:numValidationSamples, :);

% 更新训练集，移除用作验证集的部分
XTrain = XTrainShuffled(:, numValidationSamples+1:end);
YTrain = YTrainShuffled(numValidationSamples+1:end, :);

%% 创建 LSTM 网络结构，适应你的预测目标
layers = [
    sequenceInputLayer(num_features)
    lstmLayer(150, 'OutputMode', 'last')                    % 150
    reluLayer                            % Relu 激活层
    % dropoutLayer(0.2) % 添加了Dropout层                   % 0
    fullyConnectedLayer(3)    
    regressionLayer
];

% 训练选项，与之前保持一致
options = trainingOptions('adam', ...
    'MaxEpochs', 100, ...                                   % 100
    'MiniBatchSize', 20, ...                                % 20
    'InitialLearnRate', 0.0008, ...                         % 0.0008
    'GradientThreshold', 1, ...
    'Shuffle', 'every-epoch', ... 
    'Verbose', true, ...
    'Plots', 'training-progress', ...
    'L2Regularization', 0.000, ...                          % 0.000
    'ValidationData', {XVal, YVal}, ...
    'ValidationFrequency', 30, ...
    'ValidationPatience', 5, ...
    'ExecutionEnvironment', 'auto');

% 训练网络
net = trainNetwork(XTrain, YTrain, layers, options);

%% 进行预测
YPred = predict(net, XTest);

% 反归一化预测和真实数据
YPredOriginal = YPred .* (poseMax - poseMin) + poseMin;
YTestOriginal = YTest .* (poseMax - poseMin) + poseMin;
YTrainOriginal = YTrain .* (poseMax - poseMin) + poseMin;

% 重新计算RMSE，使用反归一化后的数据
trainRMSE = sqrt(mean((YTrainOriginal - predict(net, XTrain) .* (poseMax - poseMin) + poseMin).^2));
testRMSE = sqrt(mean((YTestOriginal - YPredOriginal).^2));

% 绘制训练集的预测结果对比图
fig1 = figure;
plot(1:length(YTrain), YTrain(:,1), 'r-', 'LineWidth', 1);
hold on;
plot(1:length(YTrain), predict(net, XTrain), 'b--', 'LineWidth', 1);
legend('实际位姿', '预测位姿');
xlabel('时间步');
ylabel('位姿值');
title(['训练集位姿对比；' 'RMSE=' num2str(trainRMSE)]);
xlim([1, length(YTrain)]);
grid on;
savefig(fig1, 'trainPredictionComparison.fig');

% 绘制测试集的预测结果对比图
fig2 = figure;
plot(1:length(YTest), YTest(:,1), 'r-', 'LineWidth', 1);
hold on;
plot(1:length(YPred), YPred(:,1), 'b--', 'LineWidth', 1);
legend('实际位姿', '预测位姿');
xlabel('时间步');
ylabel('位姿值');
title(['测试集位姿对比；' 'RMSE=' num2str(testRMSE)]);
xlim([1, length(YTest)]);
grid on;
savefig(fig2, 'testPredictionComparison.fig');

% 生成训练集预测
trainPredictions = predict(net, XTrain);

%% 计算训练成果
% 重新生成训练集预测，并立即反归一化
trainPredictionsOriginal = predict(net, XTrain) .* (poseMax - poseMin) + poseMin;

% 计算R2
R2_train = 1 - norm(YTrainOriginal - trainPredictionsOriginal)^2 / norm(YTrainOriginal - mean(YTrainOriginal))^2;
R2_test = 1 - norm(YTestOriginal - YPredOriginal)^2 / norm(YTestOriginal - mean(YTestOriginal))^2;

disp(['训练集的标准化残差平方和R2: ', num2str(R2_train)]);
disp(['测试集的标准化残差平方和R2: ', num2str(R2_test)]);

% 计算MAE
MAE_train = mean(abs(trainPredictionsOriginal - YTrainOriginal));
MAE_test = mean(abs(YPredOriginal - YTestOriginal));

disp(['训练集的平均绝对误差MAE: ', num2str(MAE_train)]);
disp(['测试集的平均绝对误差MAE: ', num2str(MAE_test)]);

% 计算MSE
MSE_train = mean((trainPredictionsOriginal - YTrainOriginal).^2);
MSE_test = mean((YPredOriginal - YTestOriginal).^2);

disp(['训练集的均方误差MSE: ', num2str(MSE_train)]);
disp(['测试集的均方误差MSE: ', num2str(MSE_test)]);

%% 可视化指标
% % R2
% R2_scores = [0.99867, 0.94271]; % 训练集和测试集的R2得分
% figure;
% bar(R2_scores);
% ylabel('R2得分');
% set(gca, 'XTickLabel', {'训练集', '测试集'});
% title('R2得分比较');
% 
% % MAE
% MAE_train = [0.94507, 0.45309, 0.13719];
% MAE_test = [2.3911, 1.2949, 0.17262];
% figure;
% subplot(1,2,1);
% bar(MAE_train);
% title('训练集MAE');
% ylabel('MAE');
% xlabel('维度');
% subplot(1,2,2);
% bar(MAE_test);
% title('测试集MAE');
% xlabel('维度');
% 
% % MSE
% MSE_train = [1.1455, 0.37427, 0.11496];
% MSE_test = [6.5459, 2.2356, 0.041482];
% figure;
% subplot(1,2,1);
% bar(MSE_train);
% title('训练集MSE');
% ylabel('MSE');
% xlabel('维度');
% subplot(1,2,2);
% bar(MSE_test);
% title('测试集MSE');
% xlabel('维度');


%% 存入文档
% 打开文件用于写入
fileID = fopen('resultSummary.txt', 'w');

% 向文件写入R2结果
fprintf(fileID, '训练集的标准化残差平方和R2: %.5f\n', R2_train);
fprintf(fileID, '测试集的标准化残差平方和R2: %.5f\n\n', R2_test);

% 向文件写入MAE结果
fprintf(fileID, '训练集的平均绝对误差MAE: %.4f\t%.5f\t%.5f\n', MAE_train);
fprintf(fileID, '测试集的平均绝对误差MAE: %.4f\t%.5f\t%.5f\n\n', MAE_test);

% 向文件写入MSE结果
fprintf(fileID, '训练集的均方误差MSE: %.4f\t%.4f\t%.5f\n', MSE_train);
fprintf(fileID, '测试集的均方误差MSE: %.4f\t%.4f\t%.7f\n', MSE_test);

% 关闭文件
fclose(fileID);
