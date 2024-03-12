clc;clear;
filename = 'packet_loss_and_covariance.xlsx';
% 读取整个工作表
data = xlsread(filename);

traceP = data(:,2);
detP = data(:,3);
detP = detP*10^30;

% 可视化迹和行列式的关系
figure;
plot(traceP, detP);
xlabel('Trace of P');
ylabel('Determinant of P');
title('Relationship');
grid on;