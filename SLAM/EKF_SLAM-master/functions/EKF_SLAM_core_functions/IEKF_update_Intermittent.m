function [x,P]= IEKF_update_Intermittent(x,P,z,R,idf,N, link_quality)
% 添加 gamma 参数来标识每个观测是否到达

global IDF GAMMA
IDF= idf;
% 创建一个与 z 的观测次数相同长度的 link_quality 数组
gamma = repmat(link_quality, 1, size(z,2));
GAMMA = gamma; % 将 gamma 变量设置为全局变量以便在子函数中使用



if isempty(idf), return, end

lenz= size(z,2);
RR= zeros(2*lenz);
zz= zeros(2*lenz,1);
for i=1:lenz
    ii= 2*i + (-1:0);
    if gamma(i) == 1 % 只有当观测到达时，才将其包含到 zz 和 RR 中
        zz(ii)= z(:,i);
        RR(ii,ii)= R;
    else % 如果观测未到达，设置一个很大的协方差，表示这个观测非常不确定
        zz(ii)= z(:,i); % 保留原始观测值
        RR(ii,ii)= diag([1e10, 1e10]); % 非常大的协方差
    end
end

[x,P] = KF_IEKF_update(x,P, zz,RR, @hmodel, @hjacobian, N);

function v= hmodel(x,z)
global IDF GAMMA
lenz= length(IDF);
v= zeros(2*lenz, 1);

for i=1:lenz
    ii= 2*i + (-1:0);
    if GAMMA(i) == 1 % 只处理到达的观测值
        [zp,dmy]= observe_model(x, IDF(i));
        v(ii)= z(ii)-zp;
        v(ii(2))= pi_to_pi(v(ii(2)));
    else % 对于未到达的观测，我们不更新 v
        v(ii)= [0;0]; % 未到达观测值的差设置为 0
    end
end

function H= hjacobian(x)
global IDF GAMMA

lenz= length(IDF);
lenx= length(x);
H= zeros(2*lenz, lenx);

for i=1:lenz
    ii= 2*i + (-1:0);
    if GAMMA(i) == 1 % 只计算到达观测值的雅可比
        [dmy,H(ii,:)]= observe_model(x, IDF(i));
    else % 对于未到达的观测，我们不更新 H
        H(ii,:)= zeros(2, lenx); % 未到达观测值的雅可比设置为 0
    end
end
