% Author: Yingjie Mi
function currentX = predictCurrentPose(avgX, avgV, avgG)
    % 输入参数:
    % avgX - 历史位姿的平均值，假设为列向量 [x; y; theta]
    % avgV - 历史速度的平均值
    % avgG - 历史转向角的平均值

    Delta_t = 5;

    % 使用平均速度和角度来预测位姿变化
    Delta_x = avgV * Delta_t * cos(avgG);
    Delta_y = avgV * Delta_t * sin(avgG);
    Delta_theta = avgG * Delta_t; % 假设avgG已经是角度变化率

    % 计算新的位姿
    newX = avgX(1) + Delta_x;
    newY = avgX(2) + Delta_y;
    newTheta = wrapToPi(avgX(3) + Delta_theta); % 确保角度在-π到π之间

    % 更新当前位姿
    currentX = [newX; newY; newTheta];
end
