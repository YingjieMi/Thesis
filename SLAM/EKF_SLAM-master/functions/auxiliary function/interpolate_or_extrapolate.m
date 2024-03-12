function z_estimated = interpolate_or_extrapolate(history_z)
    num_points = size(history_z, 2);

    if num_points < 3
        % 如果历史数据点不足以进行二次多项式拟合，回退到线性外推
        if num_points < 2
            z_estimated = history_z(:, end);  % 如果只有一个数据点，直接使用该点
        else
            z_diff = history_z(:, end) - history_z(:, end-1);
            z_estimated = history_z(:, end) + z_diff;
        end
    else
        % 使用二次多项式外推
        t = 1:num_points;  % 时间点
        z_estimated = zeros(size(history_z, 1), 1);

        for i = 1:size(history_z, 1)
            p = polyfit(t, history_z(i, :), 2);  % 对每个维度拟合二次多项式
            z_estimated(i) = polyval(p, num_points+1);  % 预测下一个时间点的值
        end
    end
end
