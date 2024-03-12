function history_z = update_history(history_z, new_z, window_size)
    % 确保new_z是一个列向量或二维数组
    if size(new_z, 2) == 1
        new_z = new_z';  % 如果new_z是列向量，转置成行向量
    end

    % 添加新数据到历史数组
    history_z = [history_z, new_z];

    % 保持历史数据窗口大小
    if size(history_z, 2) > window_size
        history_z = history_z(:, end-window_size+1:end);
    end
end
