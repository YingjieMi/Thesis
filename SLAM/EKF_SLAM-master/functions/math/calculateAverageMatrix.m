% Author： Yingjie Mi

function avgMatrix = calculateAverageMatrix(expandedHistoryP)
    % 获取扩展后的矩阵维度
    [maxRows, maxCols] = size(expandedHistoryP{1});
    
    % 初始化一个矩阵来存储平均值，维度与扩展后的矩阵相同
    avgMatrix = zeros(maxRows, maxCols);
    
    % 计算每个位置的平均值
    for i = 1:maxRows
        for j = 1:maxCols
            % 对于每个位置(i,j)，计算所有矩阵在该位置的平均值
            sum = 0;
            for k = 1:length(expandedHistoryP)
                sum = sum + expandedHistoryP{k}(i, j);
            end
            avgMatrix(i, j) = sum / length(expandedHistoryP);
        end
    end
end
