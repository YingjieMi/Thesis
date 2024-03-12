% Author: Yingjie Mi

function expandedHistoryP = expandMatricesToMaxSize(historyP)
    % 初始化最大行数和列数
    maxRows = 0;
    maxCols = 0;

    % 第一步: 遍历所有矩阵，找到最大的行数和列数
    for i = 1:length(historyP)
        [rows, cols] = size(historyP{i});
        if rows > maxRows
            maxRows = rows;
        end
        if cols > maxCols
            maxCols = cols;
        end
    end

    % 第二步: 扩展较小的矩阵
    for i = 1:length(historyP)
        [rows, cols] = size(historyP{i});
        % 如果矩阵维度小于最大维度，则扩展该矩阵
        if rows < maxRows || cols < maxCols
            % 创建一个新的最大维度矩阵，初始化为零
            expandedP = zeros(maxRows, maxCols);
            % 将原矩阵的内容复制到新矩阵的对应位置
            expandedP(1:rows, 1:cols) = historyP{i};
            % 更新cell数组中的矩阵
            historyP{i} = expandedP;
        end
    end

    % 将扩展后的矩阵cell数组作为输出返回
    expandedHistoryP = historyP;
end
