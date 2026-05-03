function data = ReadFile(filename, mode)
    % 打开文件
    fileID = fopen(filename, 'r');
    if fileID == -1
        error('无法打开文件');
    end
    
    switch(mode)
        case 0 % 读取差分数据或者解算结果数据
            % 使用 textscan 读取文件数据，处理多个空格作为一个分隔符
            data = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f %f', 'Delimiter', ' ', 'MultipleDelimsAsOne', true, 'HeaderLines', 1);
                        
            % 将数据转为 table，列名可以根据需要进行修改
            data = table(data{1}, data{2}, data{3}, data{4}, data{5}, data{6}, data{7}, data{8}, data{9}, data{10});

        case 1 % 读取真值数据
            % 使用 textscan 读取文件数据，处理多个空格作为一个分隔符
            data = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f', 'Delimiter', ',', 'MultipleDelimsAsOne', true, 'HeaderLines', 0);
                        
            % 将数据转为 table，列名可以根据需要进行修改
            data = table(data{1}, data{2}, data{3}, data{4},data{5},data{6},data{7},data{8},data{9},data{10},data{11});

        case 2 % 读取denu轨迹数据
            % 使用 textscan 读取文件数据，处理多个空格作为一个分隔符
            data = textscan(fileID, '%f %f %f %f %f %f %f', 'Delimiter', ',', 'MultipleDelimsAsOne', true, 'HeaderLines', 1);
                        
            % 将数据转为 table，列名可以根据需要进行修改
            data = table(data{1}, data{2}, data{3}, data{4},data{5},data{6},data{7});
        
    end
    
    % 关闭文件
    fclose(fileID);
end
