% 提示用户输入
choice = input(['请选择:\n' ...
    '     0 计算初始的平均速度、位置与姿态角\n' ...
    '     1 计算结果与参考真值差分图\n' ...
    '     2 计算结果与参考真值对比图\n' ...
    '     3 计算结果与参考真值轨迹图\n' ...
    '     4 计算结果RMSE随时间变化图\n' ...
    '     5 参考数据的差分图与对比图\n']);
if(~(choice == 0 || choice == 1 || choice == 2|| choice == 3 || choice == 4 || choice == 5))
    fprintf("\n无法识别：%d 程序结束!\n",choice);
end
switch(choice)
    case 0
        % 读取数据
        truthdata_file_path = "truth.nav";
        truthdata = ReadFile(truthdata_file_path, 1);
        
        % 提取数据列
        second = table2array(truthdata(:, 2));
        B = table2array(truthdata(:, 3));
        L = table2array(truthdata(:, 4));
        H = table2array(truthdata(:, 5));
        
        roll = table2array(truthdata(:, 9));
        pitch = table2array(truthdata(:, 10));
        yaw = table2array(truthdata(:, 11));
        
        % 设置时间阈值
        time_threshold = 440532.000;
        
        % 筛选出时间小于阈值的数据
        index_before_threshold = second < time_threshold;
        
        % 取出符合条件的数据
        B_selected = B(index_before_threshold);
        L_selected = L(index_before_threshold);
        H_selected = H(index_before_threshold);
        roll_selected = roll(index_before_threshold);
        pitch_selected = pitch(index_before_threshold);
        yaw_selected = yaw(index_before_threshold);
        
        % 计算这些数据的平均值
        B_avg = mean(B_selected);
        L_avg = mean(L_selected);
        H_avg = mean(H_selected);
        roll_avg = mean(roll_selected);
        pitch_avg = mean(pitch_selected);
        yaw_avg = mean(yaw_selected);
        
        % 设置输出格式为10位小数
        fprintf('Average B: %.10f\n', B_avg);
        fprintf('Average L: %.10f\n', L_avg);
        fprintf('Average H: %.10f\n', H_avg);
        fprintf('Average roll: %.10f\n', roll_avg);
        fprintf('Average pitch: %.10f\n', pitch_avg);
        fprintf('Average yaw: %.10f\n', yaw_avg);

    case 1    
        % dresult_zero_file_path = "diff_zero_result.txt";
        % dresult_file_path = "diff_result.txt";
        % 
        % dresult_zero_data = ReadFile(dresult_zero_file_path, 0);
        % dresult_data = ReadFile(dresult_file_path, 0);
        % 
        % DiffFig(dresult_zero_data, dresult_data, 1);
        
        dresult_file_path = "diff_result.txt";

        dresult_data = ReadFile(dresult_file_path, 0);

        DiffFig(dresult_data, 1);
    
    case 2
        ours_result_file_path = "result.txt";
        truthdata_file_path = "true.txt";

        ours_data = ReadFile(ours_result_file_path, 0);
        true_data = ReadFile(truthdata_file_path, 0);

        DataFig(ours_data, true_data);

    case 3
        denu_file_path = "denu_result.txt";

        denu_data = ReadFile(denu_file_path, 2);

        dEnuFig(denu_data);
   
    case 4
        dresult_file_path = "diff_result.txt";
        denu_file_path = "denu_result.txt";

        dresult_data = ReadFile(dresult_file_path, 0);
        denu_data = ReadFile(denu_file_path, 2);

        RmseWithTime(dresult_data, denu_data, 100);

        % dresult_file_path = "diff_zero_result.txt";
        % denu_file_path = "denu_zero_result.txt";
        % 
        % dresult_data = ReadFile(dresult_file_path, 0);
        % denu_data = ReadFile(denu_file_path, 2);
        % 
        % RmseWithTime(dresult_data, denu_data, 100);

    case 5
        ins_result_file_path = "ExampleResult/INS_Result.txt";
        true_file_path = "ExampleResult/INS_True.txt";
        ins_diff_result_file_path = "ExampleResult/INS_Diff_Result.txt";
        denu_file_path = "ExampleResult/denu_Result.txt";

        ins_data = ReadFile(ins_result_file_path, 0);
        true_data = ReadFile(true_file_path, 0);
        ins_diff_data = ReadFile(ins_diff_result_file_path, 0);
        denu_data = ReadFile(denu_file_path, 2);

        DataFig(ins_data, true_data);
        DiffFig(ins_diff_data, 0);
        dEnuFig(denu_data);
end