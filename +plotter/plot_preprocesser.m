function plot_preprocesser(folderPath,mode)
    disp("Sorting data...")
    FileName = strcat(folderPath,filesep,"userLocal_tmp.mat");

    % Data    
    load(FileName,"tmp")
    for i = 1:numel(tmp)
        Estimate{i,1} = tmp{i}.Estimator;
        Control{i,1} = tmp{i}.Controller;
    end
    

    RawData = struct("LiDAR",[],"GNSS",[],"CAMERA",[], ...
        "X",[],"Y",[],"Z",[],"Roll",[],"Pitch",[],"Yaw",[],"odom",[]);
    BaseName = fieldnames(RawData);    
    if mode == 1 || mode == 0
        numBase = 0;
    else
        numBase = numel(BaseName);
    end    
    AllEstVarName = fieldnames(Estimate{1,1});
    udVarName = setdiff(AllEstVarName, {'RawData','Plant','send','T'});
    % AllEstVar = vertcat(BaseName,udVarName);
    numAllEstVar = numBase+numel(udVarName);
    numTSdata = numel(Estimate);
    % seq_Est = zeros(numTSdata,1);
    % T_Est = zeros(numTSdata,1);
    for n = 1:numTSdata
        % Sensor, Plant
        if mode ~= 1 && mode ~= 0
            EstResult.(BaseName{1}){n,1} = Estimate{n,1}.RawData.LIDAR;
            EstResult.(BaseName{2}){n,1} = Estimate{n,1}.RawData.GNSS;
            EstResult.(BaseName{3}){n,1} = Estimate{n,1}.RawData.CAMERA;
            EstResult.(BaseName{4})(n,1) = Estimate{n,1}.Plant.X;
            EstResult.(BaseName{5})(n,1) = Estimate{n,1}.Plant.Y;
            EstResult.(BaseName{6})(n,1) = Estimate{n,1}.Plant.Z;
            EstResult.(BaseName{7})(n,1) = Estimate{n,1}.Plant.Roll;
            EstResult.(BaseName{8})(n,1) = Estimate{n,1}.Plant.Pitch;
            EstResult.(BaseName{9})(n,1) = Estimate{n,1}.Plant.Yaw;
            EstResult.(BaseName{10})(n,:) = Estimate{n,1}.Plant.odom;
        end
        
        for m = 1:numAllEstVar-numBase
            % if n ~= 1
            %     try
            %         % User difined data
            %         EstResult.(udVarName{m})(n,1) = Estimate{n,1}.(udVarName{m});
            %     catch
            %         EstResult.(udVarName{m})(n,1) = [];
            %     end
            % else
            %     try
            %         % User difined data
            %         EstResult.(udVarName{m}) = Estimate{n,1}.(udVarName{m});
            %     catch
            %         EstResult.(udVarName{m}) = [];
            %     end
            % end
            try
                % User difined data
                EstResult.(udVarName{m}){n,1} = Estimate{n,1}.(udVarName{m});
            catch
                EstResult.(udVarName{m}){n,1} = [];
            end
                
        end
        T_Est(n) = Estimate{n,1}.T;
    end

    CtrlVarName = fieldnames(Control{1,1});
    numCtrlVar = numel(CtrlVarName);
    for n = 1:numel(Control)
        for m = 1:numCtrlVar
            CtrlResult.(CtrlVarName{m}){n,1} = Control{n,1}.(CtrlVarName{m});
        end
        % T_Ctrl(n) = Control{n,1}.T;
    end

    userLocal = timetable(seconds(T_Est)');
    % CtrlTT = timetable(seconds(T_Ctrl)');

    for i = 1:numAllEstVar+numCtrlVar
        if i <= numBase
            userLocal.(BaseName{i}) = EstResult.(BaseName{i});
        elseif i <= numAllEstVar
            userLocal.(udVarName{i-numBase}) = EstResult.(udVarName{i-numBase});
        else
            userLocal.(CtrlVarName{i-numAllEstVar}) = CtrlResult.(CtrlVarName{i-numAllEstVar});
        end
    end
    
    userLocal = timetable2table(userLocal);
    userLocal = removevars(userLocal, "sequence");
    userLocal = removevars(userLocal, "T");
    save(strcat(folderPath,filesep,"userLocal.mat"),"userLocal")
    % 必要ならtimetableに戻す
    % FinalTT = table2timetable(Merged, 'RowTimes', 'tCtrl');  % もしくは 'tEst' を指定


end