function plot3session_preprocesser(folderPath,keepAllSequences)
    disp("Sorting data...")
    EstFileName = strcat(folderPath,filesep,"userLocal_Estimate.mat");
    CtrlFileName = strcat(folderPath,filesep,"userLocal_Control.mat");
    NodeFileName = strcat(folderPath,filesep,"userLocal_Node.mat");

    % Estimate data
    % if exist(EstFileName,"file") == 0
    %     obj.DL.stop();
    % end
    load(EstFileName,"Estimate")
    
    % Control data
    % if exist(CtrlFileName,"file") == 0
    %     obj.DL.stop();
    % end
    load(CtrlFileName,"Control")
    load(NodeFileName,"Node")

    RawData = struct("LiDAR",[],"GNSS",[],"CAMERA",[], ...
        "X",[],"Y",[],"Z",[],"Roll",[],"Pitch",[],"Yaw",[],"odom",[]);
    BaseName = fieldnames(RawData);
    numBase = numel(BaseName);
    AllEstVarName = fieldnames(Estimate{1,1});
    udVarName = setdiff(AllEstVarName, {'RawData','Plant','send','T'});
    AllEstVar = vertcat(BaseName,udVarName);
    numAllEstVar = numel(AllEstVar);
    numTSdata = numel(Estimate);
    % seq_Est = zeros(numTSdata,1);
    % T_Est = zeros(numTSdata,1);
    for n = 1:numTSdata
        % Sensor, Plant
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
        
        for m = 1:numAllEstVar-numBase
            % % if n ~= 1
            % %     try
            % %         % User difined data
            % %         EstResult.(udVarName{m})(n,1) = Estimate{n,1}.(udVarName{m});
            % %     catch
            % %         EstResult.(udVarName{m})(n,1) = [];
            % %     end
            % % else
            % %     try
            % %         % User difined data
            % %         EstResult.(udVarName{m}) = Estimate{n,1}.(udVarName{m});
            % %     catch
            % %         EstResult.(udVarName{m}) = [];
            % %     end
            % % end
            % try
            %     % User difined data
            %     EstResult.(udVarName{m}){n,1} = Estimate{n,1}.(udVarName{m});
            % catch
            %     EstResult.(udVarName{m}){n,1} = [];
            % end
            if n == 1
                try 
                    try
                        % User difined data
                        EstResult.(udVarName{m})(n,1) = Estimate{n,1}.(udVarName{m});
                    catch
                        EstResult.(udVarName{m}){n,1} = Estimate{n,1}.(udVarName{m});
                    end
                catch
                    try
                        % User difined data
                        EstResult.(udVarName{m})(n,1) = [];
                    catch
                        EstResult.(udVarName{m}){n,1} = [];
                    end
                end  
            else
                try 
                    try
                        % User difined data
                        EstResult.(udVarName{m})(n,1) = Estimate{n,1}.(udVarName{m});
                    catch
                        EstResult.(udVarName{m}){n,1} = Estimate{n,1}.(udVarName{m});
                    end
                catch
                    try
                        % User difined data
                        EstResult.(udVarName{m})(n,1) = [];
                    catch
                        EstResult.(udVarName{m}){n,1} = [];
                    end
                end
            end                
        end
        EstResult.sequence(n,1) = Estimate{n,1}.send.sequence;
        seq_Est(n) = Estimate{n,1}.send.sequence;
        T_Est(n) = Estimate{n,1}.T;
    end

    CtrlVarName = fieldnames(Control{1,1});
    numCtrlVar = numel(CtrlVarName);
    for n = 1:numel(Control)
        for m = 1:numCtrlVar
            CtrlResult.(CtrlVarName{m}){n,1} = Control{n,1}.(CtrlVarName{m});
        end
        seq_Ctrl(n) = CtrlResult.sequence{n,1};
        T_Ctrl(n) = Control{n,1}.T;
    end

    NodeVarName = fieldnames(Node{1,1});
    numNodeVar = numel(NodeVarName);
    for n = 1:numel(Node)
        for m = 1:numNodeVar
            NodeResult.(NodeVarName{m}){n,1} = Node{n,1}.(NodeVarName{m});
        end
        seq_Node(n) = NodeResult.sequence_sens{n,1};
        T_Node(n) = Node{n,1}.T;
    end

    EstTT = timetable(seconds(T_Est)');
    CtrlTT = timetable(seconds(T_Ctrl)');
    NodeTT = timetable(seconds(T_Node)');

    for i = 1:numAllEstVar
        if i <= numBase
            EstTT.(BaseName{i}) = EstResult.(BaseName{i});
        else
            EstTT.(udVarName{i-numBase}) = EstResult.(udVarName{i-numBase});
        end
    end
    for i = 1:numCtrlVar
        CtrlTT.(CtrlVarName{i}) = CtrlResult.(CtrlVarName{i});
    end
    for i = 1:numNodeVar
        NodeTT.(NodeVarName{i}) = NodeResult.(NodeVarName{i});
    end
    EstTT.sequence(:) = seq_Est;
    CtrlTT.sequence = cell2mat(CtrlTT.sequence);
    NodeTT.sequence(:) = seq_Node;
    T_est  = timetable2table(EstTT , 'ConvertRowTimes', true); % EstTT.Time が追加される
    T_ctrl = timetable2table(CtrlTT, 'ConvertRowTimes', true); % CtrlTT.Time が追加される
    T_node = timetable2table(NodeTT, 'ConvertRowTimes', true); % NodeTT.Time が追加される
    
    % 見やすくするために、変数名を変更
    T_est.Properties.VariableNames{'Time'}  = 'tEst';
    T_ctrl.Properties.VariableNames{'Time'} = 'tCtrl';
    T_node.Properties.VariableNames{'Time'} = 'tNode';
    
    % Est Ctrl join
    leftVars  = {'tEst'};
    rightVars = {'tCtrl'};
    for i = 1:numAllEstVar
        leftVars{end+1} = AllEstVar{i};
    end
    for i = 1:numCtrlVar
        rightVars{end+1} = CtrlVarName{i};
    end
    if keepAllSequences
        % outerjoin：一方にしかないデータも残したい場合はこちら
        EC = outerjoin( T_est, T_ctrl, ...
                            'Keys'          , 'sequence', ...
                            'LeftVariables' , leftVars, ...
                            'RightVariables', rightVars, ...
                            'MergeKeys',true);   % Seq を重複せず一つにまとめる

    else
        % innerjoin：共通するシーケンス番号のデータだけを取得（推奨）
        EC = innerjoin( T_est, T_ctrl, ...
                            'Keys'          , 'sequence', ...
                            'LeftVariables' , leftVars, ...
                            'RightVariables', rightVars);
    end

    % Node EC join
    ECVarName = fieldnames(EC)';
    tNodeVarName = fieldnames(T_node)';
    if keepAllSequences
        % outerjoin：一方にしかないデータも残したい場合はこちら
        userLocal = outerjoin( T_node, EC, ...
                            'Keys'          , 'sequence', ...
                            'LeftVariables' , tNodeVarName(1:end-3), ...
                            'RightVariables', ECVarName(1:end-3), ...
                            'MergeKeys',true);   % Seq を重複せず一つにまとめる

    else
        % innerjoin：共通するシーケンス番号のデータだけを取得（推奨）
        userLocal = innerjoin( T_node, EC, ...
                            'Keys'          , 'sequence', ...
                            'LeftVariables' , tNodeVarName(1:end-3), ...
                            'RightVariables', ECVarName(1:end-3));
    end
    seq_cmd = cell2mat(userLocal.sequence_cmd(1:end-1));
    sCmdSeqIdx = find(seq_cmd > 0, 1 );
    userLocal.timelagEC = seconds( userLocal.tCtrl - userLocal.tEst );
    [~,idx] = ismember(seq_cmd(sCmdSeqIdx:numel(seq_cmd)),cell2mat(userLocal.sequence_sens)); % userLocal.sequence
    tNodeValid = userLocal.tNode(sCmdSeqIdx:end);
    tNodeBase = userLocal.tNode;
    
    for i = 1:numel(tNodeValid)-1
        if idx(i) ~= 0
            tNodeTimelag(i) = tNodeValid(i) - tNodeBase(idx(i));
        else
            tNodeTimelag(i) = NaN;
        end
    end
    userLocal.timelag = seconds(vertcat(zeros(sCmdSeqIdx,1), tNodeTimelag')); %seconds(vertcat(zeros(sCmdSeqIdx-1,1), tNodeTimelag'));
    userLocal = removevars(userLocal, "T_T_node");
    userLocal = removevars(userLocal, "sequence_sens");
    userLocal = movevars(userLocal, "tEst", "Before", "sequence_cmd");
    userLocal = movevars(userLocal, "tCtrl", "Before", "sequence_cmd");
    % userLocal = movevars(userLocal, "sequence", "After", "timelag");
    % userLocal = movevars(userLocal, "sequence_cmd", "After", "sequence");
    userLocal = removevars(userLocal, "T_EC");
    
    save(strcat(folderPath,filesep,"userLocal.mat"),"userLocal")
    % 必要ならtimetableに戻す
    % FinalTT = table2timetable(Merged, 'RowTimes', 'tCtrl');  % もしくは 'tEst' を指定


end