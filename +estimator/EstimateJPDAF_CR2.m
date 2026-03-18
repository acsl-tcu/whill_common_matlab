classdef EstimateJPDAF_CR2 < handle
    %% Estimator
    %% Method
    % Estimate2: 初回のみ呼び出されるMethod．このクラスで用いる変数の初期定義を行う．
    % Main: 毎時刻呼び出されるMethod． 実行する推定プログラムを作る．
    %% Controllerへの推定値送信
    % "SendVarSpec"を構造体とし，推定値の変数名とその型を定義する(以下の中から選択)．
    % Controllerで同じ変数名として取り出すことが可能．
    % 型によって配列/行列への対応可否があります:
    % 行列可: {'int8','uint8','int16','uint16','int32','uint32','int64','uint64','single','double'}
    % 不可: {'string','char'}
    % "send"を構造体とし，送信したいデータを対応する変数に代入．
    %% 推定結果の保存
    % "result"を構造体とし，保存したい値を格納．

    properties (Constant)

        % -----Point cloud operation parameter---------------------
        pms_clstPCA = struct('MaxWindowRadius', 5, ...
                             'ElevationThreshold', 0.1, ...
                             'ElevationScale', 0.25, ...
                             'minDistance', 0.35, ...           % Clastaring threshold
                             'gridStep', 0.05, ...               % Downsampling rate
                             'numClusterPoints', [3 inf]);

        % -----Map operation parameter-----------------------------
        pms_ROI = struct('observe',  struct('xRoi', [-1 10], ...     % Observation x roi [x_min(m) x_max(m)]
                                            'yRoi', [-2 2], ...     % Observation y roi [y_min(m) y_max(m)]
                                            'zRoi', [-1 1]), ...  % Observation z roi [z_min(m) z_max(m)]
                         'estimate', struct('xRoi', [-1 10], ...     % Estimation range [2*x_min(m) 2*x_max(m)] (Ego positon center)
                                            'yRoi', [-2 2]), ...    % Estimation range [2*y_min(m) 2*y_max(m)] (Ego positon center)
                         'grid',     struct('gridNum', [100 30]));  % Grid size in the estimation range [x_grid_num(-) y_grid_num(-)]

        % -----Kalman filter parameter-----------------------------
        pms_pedflow = struct('K1', 1, ...           % Spring constant(State)
                             'K2', 5, ...           % Spring constant(Input)
                             'D1', 11, ...          % Damping ratio(State)
                             'D2', 0.2, ...         % Damping ratio(Input)
                             'M', 1, ...            % Mass
                             'Q', 1, ...            % System noize
                             'R', 200, ...          % Observation noize
                             'A', zeros(2), ...     % Initialize(変える場合はpedestrianFlowEstimator関数内も変更)
                             'B', eye(2), ...       % Input matrix
                             'C', [1 0]);           % Observation matrix

        % -----JPDAF parameter-------------------------------------
        pms_JPDAF = struct('positionSelector', [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 0 0], ...
                           'velocitySelector', [0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 0], ...
                           'MeasurementNoise', 0.01*eye(3));
    end

    properties
        %=======DO NOT DELETE======
        % Send variable type set to Controller
        % Example:
        SendVarSpec = struct( ...
                            'pose' , struct('Type','double','MaxSize',[1 3]), ...
                            'pos' ,  struct('Type', 'double','MaxSize',[20,3]));
        %==========================
        Allxhat % Example
        mode
        udd

        % =====Initialize parameter================================
        T_old   = 0;  % Timer
        preTrack

        pms_MAP
        tp
        trackP
        detectionP

        % -----Save flag-------------------------------------------
        pmsSaveFlag = true;
        
        % -----Mex flag--------------------------------------------
        % If you changed the parameters, 
        % I recommend you turn 'true' for the mex process only once.
        estimatorMexFlag = false;
        % =========================================================

    end

    methods
        function obj = EstimateJPDAF_CR2(mode,OfflinePath)            
            obj.mode = mode;
            
            if obj.mode == 1
                % Load matfile
                obj.udd = load(OfflinePath);
            end

            % Generating the map information
            obj.pms_MAP = struct('map', gen_gridmap(obj.pms_ROI));
            
            % -----Tracker plot parameter------------------------------
            obj.tp = theaterPlot('XLimits',[-1 10],'YLimits',[-2 2],'ZLimits',[-2 1]);
            obj.trackP = trackPlotter(obj.tp,'DisplayName','Tracks','MarkerFaceColor','g','HistoryDepth',0);
            obj.detectionP = detectionPlotter(obj.tp,'DisplayName','Detections','MarkerFaceColor','r');

        end

        function [result,send] = main(obj,sensordata,Plant,T)
            result.RawData = sensordata;
            result.Plant = Plant;
            
            persistent  current
            if obj.pmsSaveFlag
                result.pms_clstPCA  = {obj.pms_clstPCA};
                result.pms_map      = {obj.pms_ROI};
                result.pms_pedflow  = {obj.pms_pedflow};
                result.pms_JPDAF    = {obj.pms_JPDAF};
                obj.pmsSaveFlag = false;
            end

            % -----自己位置・姿勢-------------------------------------
            % current.X     = 0;
            % current.Y     = 0;
            % current.Z     = 0;
            % current.yaw   = 0;

            current.X     = Plant.X;
            current.Y     = Plant.Y;
            current.Z     = Plant.Z;
            current.yaw   = Plant.Yaw;   

            current.time = T;
            current.dt    = current.time - obj.T_old;   % 経過時間間隔を評価

            % -----Generating MEX function---------------------------------
            if obj.estimatorMexFlag == true
                % tracker_progMexGen
                pedestrianTracker2MexGen
                % clustringPCA_MexGen
                % pedestrianFlowEstimatorMexGen     <- こっちはMEX化できません。要工夫。頑張って。
                obj.estimatorMexFlag = false;
            end


            
            pcloud                  = sensordata.LIDAR; %点群を保存
            if isvector(pcloud.data)
                xyz                     = pointCloud(rosReadXYZ(struct(pcloud))); %pointCloud2からpointCloudに変換
            else
                xyz = pcloud.data;
            end
            ptCloud = xyz;            

             % -----地面点群除去
            [groundPtsIdx,nonGroundPtCloud,~] = segmentGroundSMRF(ptCloud,'MaxWindowRadius',obj.pms_clstPCA.MaxWindowRadius,...
                                        'ElevationThreshold',obj.pms_clstPCA.ElevationThreshold,'ElevationScale',obj.pms_clstPCA.ElevationScale);
            
            idxSet = 1:ptCloud.Count;

            % -----クラスタリング＋主成分分析-----------------------------------
            [labels,PCA_PtCloud] = ...
                clustring_PCA(nonGroundPtCloud.Location,current,obj.pms_clstPCA,obj.pms_ROI);

            numClusters = length(unique(labels));
            observation = NaN(numClusters,3);
            obsPC = cell(numClusters,1);

            for i = 1 : numClusters
                PointObject = pointCloud(PCA_PtCloud(labels==i,:));
                if PointObject.Count > 4 && PointObject.Count < 10000 %v1_20,7000v2_40,7000
                    observation(i,1) = mean(PointObject.Location(:,1));
                    observation(i,2) = mean(PointObject.Location(:,2));
                    observation(i,3) = mean(PointObject.Location(:,3));
                end
                obsPC{i,1} = PointObject;
            end
            observation = rmmissing(observation);
            if isempty(observation)
                observation = [];
            end

            % -----オブジェクト追跡-------------------------------------------
            [confirmedTracks,tentative,alltracks,info,detections,pos,cov,vel,meas,measCov] = pedestrianTracker2(observation,current,obj.pms_JPDAF);

            obj.preTrack = confirmedTracks;

            grid on
            PlotTrackingResult(confirmedTracks,pos,vel,cov,meas,measCov,obj.tp,current,obj.trackP,obj.detectionP);

            % -----Data update---------------------------------------------
            obj.T_old           = current.time;
            
            % save data
            result.xhat = obj.Allxhat; % example
            
            result.plane        = ptCloud;
            result.detection    = detections;
            result.confirmd     = {confirmedTracks};
            result.obstacleIndices  = {idxSet(not(groundPtsIdx))'};
            result.pos          = {pos};
            result.vel          = {vel};
            result.cov          = {cov};
            result.observation = observation;
            result.current = current;
            result.obsPC = obsPC;

            % send to Controller
            send.pose = [Plant.X, Plant.Y, Plant.Z, Plant.Roll, Plant.Pitch, Plant.Yaw];
            send.pos = pos;
            



        end


    end




end