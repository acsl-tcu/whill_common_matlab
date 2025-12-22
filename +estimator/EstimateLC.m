classdef EstimateLC < handle
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
        pms_ROI = struct('observe',  struct('xRoi', [-1 8], ...     % Observation x roi [x_min(m) x_max(m)]
                                            'yRoi', [-3 3], ...     % Observation y roi [y_min(m) y_max(m)]
                                            'zRoi', [-1.5 1]), ...  % Observation z roi [z_min(m) z_max(m)]
                         'estimate', struct('xRoi', [-1 8], ...     % Estimation range [2*x_min(m) 2*x_max(m)] (Ego positon center)
                                            'yRoi', [-3 3]), ...    % Estimation range [2*y_min(m) 2*y_max(m)] (Ego positon center)
                         'gridNum', [100 30]);                      % Grid size in the estimation range [x_grid_num(-) y_grid_num(-)]

        % -----JPDAF parameter-------------------------------------
        pms_JPDAF = struct('positionSelector', [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 0 0], ...
                           'velocitySelector', [0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 0], ...
                           'MeasurementNoise', 0.1*eye(3));

        % -----Compression parameter-------------------------------
        AngularVelocity = 21000;
        
        % -----Camera Parameter------------------------------------
        scorethreshold = 0.6;
    end

    properties
        %=======DO NOT DELETE======
        % Send variable type set to Controller
        % Example:
        SendVarSpec = struct( ...
                            'pose' , struct('Type','double','MaxSize',[1 3]));
        %==========================
        Allxhat % Example
        mode
        udd

        % =====Initialize parameter================================
        T_old   = 0;  % Timer

        pms_MAP
        tp
        trackP
        detectionP

        calibpms   % キャリブレーション結果のファイルload用
        intrinsics % カメラの内部パラメータ
        tform      % LiDARからカメラへの剛体変換

        preLstamp = 0; % 前フレームのLiDARタイムスタンプ

        idxSet
        groundPtsIdx

        % -----Save flag-------------------------------------------
        pmsSaveFlag = true;
        
        % -----Mex flag--------------------------------------------
        % If you changed the parameters, 
        % I recommend you turn 'true' for the mex process only once.
        estimatorMexFlag = false;
        % =========================================================

    end

    methods
        function obj = EstimateLC(mode,OfflinePath,calibparamPath,cameraparamPath)            
            obj.mode = mode;
            
            if obj.mode == 1
                % Load matfile
                obj.udd = load(OfflinePath);
            end

            % Generating the map information
            obj.pms_MAP = struct('map', gen_gridmap(obj.pms_ROI));
            
            % -----Tracker plot parameter------------------------------
            obj.tp = theaterPlot('XLimits',[-10 10],'YLimits',[-10 10],'ZLimits',[-2 1]);
            obj.trackP = trackPlotter(obj.tp,'DisplayName','Tracks','MarkerFaceColor','g','HistoryDepth',0);
            obj.detectionP = detectionPlotter(obj.tp,'DisplayName','Detections','MarkerFaceColor','r');

            % -----Camera calibration parameter------------------------
            obj.calibpms = load(calibparamPath);
            obj.intrinsics = obj.calibpms.intrinsicsss.Intrinsics;
            obj.tform = obj.calibpms.tform;

        end

        function [result,send] = main(obj,sensordata,Plant,T)
            result.RawData = sensordata;
            result.Plant = Plant;
            
            persistent  current
            if obj.pmsSaveFlag
                result.pms_clstPCA = {obj.pms_clstPCA};
                result.pms_map     = {obj.pms_ROI};
                result.pms_JPDAF   = {obj.pms_JPDAF};
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

            % -----経過時間・時間間隔----------------------------------
            current.time = T;
            current.dt   = current.time - obj.T_old;   % 経過時間間隔を評価

            Lstampnano = num2str(sensordata.LIDAR.header.stamp.nanosec);
            if length(Lstampnano) ~= 9
                for i = 1 : 9 - length(Lstampnano)
                    Lstampnano = ['0' Lstampnano];
                end
            end
            Lstamp = str2double(num2str(sensordata.LIDAR.header.stamp.sec) + "." + Lstampnano); % LiDARタイムスタンプ
            dtL = Lstamp - obj.preLstamp; % LiDARサンプリング時間

            % -----Z軸圧縮-----------------------------------------------
            % LiDARの分解能
            resolution = 360 * obj.AngularVelocity * 0.001 /18750; %手入力ではなく初めから変えてもよし
            Z_Comp = (tan(deg2rad(2))/tan(deg2rad(resolution)));

            % -----Generating MEX function---------------------------------
            if obj.estimatorMexFlag == true
                % tracker_progMexGen
                pedestrianTracker2MexGen
                % clustringPCA_MexGen
                % pedestrianFlowEstimatorMexGen     <- こっちはMEX化できません。要工夫。頑張って。
                obj.estimatorMexFlag = false;
            end


            if ~isempty(sensordata.LIDAR)
                % ptCloud = pointCloud(rosReadXYZ(sensordata.LIDAR));
            end
            % ptCloud = pointCloud(rosReadXYZ(sensordata.LIDAR));
            pcloud                  = sensordata.LIDAR; %点群を保存
                    if isvector(pcloud.data)
                        xyz                     = pointCloud(rosReadXYZ(struct(pcloud))); %pointCloud2からpointCloudに変換
                    else
                        xyz = pcloud.data;
                    end
                    ptCloud = xyz;
            if ~isempty(sensordata.CAMERA)
                % fusion();
            end
            camdata = sensordata.CAMERA;
            % if ~isempty(sensordata.GNSS)
            % end
            % % if ~isempty(sensordata.SelfPos)
            % % end

            if dtL ~=0 % 連続するフレームで点群が同一の場合は処理しない
                % -----地面点群除去-----------------------------------------------
                [obj.groundPtsIdx,nonGroundPtCloud,~] = segmentGroundSMRF(ptCloud,'MaxWindowRadius',obj.pms_clstPCA.MaxWindowRadius,...
                    'ElevationThreshold',obj.pms_clstPCA.ElevationThreshold,'ElevationScale',obj.pms_clstPCA.ElevationScale);

                obj.idxSet = 1:ptCloud.Count;

                % -----関心領域内点群切り取り---------------------------------------
                [pCloudCrop, pCloudLCS] = pcCrop(nonGroundPtCloud,current,obj.pms_ROI);

                % -----クラスタリング＋主成分分析-----------------------------------
                [labels,PCA_PtCloud] = ...
                    clustring_PCA(pCloudCrop,obj.pms_clstPCA,Z_Comp);

                % -----カメラ・LiDARフュージョン-----------------------------------
                [ObjectData,Objectpt,labels,ObjectData_fromCamera,ObjectData_fromLiDAR] = LiDARcamerafusion(nonGroundPtCloud, pCloudLCS.Location, PCA_PtCloud, camdata, obj.intrinsics, obj.tform, obj.scorethreshold, labels,Z_Comp);
                % 地面除去をした点群(nonGroundPtCloud)(壁などの点群有)と追跡対象のみの点群(PCA_PtCloud)(壁などの点群無)の両方を引数にする．クラスタリングなどの手法は各々で変更してください．
                % ObjectData_fromCameraとObjectData_fromLiDARは検証用．開発終わったら消して良い．

                numClusters = size(ObjectData,1);
                observation = NaN(numClusters,3);

                for i = 1 : numClusters
                    PointObject = pointCloud(ObjectData{i,1});
                    if PointObject.Count > 4 && PointObject.Count < 700 %v1_20,7000v2_40,7000
                        observation(i,1) = mean(PointObject.Location(:,1));
                        observation(i,2) = mean(PointObject.Location(:,2));
                        observation(i,3) = mean(PointObject.Location(:,3));
                    end
                    obsPC{i,1} = PointObject;
                end

                observation = rmmissing(observation);
                if isempty(observation)
                    observation = [];
                    obsPC{1,1} = [];
                end
            else
                ObjectData = {};
                ObjectData_fromCamera = {};
                ObjectData_fromLiDAR = {};
                observation = [];
                obsPC = {};
            end

            % -----オブジェクト追跡-------------------------------------------
            % [pos,cov,vel,detections,confirmedTracks] = ...
            %     pedestrianTracker(PCA_PtCloud,current,obj.pms_JPDAF,labels);
 
            [confirmedTracks,tentative,alltracks,info,detections,pos,cov,vel,meas,measCov] = pedestrianTracker2(observation,current,obj.pms_JPDAF);

            grid on
            PlotTrackingResult(obj.trackP,obj.detectionP,confirmedTracks,pos,vel,cov,meas,measCov,current);

            % -----Data update---------------------------------------------
            obj.T_old           = current.time;
            
            % save data
            result.xhat = obj.Allxhat; % example
            
            result.plane        = ptCloud;
            result.detection    = detections;
            result.confirmd     = confirmedTracks;
            result.obstacleIndices  = obj.idxSet(not(obj.groundPtsIdx))';
            result.pos          = pos;
            result.vel          = vel;
            result.cov          = cov;
            result.observation = observation;
            result.current = current;
            result.obsPC = obsPC;
            result.ObjectData = ObjectData;
            result.ObjectData_fromCamera = ObjectData_fromCamera;
            result.ObjectData_fromLiDAR = ObjectData_fromLiDAR;

            % send to Controller
            send.pose = [0, 0, 0];
            



        end


    end




end