classdef ROS2CommManager < bridge.ROS2CommStrategy
    properties(Constant)
        % -------Wheelchair info --------
        % x:Left wheel, y:Right wheel
        t_cr2 = 0.496 % Tread width of CR2
        WhillColOpt = {'red','white','blue','green'}
        
    end

    properties
        vehicleColor

    end

    methods
        function obj = ROS2CommManager(cfg,mode)
            % Call obj.mode, obj.sensorIdx, obj.node
            obj@bridge.ROS2CommStrategy(cfg.vehicleInfo, mode, cfg.rosNamespace, cfg.RID);
            obj.vehicleColor = cfg.vehicleInfo.color;
            
            % センサデータの収集クラス
            obj.sensorFetcher = bridge.CR2.SensorFetcher(cfg);            
        end

        %% 必須メソッドの定義
        function topics = getSensorTopics(obj)
            % Subscribeするセンサのトピックを定義
            % Real Exp.
            topics = {'/combined/cloud/fusion','/ublox_gps_node/fix','/yolo/detections','/Odometry',[],'/unilidar/imu/back'};
        end

        function types = getSensorMsgTypes(obj)
            % Subscribeするセンサのメッセージ型を定義
            % Real Exp.
            types = {'sensor_msgs/PointCloud2','sensor_msgs/NavSatFix','yolo_msgs/YoloDetectionArray','nav_msgs/Odometry',[],'sensor_msgs/Imu'};
        end

        function topics = getVehicleSubsTopics(obj)
            % Subscribeするビークルのトピックを定義
            % Real Exp.
            topics = {'/whill/states/model_cr2'};
            % topics = setWhillColor(obj,topics);
        end

        function types = getVehicleSubsMsgTypes(obj)
            % Subscribeするビークルのメッセージ型を定義
            % Real Exp.
            types = {'whill_msgs/ModelCr2State'};
        end

        function topics = getVehiclePubsTopics(obj)
            % Publishするビークルのトピックを定義
            % Real Exp.
            topics = {'whill_msgs/ModelCr2State'};
            % topics = setWhillColor(obj,topics);
        end

        function types = getVehiclePubsMsgTypes(obj)
            % Publishするビークルのメッセージ型を定義
            % Real Exp.
            types = {'geometry_msgs/Twist'};
        end

        function qos = getQoSProfile(obj, topicType)
            % トピックのQoSを設定
            if nargin < 2
                topicType = [];
            end
            % 特に指定が無ければデフォルト
            % 指定する場合'sensorSubs','vehicleSubs','vehiclePubs'毎に指定も可能
            
            % Real Exp.

            % Default
            % qos = obj.BESTEFFORT_QOS; % obj.DEFAULT_QOS;

            % Custom
            qos = struct("Reliability","reliable","Durability","volatile",...
                           "History","keeplast","Depth",10);

            % switch obj.mode
            % 
            %     case 2
            %         qos = obj.BESTEFFORT_QOS;                    
            %         if strcmp(topicType,'vehiclePubs')
            %             qos.Reliability = "reliable"; % Fixing for sfm gazebo
            %         end
            %     case 3
            %         qos = obj.DEFAULT_QOS;
            % end
        end

        function msg = createVehicleCommand(obj, cmd)
            V = cmd.V;
            msg = obj.vehicleMsg;
            % seq = cmd.sequence;
            msg.linear.x = double(V(1));
            msg.angular.z = double(V(2));
        end

        function cmd = createSafetyStopCommand(obj)
            % 実機実験用: 緊急停止コマンドを作成
            cmd = struct();
            cmd.V = zeros(size(obj.prevCmd.V));
            cmd.sequence = -1;
        end
    end
    methods (Access=private)
        function topic = setWhillColor(obj,topic)
            % CR用
            % 指定カラーに基づいてトピック名を定義
            color = char(lower(obj.vehicleColor));
            if ~ismember(color,obj.WhillColOpt)
                error('InvalidColor:NotRecognized', ...
                    'The specified color "%s" is invalid. Valid options are: %s.', ...
                    color, strjoin(colors, ', '));
            end
            prefix = ['/Drp5_whill_' color '/'];
            topic{1} = regexprep(topic{1}, '^(.*/)(whill_node.*)$', [prefix '$2']);
        end
    end    
end

