classdef ROS2CommManager < bridge.ROS2CommStrategy
    properties(Constant)
        
        
    end

    properties

    end

    methods
        function obj = ROS2CommManager(cfg,mode)
            % Call obj.mode, obj.sensorIdx, obj.node
            obj@bridge.ROS2CommStrategy(cfg.vehicleInfo, mode, cfg.rosNamespace, cfg.RID);
            
            % センサデータの収集クラス
            obj.sensorFetcher = bridge.Go2.SensorFetcher(cfg);
        end
        
        %% 必須メソッドの定義
        function topics = getSensorTopics(obj)
            % Subscribeするセンサのトピックを定義
            switch obj.mode
                case 2 % Gazebo
                    % 他モードと配列数を合わせることで，sensorIdxの整合性を持たせる
                    topics = {'/velodyne_points',[],[],'/wheelchair/pose',[]};
                case 3 % Real Exp.
                    topics = {[],[],[],[],[],[]};
            end
        end

        function types = getSensorMsgTypes(obj)
            % Subscribeするセンサのメッセージ型を定義
            switch obj.mode
                case 2
                    types = {'sensor_msgs/PointCloud2',[],[],'geometry_msgs/Pose',[]};
                case 3
                    types = {[],[],[],[],[],[]};
            end
        end

        function topics = getVehicleSubsTopics(obj)
            % Subscribeするビークルのトピックを定義
            switch obj.mode
                case 2
                    topics = {'/wheelchair/odom'};
                case 3
                    topics = {'/lf/sportmodestate'};
            end
        end

        function types = getVehicleSubsMsgTypes(obj)
            % Subscribeするビークルのメッセージ型を定義
            switch obj.mode
                case 2
                    types = {'nav_msgs/Odometry'};
                case 3
                    types = {'unitree_go/SportModeState'};
            end
        end

        function topics = getVehiclePubsTopics(obj)
            % Publishするビークルのトピックを定義
            switch obj.mode
                case 2
                    topics = {'/wheelchair/diff_drive_controller/cmd_vel'};
                case 3                    
                    topics = {'/api/sport/request'};
            end
        end

        function types = getVehiclePubsMsgTypes(obj)
            % Publishするビークルのメッセージ型を定義
            switch obj.mode
                case 2
                    types = {'geometry_msgs/TwistStamped'};
                case 3
                    types = {'unitree_api/Request'};
            end
        end

        function qos = getQoSProfile(obj, topicType)
            % トピックのQoSを設定
            if nargin < 2
                topicType = [];
            end
            switch obj.mode
                % 特に指定が無ければデフォルト
                % 指定する場合'sensorSubs','vehicleSubs','vehiclePubs'毎に指定
                case 2
                    qos = obj.BESTEFFORT_QOS;                    
                    % if strcmp(topicType,'vehiclePubs')
                    %     qos.Reliability = "reliable"; % Fixing for sfm gazebo
                    %     qos.Depth = 10;
                    % end
                case 3
                    qos = obj.DEFAULT_QOS;
            end
        end

        function msg = createVehicleCommand(obj, cmd)
            V = cmd.V;
            msg = obj.vehicleMsg;

            % msg=ros2message(ros2.pub);
            % param=struct('x',0.,'y',0.,'z',0.);
            % msg.parameter=jsonencode(param);
            % msg.header.identity.api_id=ros2.ROBOT_SPORT_API_ID_MOVE;

            % ros2.ROBOT_SPORT_API_ID_MOVE = int64(1008);

            % seq = cmd.sequence;
            if obj.mode == 2
                msg.twist.linear.x = double(V(1));
                msg.twist.angular.z = double(V(2));
            elseif obj.mode == 3
                param=struct('x',V(1),'y',V(2),'z',V(3));
                msg.parameter=jsonencode(param);
                msg.header.identity.api_id = int64(1008);
            end
        end

        function cmd = createSafetyStopCommand(obj)
            % 実機実験用: 緊急停止コマンドを作成
            cmd = struct();
            cmd.V = zeros(size(obj.prevCmd.V));
            cmd.sequence = -1;
        end
    end    
end

