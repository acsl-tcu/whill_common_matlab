classdef ROS2CommManager < handle
    properties(Constant)        
        % CR Sensor ------------------------------------------------------- 
        SensorTopicSubs = {'/velodyne_points','/ublox_gps_node/fix','/yolo/detections','/current_pose','/current_pose'};
        SensorMsgtypeSubs = {'sensor_msgs/PointCloud2','sensor_msgs/NavSatFix','yolo_msgs/YoloDetectionArray','geometry_msgs/PoseStamped','geometry_msgs/PoseWithCovarianceStamped'};
        
        % CR2 Sensor-------------------------------------------------------
        SensorTopicSubsCR2 = {'/combined/cloud/fusion','/ublox_gps_node/fix','/yolo/detections','/Odometry',[],'/unilidar/imu/back'};
        SensorMsgTypeSubsCR2 = {'sensor_msgs/PointCloud2','sensor_msgs/NavSatFix','yolo_msgs/YoloDetectionArray','nav_msgs/Odometry',[],'sensor_msgs/Imu'};

        % Gazebo Sensor----------------------------------------------------
        gSensorTopicSubs = {'/velodyne_points',[],[],'/wheelchair/pose'};
        gSensorMsgtypeSubs = {'sensor_msgs/PointCloud2',[],[],'geometry_msgs/Pose'};

        % Wheelchair info {1}:CR, {2}:CR2----------------------------------
        % x:Left wheel, y:Right wheel
        t_cr = 0.495 % Tread width of CR
        t_cr2 = 0.496 % Tread width of CR2
        WhillColOpt = {'red','white','blue','green'}

        % WhillTopicSubs = {'/Drp5_whill/whill_node/motor_speed','/whill/states/model_cr2'};
        % WhillTopicPubs = {'/Drp5_whill/whill_node/cmd_vel','whill_msgs/ModelCr2State'};        

        WhillMsgtypeSubs = {'geometry_msgs/Vector3','whill_msgs/ModelCr2State'};
        WhillMsgtypePubs = {'geometry_msgs/Twist','geometry_msgs/Twist'};

        % Gazebo
        gWhillTopicSubs = {'/wheelchair/odom'};
        gWhillTopicPubs = {'/wheelchair/diff_drive_controller/cmd_vel'};
        gWhillMsgtypeSubs = {'nav_msgs/Odometry'};
        gWhillMsgtypePubs = {'geometry_msgs/TwistStamped'};
        
        % ROS QoS profile
        qos_profile = struct("Reliability","reliable","Durability","volatile","History","keeplast","Depth",10)
        gqos_profile = struct("Reliability","besteffort","Durability","volatile","History","keeplast","Depth",1)
        
    end

    properties
        WhillTopicSubs = {'/empty/whill_node/motor_speed','/whill/states/model_cr2'};
        WhillTopicPubs = {'/empty/whill_node/cmd_vel',"/whill/controller/cmd_vel"};

        varpub
        varmsg
        vehicleType
        vehicleColor
        mode
        sensorIdx
        whillPubs
        msg_cmdvel
        whillSubs
        sensorSubs
        Sens
        cmdMissed=0;
        prevCmd

    end
    properties (Access=private)
        node

    end

    methods
        function obj = ROS2CommManager(cfg,mode)
            obj.vehicleType = cfg.vehicleType;
            obj.vehicleColor = cfg.vehicleColor;
            obj.mode = mode;
            obj.sensorIdx = cfg.sensorIdx;
            obj.node = bridge.ROS2CommManager.createNode(cfg.rosNamespace,cfg.RID);
            switch obj.vehicleType
                case 1
                    obj.Sens = bridge.SensorFetcher(cfg.modeNumber,cfg.vehicleType,cfg.sensorIdx,cfg.base_sensor);
                case 2
                    obj.Sens = bridge.SensorFetcher_CR2(cfg.modeNumber,cfg.vehicleType,cfg.sensorIdx,cfg.base_sensor);
            end
            
        end

        function genSensorSubs(obj)
            switch obj.vehicleType
                case 1
                    if obj.mode == 2 % gazebo
                        m = numel(obj.gSensorTopicSubs);
                        subs = obj.gSensorTopicSubs;
                        msgs = obj.gSensorMsgtypeSubs;
                        qos = obj.gqos_profile;
                    else
                        m = numel(obj.SensorTopicSubs);
                        subs = obj.SensorTopicSubs;
                        msgs = obj.SensorMsgtypeSubs;
                        qos = obj.qos_profile;
                    end
                case 2
                    m = numel(obj.SensorTopicSubsCR2);
                    subs = obj.SensorTopicSubsCR2;
                    msgs = obj.SensorMsgTypeSubsCR2;
                    qos = obj.gqos_profile;
            end
            obj.sensorSubs = cell(m,1);
            for i = 1:m
                if obj.sensorIdx(i)
                    obj.sensorSubs{i} = ros2subscriber(obj.node, ...
                        subs{i},msgs{i}, ...
                        "Reliability",qos.Reliability, ...
                        "Durability",qos.Durability, ...
                        "History",qos.History, ...
                        "Depth",qos.Depth);
                else
                    obj.sensorSubs{i} = [];
                end
            end
        end

                        
        function genWhillSubs(obj)
            if obj.mode == 2
                subs = obj.gWhillTopicSubs;
                msgs = obj.gWhillMsgtypeSubs;
                qos = obj.gqos_profile;
            else
                switch obj.vehicleType
                    case 1
                        obj.WhillTopicSubs = setWhillColor(obj,obj.WhillTopicSubs);
                        subs = obj.WhillTopicSubs;
                        msgs = obj.WhillMsgtypeSubs;
                        qos = obj.qos_profile;
                    case 2
                        % obj.WhillTopicSubs = obj.WhillTopicSubs{2}; %setWhillColor(obj,obj.WhillTopicSubs);
                        subs = obj.WhillTopicSubs;
                        msgs = obj.WhillMsgtypeSubs;
                        qos = obj.qos_profile;
                end
            end
            obj.whillSubs = ros2subscriber(obj.node, ...
                subs{obj.vehicleType}, ...
                msgs{obj.vehicleType}, ...
                "Reliability",qos.Reliability, ...
                "Durability",qos.Durability, ...
                "History",qos.History, ...
                "Depth",qos.Depth);
        end

        function genWhillPubs(obj)
            if obj.mode == 2
                pubs = obj.gWhillTopicPubs;
                msgs = obj.gWhillMsgtypePubs;
                qos = obj.gqos_profile;
                qos.Reliability = "reliable"; % Fixing for sfm gazebo
            else
                switch obj.vehicleType
                    case 1
                        obj.WhillTopicPubs = setWhillColor(obj,obj.WhillTopicPubs);
                        pubs = obj.WhillTopicPubs;
                        msgs = obj.WhillMsgtypePubs;
                        qos = obj.qos_profile;
                    case 2
                        % obj.WhillTopicSubs = obj.WhillTopicSubs{2}; %setWhillColor(obj,obj.WhillTopicSubs);
                        pubs = obj.WhillTopicPubs;
                        msgs = obj.WhillMsgtypePubs;
                        qos = obj.qos_profile;
                end
            end
            obj.whillPubs = ros2publisher(obj.node, ...'geometry_msgs/Twist','geometry_msgs/Twist'}
                pubs{obj.vehicleType}, ...
                msgs{obj.vehicleType}, ...
                "Reliability",qos.Reliability, ...
                "Durability",qos.Durability, ...
                "History",qos.History, ...
                "Depth",qos.Depth);
            obj.msg_cmdvel = ros2message(obj.whillPubs);
        end

        function send_msgs_toWhill(obj,cmd)
            V = cmd.V;
            seq = cmd.sequence;
            if ~isempty(obj.prevCmd) && seq == obj.prevCmd.sequence
                obj.cmdMissed = obj.cmdMissed + 1;
            else
                obj.prevCmd = cmd;
                obj.cmdMissed = 0;
            end
            
            if obj.mode == 2
                obj.msg_cmdvel.twist.linear.x = double(V(1));
                obj.msg_cmdvel.twist.angular.z = double(V(2));
            elseif obj.mode == 3
                switch obj.vehicleType
                    case 1 % システムの都合でCRの角速度はobj.t_crを乗算
                        obj.msg_cmdvel.linear.x = double(V(1));
                        obj.msg_cmdvel.angular.z = double(V(2))*obj.t_cr; % rad/s -> m/s
                    case 2
                        obj.msg_cmdvel.linear.x = double(V(1));
                        obj.msg_cmdvel.angular.z = double(V(2));
                end
            end

            if obj.cmdMissed >= 5 && obj.mode == 3
                obj.msg_cmdvel.linear.x = 0;
                obj.msg_cmdvel.angular.z = 0;
                fprintf(2, 'The control input has not been updated. Set the input to 0 for safety.\n');
            end
            send(obj.whillPubs,obj.msg_cmdvel)
            
        end

        function [data, Plant] = getSensorData(obj)
            [data ,Plant] = obj.Sens.getSensorData(obj.sensorSubs, obj.whillSubs);
        end

    end
    methods (Access=private)
        function topic = setWhillColor(obj,topic)
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
    methods (Static, Access=private)
        function n = createNode(nodeName,rid)
            disp("Establishing ROS2 Node...")
            persistent sNode
            if isempty(sNode) || ~isvalid(sNode)
                sNode = ros2node(nodeName,rid);    
            end
            n = sNode;
            
        end


    end
    
end

