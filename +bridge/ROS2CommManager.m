classdef ROS2CommManager < handle
    properties(Constant)
        % Estimator to Contoller
        matlabType = {'int8','uint8','int16','uint16','int32','uint32','int64','uint64','single','double','string','char'};
        ros2msgType = {'std_msgs/Int8MultiArray','std_msgs/UInt8MultiArray', ...
            'std_msgs/Int16MultiArray','std_msgs/UInt16MultiArray', ...
            'std_msgs/Int32MultiArray','std_msgs/UInt32MultiArray', ...
            'std_msgs/Int64MultiArray','std_msgs/UInt64MultiArray', ...
            'std_msgs/Float32MultiArray','std_msgs/Float64MultiArray', ...
            'std_msgs/String','std_msgs/String'};
        
        % Sensor -----------------------------------------------------------------------------------------------------------       
        SensorTopicSubs = {'/velodyne_points','/ublox_gps_node/fix','/yolo/detections','/current_pose','/current_pose'};
        SensorMsgtypeSubs = {'sensor_msgs/PointCloud2','sensor_msgs/NavSatFix','yolo_msgs/YoloDetectionArray','geometry_msgs/PoseStamped','geometry_msgs/PoseWithCovarianceStamped'};       
        % Gazebo
        gSensorTopicSubs = {'/velodyne_points',[],[],'/wheelchair/pose'};
        gSensorMsgtypeSubs = {'sensor_msgs/PointCloud2',[],[],'geometry_msgs/Pose'};

        % Wheelchair {1}:CR, {2}:CR2-----------------------------------------------------------------------------------------
        % x:Left wheel, y:Right wheel
        t_cr = 0.495 % Tread width of CR
        % WhillTopicSubs = {'/Drp5_whill/whill_node/motor_speed','/whill/states/model_cr2'};
        % WhillTopicPubs = {'/Drp5_whill/whill_node/cmd_vel','whill_msgs/ModelCr2State'};
        WhillColOpt = {'red','white'}
        WhillMsgtypeSubs = {'geometry_msgs/Vector3','/whill/controller/cmd_vel'};
        WhillMsgtypePubs = {'geometry_msgs/Twist','geometry_msgs/Twist'};
        % Gazebo
        gWhillTopicSubs = {'/wheelchair/odom'};
        gWhillTopicPubs = {'/wheelchair/diff_drive_controller/cmd_vel'};
        gWhillMsgtypeSubs = {'nav_msgs/Odometry'};
        gWhillMsgtypePubs = {'geometry_msgs/TwistStamped'};

        % CR2 Sensor
        SensorTopicSubsCR2 = {'/combined/cloud'};
        SensorMsgTypeSubsCR2 = {'sensor_msgs/PointCloud2'};

        qos_profile = struct("Reliability","reliable","Durability","volatile","History","keeplast","Depth",1)
        gqos_profile = struct("Reliability","besteffort","Durability","volatile","History","keeplast","Depth",1)

        
        
        
    end

    properties
        WhillTopicSubs = {'/empty/whill_node/motor_speed','/whill/states/model_cr2'};
        WhillTopicPubs = {'/empty/whill_node/cmd_vel','whill_msgs/ModelCr2State'};

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
            % cfg.vehicleType, 3, cfg.sensorIdx, cfg.rosNamespace, cfg.RID, cfg.base_sensor
            % cfg.vehicleType, 2, cfg.sensorIdx, cfg.rosNamespace, cfg.RID, cfg.base_sensor
            obj.vehicleType = cfg.vehicleType;
            obj.vehicleColor = cfg.vehicleColor;
            obj.mode = mode;
            obj.sensorIdx = cfg.sensorIdx;
            obj.node = bridge.ROS2CommManager.createNode(cfg.rosNamespace,cfg.RID);
            obj.Sens = bridge.SensorFetcher(cfg.modeNumber,cfg.vehicleType,cfg.sensorIdx,cfg.base_sensor);
            
        end

        % function [pubs,msgs,varname] = genEstimatorPubs(obj,info,FileName)  
        %     varname = fieldnames(info);
        %     n = numel(varname);
        %     pubs = cell(n,1);
        %     msgs = cell(n,1);
        %     VarType = cell(n,1);
        %     for i = 1:n
        %         VarType{i} = info.(varname{i});
        %         idx = find(strcmp(VarType{i},obj.matlabType));
        %         if ~isempty(idx)
        %             pubs{i} = ros2publisher(obj.node, ...
        %                 strcat("/estimation_data",string(i)), ...
        %                 obj.ros2msgType{idx});
        %             msgs{i} = ros2message(obj.ros2msgType{idx});
        %         else
        %             error('Unexpected variable type.')
        %         end
        %     end
        % 
        %     % Send variables' name and their type in advance
        %     VarName_join = strjoin(varname,',');topic
        %     VarType_join = strjoin(VarType,',');
        %     VarData = strjoin({VarName_join,VarType_join,char(FileName)},'.');
        %     obj.varpub = ros2publisher(obj.node, ...
        %                 "/estimation_data0", ...
        %                 obj.ros2msgType{11});
        %     obj.varmsg = ros2message(obj.ros2msgType{11});
        %     obj.varmsg.data = VarData;
        %     send(obj.varpub,obj.varmsg);
        % end    

        % function [subs,VarName,isNum] = genContollerSubs(obj,VarData)
        %     % Estimator to Controller
        %     VarName = split(VarData{1},',');
        %     VarType = split(VarData{2},',');
        %     n = numel(VarType);
        %     subs = cell(n,1);
        %     idx = zeros(n,1);
        %     for i = 1:n
        %         idx(i) = find(strcmp(VarType{i},obj.matlabType),1);
        %         if ~isempty(idx(i))
        %             subs{i} = ros2subscriber(obj.node, ...
        %                         strcat("/estimation_data",string(i)), ...
        %                         obj.ros2msgType{idx(i)});        
        %         else
        %             error('Unexpected variable type.')
        %         end
        %     end
        %     isNum = idx < numel(obj.matlabType)-1;
        % end

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
                obj.WhillTopicSubs = setWhillColor(obj,obj.WhillTopicSubs);
                subs = obj.WhillTopicSubs;
                msgs = obj.WhillMsgtypeSubs;
                qos = obj.qos_profile;
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
                obj.WhillTopicPubs = setWhillColor(obj,obj.WhillTopicPubs);
                pubs = obj.WhillTopicPubs;
                msgs = obj.WhillMsgtypePubs;
                qos = obj.qos_profile;
            end
            obj.whillPubs = ros2publisher(obj.node, ...
                pubs{obj.vehicleType}, ...
                msgs{obj.vehicleType}, ...
                "Reliability",qos.Reliability, ...
                "Durability",qos.Durability, ...
                "History",qos.History, ...
                "Depth",qos.Depth);
            obj.msg_cmdvel = ros2message(obj.whillPubs);
        end

        % function send_msgs_toCtrl(obj,cnt,EstData,EstVarName,pubs,msgs)
        %     if ~isempty(EstData)
        %         for k = 1:length(pubs)
        %             if ~isa(EstData.(EstVarName{k}), 'string') && ~isa(EstData.(EstVarName{k}), 'char')
        %                 msgs{k}.layout.dim.label = char(join(string(size(EstData.(EstVarName{k}))),","));
        %                 msgs{k}.layout.data_offset = uint32(cnt); % Sequence
        %             else
        %                 EstData.(EstVarName{k}) = char(join([string(cnt),EstData.(EstVarName{k})],","));
        %             end
        %             msgs{k}.data = EstData.(EstVarName{k});
        %             % send(pubs{k}, msgs{k});
        %         end
        %         for k = 1:length(pubs)
        %             send(pubs{k}, msgs{k});
        %         end
        %     end
        % end

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
                obj.msg_cmdvel.linear.x = double(V(1));
                obj.msg_cmdvel.angular.z = double(V(2))*obj.t_cr; % rad/s -> m/s
            end

            if obj.cmdMissed >= 5 && obj.mode == 3
                obj.msg_cmdvel.linear.x = 0;
                obj.msg_cmdvel.angular.z = 0;
                fprintf(2, 'The control input has not been updated. Set the input to 0 for safety.\n');
            end
            send(obj.whillPubs,obj.msg_cmdvel)
            
        end

        % function stopController(obj)
        %     obj.varmsg.data = 'stop';
        %     send(obj.varpub,obj.varmsg);
        % 
        % 
        % end

        function [data, Plant] = getSensorData(obj)
            [data ,Plant] = obj.Sens.getSensorData(obj.sensorSubs, obj.whillSubs);
        end

    end
    methods (Access=private)
        function topic = setWhillColor(obj,topic)
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
