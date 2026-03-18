classdef SensorFetcher < handle
    % 各センサデータのメッセージをsubscriberから受け取り，使いやすい形に整理するクラス

    properties
        mode
        sensorIdx
        baseSensor
        nostd = 0;

        offset
        R
        yaw0

    end
    properties (Access=private)
        prevAngle
        timeout = 0.05;

    end
    properties (Constant)
        % t_cr = 0.495; % Tread width of CR
    end

    methods
        function obj = SensorFetcher(cfg)
            obj.mode = cfg.modeNumber;
            obj.sensorIdx = cfg.vehicleInfo.sensor;
            obj.baseSensor = cfg.vehicleInfo.base_sensor;
            if obj.baseSensor == 0
                obj.nostd = 1;
            end
        end

        function [data,state] = getSensorData(obj,sensorSubs,sub)
            data = [];
            % ===== ros2読み取り =====
            [msg,status,statustext] = receive(sub{1});
            
            % ----- 位置 -----
            x = double(msg.position(1)); %位置x
            y = double(msg.position(2)); %位置y
            % ----- 角度 -----
            q    = msg.imu_state.quaternion;
            quat = [q(1), q(2), q(3), q(4)];
            eul  = quat2eul(quat);           % eul = [yaw,pitch,roll]
            yaw  = double(eul(1)); 
            if isempty(obj.yaw0)
                cosy=cos(yaw);
                siny=sin(yaw);
                obj.R=[cosy siny;-siny cosy];
                obj.offset=[x;y];
                obj.yaw0=yaw;
            end
            % clear P
            P=[x;y];
            P=obj.R*(P-obj.offset);
            x=P(1);
            y=P(2);
            theta=double(wrapToPi(yaw-obj.yaw0));
            % ------ 速度 -----
            mu = double(msg.velocity(1)); 
            nu = double(msg.velocity(2));
            % ------ 角速度 -----
            omega = double(msg.yaw_speed); %角速度ω
            
            state = [x;y;theta;mu;nu;omega];
            
        end
    end
end
