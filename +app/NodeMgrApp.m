classdef NodeMgrApp < handle
    properties
        Mode        
        SensSHM     
        CmdSHM
        Timer
        DLog
        SMgr
        % TimerCB
        cmd
        checkBin = 1;
    end
    properties (Constant)
        sensorVars = {
                     'LIDAR',       'single',   [35000 3];
                     'Ldr_sec',     'int32',    [1,1];
                     'Ldr_nsec',    'uint32',   [1,1];
                     'GNSS',        'double',   [2 1];
                     'G_sec',       'int32',    [1,1];
                     'G_nsec',      'uint32',   [1,1];
                     'CAM_detLabel','uint8',    [1 30];
                     'CAM_detBbox', 'uint16',   [4,30];
                     'CAM_detHue',  'uint32',   [180 30];
                     'CAM_detScore','single',   [1 30];
                     'CAM_pmasks',  'uint8',    [480 640];
                     'CAM_sec',     'int32',    [1,1];
                     'CAM_nsec',    'uint32',   [1,1];
                     'Plant',       'double',   [10 1]};


    end
    methods
        function obj = NodeMgrApp(cfg)
            % I/O mode
            obj.Mode = mode.ModeFactory.build(cfg);
            % SHM
            fname = strcat(tempdir,cfg.sharedMemKey);
            obj.CmdSHM  = bridge.SharedMem(fname+"cmd.bin",obj.checkBin);
            obj.SensSHM = bridge.SharedMem(fname+"sensor.bin",obj.checkBin,obj.sensorVars);
            obj.Timer  = rateControl(1/cfg.tspan);
            obj.DLog = cfg.logger;
            obj.SMgr = session.SessionManager.build(cfg,session.SessionManager.initiator);
            % % Timer callback configuration
            % delete(timerfindall)
            % obj.TimerCB = timer( ...
            %     'ExecutionMode','fixedRate', ...
            %     'Period',        cfg.tspan, ...
            %     'BusyMode',      'drop', ...
            %     'TimerFcn',      @(~,~)obj.step(), ...
            %     'ErrorFcn',      @(~,~)obj.err(), ...
            %     'StopFcn',       @(~,~)obj.stop());
        end
        function run(obj)
            c = onCleanup(@()obj.Mode.shutdown());
            obj.Mode.setup();
            obj.SMgr.start();
            reset(obj.Timer)
            % start(obj.TimerCB)
            while obj.SMgr.Stopper.UserData
                [sens,Plant] = obj.Mode.receiveData();
                result.sequence_sens = obj.SensSHM.write(obj.encodeSensor(sens,Plant));
                [currentCmd, ok] = obj.CmdSHM.read();
                if ok
                    obj.cmd = currentCmd;
                    obj.Mode.sendData(obj.cmd);
                    result.sequence_cmd = obj.cmd.sequence;
                elseif ~isempty(obj.cmd)
                    result.sequence_cmd = obj.cmd.sequence;
                else
                    result.sequence_cmd = uint32(0);
                end
                result.T = posixtime(datetime('now','TimeZone','local'));
                obj.DLog.addData(result)
                drawnow limitrate
                waitfor(obj.Timer)
            end
            obj.SMgr.stop();
            disp('Exporting data.');
            obj.DLog.stop();
            disp('Node session has done.')
        end
    end
    methods(Access=private)
        % function step(obj,~,~)
        %     if ~obj.SMgr.Stopper.UserData, stop(obj.TimerCB); end
        %     [sens,Plant] = obj.Mode.receiveData();
        %     result.sequence_sens = obj.SensSHM.write(obj.encodeSensor(sens,Plant));
        %     [currentCmd, ok] = obj.CmdSHM.read();
        %     if ok
        %         obj.cmd = currentCmd;
        %         obj.Mode.sendData(obj.cmd);
        %         result.sequence_cmd = obj.cmd.sequence;
        %     elseif ~isempty(obj.cmd)
        %         result.sequence_cmd = obj.cmd.sequence;
        %     else
        %         result.sequence_cmd = uint32(0);
        %     end
        %     result.T = posixtime(datetime('now','TimeZone','local'));
        %     obj.DLog.addData(result)
        %     drawnow
        % end
        % % function start(obj,~,~)
        % % 
        % % end
        % function stop(obj,~,~)
        %     c = onCleanup(@()obj.Mode.shutdown());
        %     obj.SMgr.stop();
        %     disp('Exporting data.');
        %     obj.DLog.stop();
        %     disp('Node session has done.')
        %     delete(obj.TimerCB)
        % end
        % function err(obj,~,~)
        %     disp('Error has detected. Stop the system.')
        % end
        function d = encodeSensor(obj, sensor, Plant)
            names = obj.sensorVars(:,1);
        
            % --- LIDAR ---
            if ~isempty(sensor.LIDAR)
                d.(names{1}) = rosReadXYZ(sensor.LIDAR); % single
                d.(names{2}) = int32(sensor.LIDAR.header.stamp.sec);
                d.(names{3}) = uint32(sensor.LIDAR.header.stamp.nanosec);
            else
                d.(names{1}) = single(0);
                d.(names{2}) = int32(0);
                d.(names{3}) = uint32(0);
            end
        
            % --- GNSS ---
            if ~isempty(sensor.GNSS)
                d.(names{4}) = double([sensor.GNSS.latitude; sensor.GNSS.longitude]);
                d.(names{5}) = int32(sensor.GNSS.header.stamp.sec);
                d.(names{6}) = uint32(sensor.GNSS.header.stamp.nanosec);
            else
                d.(names{4}) = double(0);
                d.(names{5}) = int32(0);
                d.(names{6}) = uint32(0);
            end
        
            % --- CAMERA ---
            cam = sensor.CAMERA;
            if ~isempty(cam)
                ndet = numel(cam.detections);
                CAM_detLabel = zeros(1, ndet, 'uint8');
                CAM_detBbox  = zeros(4, ndet, 'uint16');
                CAM_detHue   = zeros(180, ndet, 'uint32');
                CAM_detScore = zeros(1, ndet, 'single');
        
                for i = 1:ndet
                    CAM_detLabel(1,i) = cam.detections(i).label;
                    CAM_detBbox(:,i)  = cam.detections(i).bboxparam;
                    CAM_detHue(:,i)   = cam.detections(i).hue;
                    CAM_detScore(1,i) = cam.detections(i).score;
                end
        
                d.(names{7})  = CAM_detLabel;
                d.(names{8})  = CAM_detBbox;
                d.(names{9})  = CAM_detHue;
                d.(names{10}) = CAM_detScore;
                d.(names{11}) = uint8(cam.processed_masks);
                d.(names{12}) = int32(cam.header.stamp.sec);
                d.(names{13}) = uint32(cam.header.stamp.nanosec);
            else
                d.(names{7})  = uint8(0);
                d.(names{8})  = uint16(0);
                d.(names{9})  = uint32(0);
                d.(names{10}) = single(0);
                d.(names{11}) = uint8(0);
                d.(names{12}) = int32(0);
                d.(names{13}) = uint32(0);
            end
        
            % --- Plant ---
            Plantarray = zeros(10,1,'double');
            Plantarray(1:6) = [Plant.X; Plant.Y; Plant.Z; Plant.Roll; Plant.Pitch; Plant.Yaw];
            for i = 1:min(numel(Plant.odom), 4)
                Plantarray(6 + i) = Plant.odom(i);
            end
            d.(names{14}) = Plantarray;
        end
    end
end
