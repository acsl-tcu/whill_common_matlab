classdef EstimatorApp < handle
    properties
        Est
        SensSHM
        EstSHM
        Timer
        estVars
        DLog
        sensordata
        Plant
        SMgr
        % TimerCB
        Spr
        seq
        checkBin = 1;

    end
    properties (Constant)
        



    end
    methods
        function obj = EstimatorApp(cfg)
            obj.Est    = cfg.estimator;
            EstVarSpec = obj.Est.SendVarSpec;

            f = fieldnames(EstVarSpec);
            n = numel(f);
            tmp = cell(n,3);
            for k = 1:n
                info        = EstVarSpec.(f{k});
                tmp{k,1}    = f{k};        % Variable Name
                tmp{k,2}    = info.Type;     % Type
                tmp{k,3}    = info.MaxSize;  % Max Size
            end
            obj.estVars = tmp;
            fname = strcat(tempdir,cfg.sharedMemKey);
            obj.EstSHM  = bridge.SharedMem(fname+"est.bin",obj.checkBin,obj.estVars);
            obj.SensSHM = bridge.SharedMem(fname+"sensor.bin",obj.checkBin);            
            obj.Timer  = rateControl(1/cfg.tspan);
            obj.DLog = cfg.logger;
            obj.SMgr = session.SessionManager.build(cfg,session.SessionManager.participant);
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
            disp('Estimator is ready to run and waiting for a Node session.')
            obj.SMgr.start();
            disp('Executing session...');
            reset(obj.Timer)
            % start(obj.TimerCB)
            while obj.SMgr.isWorking
                [sens, ok] = obj.SensSHM.read();
                if ok % Reader finished correctly
                    [obj.sensordata,obj.Plant] = obj.decodeSensor(sens);
                    obj.seq = sens.sequence;
                end
                if ~isempty(obj.sensordata) && ~isempty(obj.Plant)
                    [result,sendData] = obj.Est.main(obj.sensordata, obj.Plant,obj.Timer.TotalElapsedTime);  % Estimator
                    sendData.sequence = obj.EstSHM.write(sendData,obj.seq); % Send to Controller
                    result.send = sendData;
                    result.T = posixtime(datetime('now','TimeZone','local'));
                    obj.DLog.addData(result);
                end
                waitfor(obj.Timer)
            end
            disp('Exporting data.');
            obj.DLog.stop();
            disp('Finished.')
        end
        
    end
    methods(Access=private)
        % function step(obj,~,~)
        %     if ~obj.SMgr.isWorking, stop(obj.TimerCB); end
        %     [sens, ok] = obj.SensSHM.read();
        %     if ok % Reader finished correctly
        %         [obj.sensordata,obj.Plant] = obj.decodeSensor(sens);
        %         obj.seq = sens.sequence;
        %     end
        %     if ~isempty(obj.sensordata) && ~isempty(obj.Plant)
        %         [result,sendData] = obj.Est.main(obj.sensordata, obj.Plant,obj.Timer.elapsed());  % Estimator
        %         sendData.sequence = obj.EstSHM.write(sendData,obj.seq); % Send to Controller
        %         result.send = sendData;
        %         result.T = posixtime(datetime('now','TimeZone','local'));
        %         obj.DLog.addData(result);
        %     end
        % end
        % function stop(obj,~,~)
        %     disp('Exporting data.');
        %     obj.DLog.stop();
        %     disp('Finished.')
        % end
        % function err(obj,~,~)
        %     disp('Error has detected. Stop the system.')
        % end
        function [sensordata, Plant] = decodeSensor(obj,d)
            names = fieldnames(d);
            sensordata = struct("LIDAR",[],"GNSS",[],"CAMERA",[]);
            Plant = struct("X",[.0],"Y",[.0],"Z",[.0], ...
                "Roll",[.0],"Pitch",[.0],"Yaw",[.0],"odom",[.0]);
            sensordata.LIDAR.data = d.(names{1});
            sensordata.LIDAR.sec = d.(names{2});
            sensordata.LIDAR.nanosec = d.(names{3});
            sensordata.GNSS.data = d.(names{4});
            sensordata.GNSS.sec = d.(names{5});
            sensordata.GNSS.nanosec = d.(names{6});
            
            label = d.(names{7});
            bbox = d.(names{8});
            hue = d.(names{9});
            score = d.(names{10});
            ndet = numel(d.CAM_detLabel);
            for i = 1:ndet                
                sensordata.CAMERA.detections(i).label = label(1,i);
                sensordata.CAMERA.detections(i).bboxparam = bbox(:,i);
                sensordata.CAMERA.detections(i).hue = hue(:,i);
                sensordata.CAMERA.detections(i).score = score(:,i);                
            end
            sensordata.CAMERA.processed_masks = d.(names{11});
            sensordata.CAMERA.sec = d.(names{12});
            sensordata.CAMERA.nanosec = d.(names{13});

            Plantarray = d.(names{14});
            Plant.X = Plantarray(1);
            Plant.Y = Plantarray(2);
            Plant.Z = Plantarray(3);
            Plant.Roll = Plantarray(4);
            Plant.Pitch = Plantarray(5);
            Plant.Yaw = Plantarray(6);
            Plant.odom(1) = Plantarray(7);
            Plant.odom(2) = Plantarray(8);
            if numel(Plantarray) > 8
                Plant.odom(3) = Plantarray(9);
                Plant.odom(4) = Plantarray(10);
            end
        end
    end
end
