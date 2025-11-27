classdef ControllerApp < handle
    properties
        Ctrl
        EstSHM
        CmdSHM
        Timer
        DLog
        SMgr
        % TimerCB
        Spr
        est
        checkBin = 1;
        
    end
    properties (Constant)
        ctrlVars = {'V','double',[2,1]};
        
    end

    methods
        function obj = ControllerApp(cfg)
            obj.Ctrl  = cfg.controller;
            
            fname = strcat(tempdir,cfg.sharedMemKey);
            obj.CmdSHM = bridge.SharedMem(fname+"cmd.bin",obj.checkBin,obj.ctrlVars);
            obj.EstSHM  = bridge.SharedMem(fname+"est.bin",obj.checkBin);            
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
            disp('Controller is ready to run and waiting for a Node session.')
            obj.SMgr.start();
            disp('Executing session...');
            reset(obj.Timer)
            % start(obj.TimerCB)
            while obj.SMgr.isWorking
                [obj.est, ok] = obj.EstSHM.read();
                if ok || ~isempty(obj.est) % Reader finished correctly
                    result = obj.Ctrl.main(obj.est,obj.Timer.TotalElapsedTime);
                    d = struct('V',result.V,'sequence',obj.est.sequence);
                    obj.CmdSHM.write(d,obj.est.sequence);
                    result.T = posixtime(datetime('now','TimeZone','local'));
                    result.sequence = obj.est.sequence;
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
        %     [obj.est, ok] = obj.EstSHM.read();
        %     if ok || ~isempty(obj.est) % Reader finished correctly
        %         result = obj.Ctrl.main(obj.est,obj.Timer.elapsed());
        %         d = struct('V',result.V,'sequence',obj.est.sequence);
        %         obj.CmdSHM.write(d,obj.est.sequence);
        %         result.T = posixtime(datetime('now','TimeZone','local'));
        %         result.sequence = obj.est.sequence;
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
    end
end
