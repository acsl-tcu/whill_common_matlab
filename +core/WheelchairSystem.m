classdef WheelchairSystem < handle
    properties (Access = private)
        Est
        Ctrl
        DLog
        DLogC
        StepTimer   
        Mode        
        Bridge      
        SMgr
        Timer
        cnt = 0;
        ctrlFB
    end

    methods
        function obj = WheelchairSystem(cfg, modeObj, SMgr)
            arguments
                cfg struct
                modeObj {mustBeA(modeObj, "mode.ModeStrategy")}
                SMgr {mustBeA(SMgr, "session.SessionStrategy")}
            end
            obj.Est   = cfg.estimator;
            obj.Ctrl  = cfg.controller;
            obj.DLog  = cfg.logger;
            obj.Timer = rateControl(1/cfg.tspan);
            obj.Timer.OverrunAction = cfg.overrunAct; % 'slip' or 'drop'
            obj.Mode  = modeObj;
            obj.SMgr  = SMgr;
        end

        function run(obj)
            c = onCleanup(@()obj.Mode.shutdown());
            obj.Mode.setup();
            obj.SMgr.start();
            reset(obj.Timer);
            while obj.SMgr.isWorking(obj.cnt+1)
                obj.cnt = obj.cnt + 1;
                % Main process
                [sens,Plant] = obj.Mode.receiveData();
                [resEst,sendData]  = obj.Est.main(sens, Plant,obj.Timer.TotalElapsedTime);
                resEst.T = posixtime(datetime('now','TimeZone','local'));
                [resCtrl]  = obj.Ctrl.main(sendData,obj.Timer.TotalElapsedTime);
                resCtrl.T = posixtime(datetime('now','TimeZone','local'));
                cmd = struct('V',resCtrl.V,'sequence',obj.cnt);
                obj.Mode.sendData(cmd);

                % Post process
                resEst.send = sendData;
                resEst.sequence = obj.cnt;
                resCtrl.sequence = obj.cnt;
                result = struct('Estimator',resEst, 'Controller',resCtrl);
                obj.DLog.addData(result);
                drawnow limitrate
                waitfor(obj.Timer);
            end
            obj.SMgr.stop();
            obj.DLog.stop();
        end
    end
end
