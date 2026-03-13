classdef Go2Factory < mode.VehicleFactory
    methods
        function modes = getSupportedModes(obj)
            % Define supported mode in this vehicle
            modes = [0,2,3]; 
            % 0: Numerical Sim.
            % 1: Offline
            % 2: Gazebo
            % 3: Real Exp.
        end
        function modeObj = buildNumericalSM(obj, cfg)
            modeObj = mode.Go2.NumericalSMMode(cfg);
        end
        % function modeObj = buildOffline(obj, cfg)
        %     modeObj = mode.CR.OfflineMode(cfg);
        % end
        function modeObj = buildGazebo(obj, cfg)
            modeObj = mode.Go2.GazeboMode(cfg);
        end
        function modeObj = buildReal(obj, cfg)
            modeObj = mode.Go2.RealMode(cfg);
        end
    end
end