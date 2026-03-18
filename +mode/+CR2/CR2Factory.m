classdef CR2Factory < mode.VehicleFactory
    methods
        function modes = getSupportedModes(obj)
            % Define supported mode in this vehicle
            modes = [3]; 
            % 0: Numerical Sim.
            % 1: Offline
            % 2: Gazebo
            % 3: Real Exp.
        end
        function modeObj = buildReal(obj, cfg)
            modeObj = mode.CR2.RealMode(cfg);
        end
    end
end