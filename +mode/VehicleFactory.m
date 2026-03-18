classdef (Abstract) VehicleFactory < handle
    methods (Abstract)
        modes = getSupportedModes(obj)
    end
    
    methods
        % Default action
        function modeObj = buildNumericalSM(obj, cfg)
            error('%s: NumericalSim is not supported.', class(obj));
        end
        function modeObj = buildOffline(obj, cfg)
            error('%s: Offline is not supported.', class(obj));
        end
        function modeObj = buildGazebo(obj, cfg)
            error('%s: Gazebo is not supported.', class(obj));
        end
        function modeObj = buildReal(obj, cfg)
            error('%s: RealEXP is not supported.', class(obj));
        end
        
        % Build mode
        function modeObj = build(obj, cfg)
            if ~ismember(cfg.modeNumber, obj.getSupportedModes())
                error('%s: modeNumber=%d is not supported.\n Supported mode: %s', ...
                    class(obj), cfg.modeNumber, ...
                    num2str(obj.getSupportedModes()));
            end
            switch cfg.modeNumber
                case 0; modeObj = obj.buildNumericalSM(cfg);
                case 1; modeObj = obj.buildOffline(cfg);
                case 2; modeObj = obj.buildGazebo(cfg);
                case 3; modeObj = obj.buildReal(cfg);
            end
        end
    end
end