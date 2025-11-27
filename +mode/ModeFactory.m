classdef ModeFactory
    methods (Static)
        function modeObj = build(cfg)
            switch cfg.modeNumber
                case 0
                    modeObj = mode.NumericalSMMode(cfg);
                case 1
                    modeObj = mode.OfflineMode(cfg);
                case 2
                    modeObj = mode.GazeboMode(cfg);
                case 3
                    modeObj = mode.RealMode(cfg);
                otherwise
                    error("unknown modeNumber");
            end
        end
    end
end
