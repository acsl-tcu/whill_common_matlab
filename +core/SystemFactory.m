classdef SystemFactory
    methods (Static)
        function factory = createVehicleFactory(cfg)
            factoryName = cfg.vehicleInfo.type + "Factory";
            if exist(factoryName, 'class')
                factory = feval(factoryName);
            else
                error("Unknown vehicle type: %s", factoryName);
            end
            % switch cfg.vehicleInfo.type
            %     case "CR" 
            %         factory = CRFactory();
            %     case "CR2"  
            %         factory = CR2Factory();
            %     case "Go2"
            %         factory = Go2Factory();
            %     otherwise
            %         error("unknown vehicle type");
            % end
        end
        function sys = build(cfg)
            import mode.*;
            import bridge.*;
            import session.*;

            factoryName = cfg.vehicleInfo.type + "." + cfg.vehicleInfo.type + "Factory";
            if exist(factoryName, 'class')
                factory = feval(factoryName);
            else
                error("Unknown vehicle type: %s", factoryName);
            end
            
            % modeObj = mode.ModeFactory.build(cfg);
            modeObj = factory.build(cfg);
            SMgrObj = session.SessionManager.build(cfg,session.SessionManager.initiator);

            % --- System 本体 ---
            % sys = core.WheelchairSystem(cfg, modeObj, SMgrObj);
            sys = core.VehicleSystem(cfg, modeObj, SMgrObj);
        end
    end
end
