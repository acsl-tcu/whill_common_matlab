% Global configurations
clc; close all; clear global; clear variables; warning('off','all');
conf.pc = [ismac; isunix; ispc];
conf.mk = [":"; ":"; ";"];
conf.def= "AppData/Local/Temp/Editor";
conf.usr= pwd;
conf.fpath = split(path, conf.mk(conf.pc));
conf.fcheck= and(~contains(conf.fpath, matlabroot), ~contains(conf.fpath, conf.def));
rmpath(strjoin(conf.fpath(conf.fcheck), conf.mk(conf.pc)));
addpath(conf.usr);

% Control target
vehicleInfo.type = "CR";

% Sensor configurations
vehicleInfo.color = 'red'; % CR series EXP Only 'red'\'white'\'blue'\'green'
vehicleInfo.sensor(1) = true; % LiDAR
vehicleInfo.sensor(2) = false; % GNSS
vehicleInfo.sensor(3) = false; % Camera
vehicleInfo.sensor(4) = false; % SLAM
vehicleInfo.sensor(5) = true; % Matching (EXP Only)
vehicleInfo.sensor(6) = false; % IMU (CR2 Only)
vehicleInfo.base_sensor = 1; % Standard sensor you use mainly. No standard:0, LiDAR:1, GNSS:2, Camera:3
vehicleInfo.manualCon = false; % Enable Manual Control using Joystick (CR Only)

% Mode configurations
mode = 3; % 2:Gazebo simulation, 3:Real exp.

% ROS2 configurations
RID = 11;

cfg = struct( ...
    "modeNumber"  , mode, ...
    "isParallel"  , true, ...
    "isMultiPC"   , false, ...
    "RID"         , RID, ...
    "vehicleInfo" , vehicleInfo, ...
    "sharedMemKey", "matlab_SHM_", ...
    "rosNamespace", "matlab");

% Save file path
mySavePath = './data';
mySaveFileName = string(datetime("now","Format","yyyyMMdd_HH")); % "yyyyMMdd_HHmmss"
Datadir = strcat(mySavePath,filesep,string(datetime("now","Format","yyyyMMdd")),filesep,mySaveFileName);
if ~exist(Datadir,"dir"), mkdir(Datadir); end

%% In case isParalell=true, you should setup the 3 matlab session, then you can run systems of each session
%% NodeMgr session
clc; close all;
clear app

cfg.tspan = 0.05; % Sensor frequency which is corresponded to standard sensor
cfg.overrunAct = 'slip'; % 'slip'(recommend) or 'drop'
cfg.logger = logger.DataLogger(Datadir,"Node");

% Run app
NodeApp = "app." + vehicleInfo.type + ".NodeMgrApp";
app = feval(NodeApp, cfg);
app.run();

%% Estimator session
clc; close all;
clear app
% Activation paralell worker
% if isempty(gcp('nocreate')), parpool; end

cfg.tspan = 0.05; % Session frequency
cfg.overrunAct = 'slip'; % 'slip'(recommend) or 'drop'

offlinePath = "path/to/your/userLocal.mat"; % No use but needed
% addpath(genpath("./MyPkg"))
cfg.estimator = estimator.Estimate2(mode,offlinePath);
cfg.logger = logger.DataLogger(Datadir,"Estimate");

% Run app
EstApp = "app." + vehicleInfo.type + ".EstimatorApp";
app = feval(EstApp, cfg);
app.run();

%% Controller session
clc; close all;
clear app
% Activation paralell worker
% if isempty(gcp('nocreate')), parpool; end

cfg.tspan = 0.05; % Session frequency
cfg.overrunAct = 'slip'; % 'slip'(recommend) or 'drop'

% addpath(genpath("./MyPkg"))
cfg.controller = controller.Control2();
cfg.logger = logger.DataLogger(Datadir,"Control");

% Run app
CtrlApp = "app." + vehicleInfo.type + ".ControllerApp";
app = feval(CtrlApp, cfg);
app.run();

%% Plot result
plotter.DataPlotter(Datadir,cfg)


%% Delete SHM
% Execute below to delete shared memory file if not working program.
% ======Ubuntu=======
% rm /tmp/matlab_SHM*
% ======Windows======
% del %TEMP%\matlab_SHM*


%% Delete Session Manager SHM
% DO NOT USE except for critical communication bug of each matlab session.
% delete(tempdir, "matlab_smgr.bin");
