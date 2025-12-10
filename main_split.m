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

% Sensor configurations
vehicleType = 1; % 1:CR1, 2:CR2
vehicleColor = 'red'; % Only EXP. 'red' or 'white'
sensor(1) = true; % LiDAR
sensor(2) = false; % GNSS
sensor(3) = false; % Camera
sensor(4) = true; % SLAM/Localization
sensor(5) = false; % Matching (Only EXP)
base_sensor = 1; % Standard sensor you use mainly. No standard:0, LiDAR:1, GNSS:2, Camera:3
tspan = 0.05; % Sensor frequency which is corresponded to standard sensor
overrunAct = 'slip'; % 'slip'(recommend) or 'drop'

% Mode configurations
mode = 3; % 1:Offline, 2:Gazebo simulation, 3:Real exp.
offlinePath = "/home/student/Program/matlab_common/data/20250613/20250613_171110/userLocal.mat";

% ROS2 configurations
RID = 11;

% Enable Manual Control using Joystick
manualCon = false;

cfg = struct( ...
    "modeNumber"  , mode, ...
    "isParallel"  , true, ...
    "isMultiPC"   , false, ...
    "RID"         , RID, ...
    "sensorIdx"   , sensor, ...
    "base_sensor" , base_sensor, ...
    "vehicleType" , vehicleType, ...
    "vehicleColor", vehicleColor, ...
    "tspan"       , tspan, ...
    "offlinePath" , offlinePath, ...
    "sharedMemKey", "matlab_SHM_", ...
    "rosNamespace", "matlab", ...
    "manualCon"   , false, ...
    "overrunAct"  , overrunAct);

% Save file path
mySavePath = './data';
mySaveFileName = string(datetime("now","Format","yyyyMMdd_HH")); % "yyyyMMdd_HHmmss"
Datadir = strcat(mySavePath,filesep,string(datetime("now","Format","yyyyMMdd")),filesep,mySaveFileName);
if ~exist(Datadir,"dir"), mkdir(Datadir); end

%% In case isParalell=true, you should setup the 3 matlab session, then you can run systems of each session
%% NodeMgr session
clc; close all;
clear app
cfg.logger = logger.DataLogger(Datadir,"Node");
app = app.NodeMgrApp(cfg);
app.run();

%% Estimator session
clc; close all;
clear app
% Activation paralell worker
% if isempty(gcp('nocreate')), parpool; end
% addpath(genpath("./MyPkg"))
cfg.estimator = estimator.Estimate2(mode,offlinePath);
cfg.logger = logger.DataLogger(Datadir,"Estimate");
app = app.EstimatorApp(cfg);
app.run();

%% Controller session
clc; close all;
clear app
% Activation paralell worker
% if isempty(gcp('nocreate')), parpool; end
% addpath(genpath("./MyPkg"))
cfg.controller = controller.Control2();
cfg.logger = logger.DataLogger(Datadir,"Control");
app = app.ControllerApp(cfg);
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
