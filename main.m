%% Global configurations
clc; close all; clear global; clear variables; warning('off','all');
conf.pc = [ismac; isunix; ispc];
conf.mk = [":"; ":"; ";"];
conf.def= "AppData/Local/Temp/Editor";
conf.usr= pwd;
conf.fpath = split(path, conf.mk(conf.pc));
conf.fcheck= and(~contains(conf.fpath, matlabroot), ~contains(conf.fpath, conf.def));
rmpath(strjoin(conf.fpath(conf.fcheck), conf.mk(conf.pc)));
%% Sensor configurations
vehicleType = 1; % 1:CR1, 2:CR2
vehicleColor = 'red'; % Only EXP. 'red' or 'white'
sensor(1) = true; % LiDAR
sensor(2) = false; % GNSS
sensor(3) = false; % Camera
sensor(4) = true; % SLAM
sensor(5) = false; % Matching (Only EXP)
base_sensor = 1; % Standard sensor you use mainly. No standard:0, LiDAR:1, GNSS:2, Camera:3
tspan = 0.05; % Sensor frequency which is corresponded to standard sensor
%% Mode configurations
mode = 0; % 0:Numerical simulation, 1:Offline, 2:Gazebo simulation, 3:Real exp.
% Offline: Path to MAT file
% The number of time series data points required for execution is automatically detected.
offlinePath = "/path/to/your/userLocal.mat"; 
tend = 20; % Offline: If NO MAT file is loaded, set any time value (sec).
isParallel = false;
isMultiPC = false;
%% ROS2 configurations
RID = 11;
%% Enable Manual Control using Joystick
manualCon = false;
%% Module configurations
% Save file path
mySavePath = './data';
mySaveFileName = string(datetime("now","Format","yyyyMMdd_HHmmss")); 
Datadir = strcat(mySavePath,filesep,string(datetime("now","Format","yyyyMMdd")),filesep,mySaveFileName);
if ~exist(Datadir,"dir"), mkdir(Datadir); end

% LiDAR Camera Caliblation File for Fusion
calibparamPath = "./cameracalibparam/prefer.mat";
cameraparamPath = "./cameracalibparam/internal_param_fix.mat";

% You can supply your own class instead of default if you need.
% addpath(genpath("./MyEstimate"))
plantcontainer = plant.PlantWH();
estimator = estimator.Estimate2(mode,offlinePath);
% Camera LiDAR Fusion package
% addpath(genpath("./LiDARCamera"))
% estimator = estimator.EstimateLC(mode,offlinePath,calibparamPath,cameraparamPath);
controller = controller.ControlDiff();
logger = logger.DataLogger(Datadir,'tmp');

% Activation paralell worker
% if isempty(gcp('nocreate')), parpool; end

cfg = struct( ...
    "modeNumber"  , mode, ...
    "isParallel"  , isParallel, ...
    "isMultiPC"   , isMultiPC, ...
    "RID"         , RID, ...
    "sensorIdx"   , sensor, ...
    "base_sensor" , base_sensor, ...
    "vehicleType" , vehicleType, ...
    "vehicleColor", vehicleColor, ...
    "tspan"       , tspan, ...
    "offlinePath" , offlinePath, ...
    "tend"        , tend, ...
    "rosNamespace", "matlab", ...
    "plant"       , plantcontainer, ...
    "estimator"   , estimator, ...
    "controller"  , controller, ...
    "logger"      , logger, ...
    "manualCon"   , manualCon);

sys = core.SystemFactory.build(cfg);
sys.run();

%% Plot
plotter.DataPlotter(Datadir,cfg)

