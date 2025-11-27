classdef PlantWH < handle
    %% Controller
    %% Method
    % Control2: 初回のみ呼び出されるMethod．このクラスで用いる変数の初期定義を行う．
    % Main: 毎時刻呼び出されるMethod． 実行する推定プログラムを作る．
    %% 制御結果の保存
    % "result"を構造体とし，保存したい値を格納．
    properties(Constant)
        dt = 0.05; % 時間の更新(50 ms)
    end

    properties
        Pose
        U
    end

    methods
        function [obj] = PlantWH()
            %%%%%% プラントモデルの初期位置
            obj.Pose.x = 0;
            obj.Pose.y = 0;
            obj.Pose.yaw = 0;

            obj.Pose.xpr = obj.Pose.x;
            obj.Pose.ypr = obj.Pose.y;
            obj.Pose.yawpr = obj.Pose.yaw;
        end
        function [obj,PlantDATA] = main(obj,Uc)
            %%%%%% プラントモデルの状態更新
            obj.Pose.x = obj.Pose.xpr + Uc.V(1)*cos(obj.Pose.yawpr)*obj.dt;
            obj.Pose.y = obj.Pose.ypr + Uc.V(1)*sin(obj.Pose.yawpr)*obj.dt;
            obj.Pose.yaw = obj.Pose.yawpr + Uc.V(2)*obj.dt;

            obj.Pose.xpr = obj.Pose.x;
            obj.Pose.ypr = obj.Pose.y;
            obj.Pose.yawpr = obj.Pose.yaw;

            PlantDATA.X = obj.Pose.x;
            PlantDATA.Y = obj.Pose.y;
            PlantDATA.Z = nan;
            PlantDATA.Roll = nan;
            PlantDATA.Pitch = nan;
            PlantDATA.Yaw = obj.Pose.yaw;
            PlantDATA.odom = [nan,nan];
        end
    end
end
