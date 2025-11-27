classdef Estimate2 < handle
    %% Estimator
    %% Method
    % Estimate2: 初回のみ呼び出されるMethod．このクラスで用いる変数の初期定義を行う．
    % Main: 毎時刻呼び出されるMethod． 実行する推定プログラムを作る．
    %% Controllerへの推定値送信
    % "SendVarSpec"を構造体とし，推定値の変数名とその型を定義する(以下の中から選択)．
    % Controllerで同じ変数名として取り出すことが可能．
    % 型によって配列/行列への対応可否があります:
    % 行列可: {'int8','uint8','int16','uint16','int32','uint32','int64','uint64','single','double'}
    % 不可: {'string','char'}
    % "send"を構造体とし，送信したいデータを対応する変数に代入．
    %% 推定結果の保存
    % "result"を構造体とし，保存したい値を格納．

    properties (Constant)

    end

    properties
        %=======DO NOT DELETE======
        % Send variable type set to Controller
        % Example:
        SendVarSpec = struct( ...
                            'pose' , struct('Type','double','MaxSize',[1 3]));
        %==========================
        Allxhat = 1 % Example
        mode
        udd


    end

    methods
        function obj = Estimate2(mode,OfflinePath)            
            obj.mode = mode;
            
            if obj.mode == 1
                % Load matfile
                % obj.udd = load(OfflinePath);
            end

        end

        function [result,send] = main(obj,sensordata,Plant,T)
            result.RawData = sensordata;
            result.Plant = Plant;
            
            % save data
            result.xhat = obj.Allxhat; % example
            
            

            % send to Controller
            send.pose = [Plant.X, Plant.Y, Plant.Yaw];
            % send = [];
            



        end


    end




end