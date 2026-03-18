classdef (Abstract) ROS2CommStrategy < handle
    %ABSTRACTROS2COMMMANAGER ROS2通信管理の抽象基底クラス
    %   各機体固有の実装は本クラスを継承して作成する
    
    properties (Constant)
        % 共通QoSプロファイル
        DEFAULT_QOS = struct("Reliability","reliable","Durability","volatile",...
                           "History","keeplast","Depth",1);
        BESTEFFORT_QOS = struct("Reliability","besteffort","Durability","volatile",...
                              "History","keeplast","Depth",1);
    end
    
    properties (Access = protected)
        node            % ROS2ノード
        vehicleType     % 機体タイプ
        mode           % 動作モード (実機/シミュレータ等)
        sensorIdx      % センサー有効フラグ
        
        % Publisher/Subscriber
        sensorSubs     % センサーデータ用Subscriber
        vehiclePubs    % 機体制御用Publisher
        vehicleSubs    % 機体状態用Subscriber
        vehicleMsg     % 機体制御Publisherメッセージ
        
        % メタデータ管理
        % varpub         % 変数情報送信用
        % varmsg         % 変数情報メッセージ
        
        % 制御コマンド管理
        cmdMissed = 0  % コマンド欠損カウンタ
        prevCmd        % 前回コマンド
        
        % センサー管理
        sensorFetcher  % センサーデータ取得Object
    end
    
    methods (Abstract)
        % 各機体で実装必須のメソッド
        % 機体ごとに異なるためサブクラスで具体的に実装する
        topics = getSensorTopics(obj)           % センサートピック定義
        types = getSensorMsgTypes(obj)          % センサーメッセージ型定義
        topics = getVehicleSubsTopics(obj)          % 機体制御Subscriberトピック定義
        types = getVehicleSubsMsgTypes(obj)         % 機体制御Subscriberメッセージ型定義
        topics = getVehiclePubsTopics(obj)          % 機体制御Publisherトピック定義
        types = getVehiclePubsMsgTypes(obj)         % 機体制御Publisherメッセージ型定義
        qos = getQoSProfile(obj, topicType)     % QoSプロファイル取得
        msg = createVehicleCommand(obj, cmd)    % 機体制御コマンド作成
        cmd = createSafetyStopCommand(obj);     % 緊急時停止コマンド作成
    end
    
    methods
        % 【共通機能】全ての機体で同じ処理を行うメソッド
        % サブクラスはこれらをそのまま使用できる（上書きも可能）        
        function obj = ROS2CommStrategy(vehicleInfo, mode, nodeName, rid)
            obj.vehicleType = vehicleInfo.type;
            obj.mode = mode;
            obj.sensorIdx = vehicleInfo.sensor;
            obj.node = obj.createNode(nodeName, rid);
        end
        
        function generateSensorSubscribers(obj)
            % センサーデータ用Subscriber群を生成
            % 機体固有のトピック・型情報を使って共通の手順でSubscriber作成
            topics = obj.getSensorTopics();    % 機体固有メソッドを呼び出し
            msgTypes = obj.getSensorMsgTypes(); % 機体固有メソッドを呼び出し
            
            if length(topics) ~= length(msgTypes)
                error('AbstractROS2CommManager:TopicMismatch', ...
                      'Number of topics and message types must match');
            end
            
            n = length(topics);
            obj.sensorSubs = cell(n,1);
            
            for i = 1:n
                if ~isempty(topics{i}) && obj.sensorIdx(i)
                    qos = obj.getQoSProfile('sensorSubs'); % 機体固有メソッドを呼び出し
                    obj.sensorSubs{i} = ros2subscriber(obj.node, ...
                        topics{i}, msgTypes{i}, ...
                        "Reliability", qos.Reliability, ...
                        "Durability", qos.Durability, ...
                        "History", qos.History, ...
                        "Depth", qos.Depth);
                else
                    obj.sensorSubs{i} = [];
                end
            end
        end
        
        function generateVehicleSubscribers(obj)
            % 機体状態受信用Subscriberを生成
            topics = obj.getVehicleSubsTopics();
            msgTypes = obj.getVehicleSubsMsgTypes();
            qos = obj.getQoSProfile('vehicleSubs');

            n = length(topics);
            obj.vehicleSubs = cell(n,1);
            
            for i = 1:n
                obj.vehicleSubs{i} = ros2subscriber(obj.node, ...
                    topics{i}, msgTypes{i}, ...
                    "Reliability", qos.Reliability, ...
                    "Durability", qos.Durability, ...
                    "History", qos.History, ...
                    "Depth", qos.Depth);
            end
        end
        
        function generateVehiclePublishers(obj)
            % 機体制御用Publisherを生成
            topics = obj.getVehiclePubsTopics();
            msgTypes = obj.getVehiclePubsMsgTypes();
            qos = obj.getQoSProfile('vehiclePubs');
            
            obj.vehiclePubs = ros2publisher(obj.node, ...
                topics{1}, ...
                msgTypes{1}, ...
                "Reliability", qos.Reliability, ...
                "Durability", qos.Durability, ...
                "History", qos.History, ...
                "Depth", qos.Depth);            
            obj.vehicleMsg = ros2message(obj.vehiclePubs);
            
        end
        
        function sendVehicleCommand(obj, cmd)
            %機体制御コマンドを送信
            
            % コマンド欠損チェック
            if ~isempty(obj.prevCmd) && cmd.sequence == obj.prevCmd.sequence
                obj.cmdMissed = obj.cmdMissed + 1;
            else
                obj.prevCmd = cmd;
                obj.cmdMissed = 0;
            end
            
            % セーフティチェック
            if obj.cmdMissed >= 5 && obj.mode == 3 % Only Real Exp. mode
                cmd = obj.createSafetyStopCommand();
                warning('AbstractROS2CommManager:CommandTimeout', ...
                        'Control command timeout. Applying safety stop.');
            end
            
            % 機体固有のコマンド作成と送信
            obj.vehicleMsg = obj.createVehicleCommand(cmd);
            send(obj.vehiclePubs, obj.vehicleMsg);
        end
        
        function [sensorData, Plant] = getSensorData(obj)
            %センサーデータと機体状態を取得
            if ~isempty(obj.sensorFetcher)
                [sensorData, Plant] = obj.sensorFetcher.getSensorData(...
                    obj.sensorSubs, obj.vehicleSubs);
            else
                sensorData = [];
                Plant = [];
            end
        end
    end
    
    methods (Static, Access = protected)
        function n = createNode(nodeName, rid)
            persistent sNode;
            if isempty(sNode) || ~isvalid(sNode)
                fprintf('Establishing ROS2 Node: %s\n', nodeName);
                sNode = ros2node(nodeName, rid);
            end
            n = sNode;
        end
    end
end