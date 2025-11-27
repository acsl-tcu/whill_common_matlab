function [ObjectData,Objectpt,new_labels,ObjectData_fromCamera,ObjectData_fromLiDAR] = LiDARcamerafusion(allptCloud,ptCloudLCS, ptCloud, camdata, intrinsics, tform, scorethreshold, labels)
    
    ObjectLidarPoints = [];
    OLPLabels = [];
    ObjectData_fromCamera = {};
    ObjectData_fromLiDAR = {};
    ObjectData = {};
    
    [~, validIndex] = projectLidarPointsOnImage(allptCloud, intrinsics, tform);
    
    alllidarPoints = allptCloud.Location(validIndex, :);

    if ~isempty(ptCloudLCS)
        [imagePoints, validIndex] = projectLidarPointsOnImage(ptCloudLCS, intrinsics, tform);
        
        lidarPoints = ptCloudLCS(validIndex, :);
    
        if isempty(camdata)
            num_de = 0;
        else
            num_de = length(camdata.detections);
        end
        labelsidx = nonzeros(unique(labels));
        num_clus = length(labelsidx);
        num_obj = 0;
    
        for ig = 1 : num_de
            if camdata.detections(ig).score>=scorethreshold
    
                bboxparam = camdata.detections(ig).bboxparam;   % [左上x1; 左上y1; 右下x2; 右下y2]
                cx = (bboxparam(1) + bboxparam(3)) / 2;
                cy = (bboxparam(2) + bboxparam(4)) / 2;
                w=bboxparam(3)-bboxparam(1);
                h=bboxparam(4)-bboxparam(2);
    
                label = camdata.detections(ig).label;
                bbox = [cx,cy,w,h]; %bbox中心xy座標
                Color = camdata.detections(ig).hue;    % 1×180の色相ヒストグラム
                mask = (camdata.processed_masks == ig);
                
                imagePointsInt = round(imagePoints);
    
                [imageHeight, imageWidth, ~] = size(mask);
    
                validPoints = imagePointsInt(:,1) >= 1 & imagePointsInt(:,1) <= imageWidth & ...
                              imagePointsInt(:,2) >= 1 & imagePointsInt(:,2) <= imageHeight;
    
                imagePointsInt = imagePointsInt(validPoints, :);
                lidarPointsValid = lidarPoints(validPoints, :);
    
                indices = sub2ind([imageHeight, imageWidth], imagePointsInt(:,2), imagePointsInt(:,1));
                maskValues = mask(indices);
                isObjectPoint = maskValues > 0;
    
                objectLidarPoints = lidarPointsValid(isObjectPoint, :); % マスク内に存在する点群(カメラを基に抜き出された点群)
    
                num_obj = num_obj + 1;
    
                [objlabels,~] = pcsegdist(pointCloud(objectLidarPoints),0.35);
                objectLidarPoints = objectLidarPoints(objlabels==mode(objlabels),:);
    
                ObjectLidarPoints =  [ObjectLidarPoints; objectLidarPoints];
                oLPLabels = num_obj * ones(size(objectLidarPoints,1),1);
                OLPLabels = [OLPLabels; oLPLabels];
    
                ObjectData_fromCamera{num_obj,1} = objectLidarPoints;
                ObjectData_fromCamera{num_obj,2} = label;
                ObjectData_fromCamera{num_obj,3} = Color;
                ObjectData_fromCamera{num_obj,4} = bbox;
                ObjectData_fromCamera{num_obj,5} = mask;
    
            end
        end
        
        num_obj = 0;
    
        for i = 1 : num_clus
            clusterptCloudLCS = pointCloud(ptCloudLCS(labels==labelsidx(i),:)); % クラスタ点群
            if ~isempty(ObjectLidarPoints) % カメラの画角内に点群が存在するか
                if any(min(alllidarPoints(:,1)) <= clusterptCloudLCS.Location(:,1)) && any(max(alllidarPoints(:,1)) >= clusterptCloudLCS.Location(:,1)) && ...
                   any(min(alllidarPoints(:,2)) <= clusterptCloudLCS.Location(:,2)) && any(max(alllidarPoints(:,2)) >= clusterptCloudLCS.Location(:,2)) && ...
                   any(min(alllidarPoints(:,3)) <= clusterptCloudLCS.Location(:,3)) && any(max(alllidarPoints(:,3)) >= clusterptCloudLCS.Location(:,3)) % クラスタ点群がカメラの画角に入っているか
        
                    % クラスタ点群範囲内のカメラで抜き出した点群
                    findidx = ObjectLidarPoints(:,1) >= clusterptCloudLCS.XLimits(1) ...
                            & ObjectLidarPoints(:,1) <= clusterptCloudLCS.XLimits(2) ...
                            & ObjectLidarPoints(:,2) >= clusterptCloudLCS.YLimits(1) ...
                            & ObjectLidarPoints(:,2) <= clusterptCloudLCS.YLimits(2);
        
                    OLP = ObjectLidarPoints(findidx,:); % カメラで抜き出した点群
                    OLPLabel =  OLPLabels(findidx);
                    [OLPlabel,~,OLPlabelidx] = unique(OLPLabel);
                    Obj_num = size(OLPlabel,1); % クラスタ範囲内のカメラで識別した物体数
        
                    if Obj_num > 1 % クラスタ数よりもカメラで識別した物体数の方が多い場合：クラスタを分割
                        OLP_mean = [accumarray(OLPlabelidx, OLP(:,1), [], @mean), accumarray(OLPlabelidx, OLP(:,2), [], @mean), accumarray(OLPlabelidx, OLP(:,3), [], @mean)]; % クラスタ範囲内のカメラで抜き出した点群の重心
                        [labels_k,Clus_mean] = clustering_kmeans(clusterptCloudLCS.Location,Obj_num,OLP_mean); % k-means法によるクラスタリング(クラスタを分割)
                        % ハンガリアン法でクラスタとカメラで抜き出した点群を対応付け
                        cost = pdist2(Clus_mean,OLP_mean,'euclidean');
                        assignment = assignDetectionsToTracks(double(cost),0.5);
                        for j = 1 : Obj_num
                            num_obj = num_obj + 1;
                            ObjectData_fromLiDAR{num_obj,1} = ptCloudLCS(ismember(ptCloudLCS, clusterptCloudLCS.Location(labels_k==j,:), 'rows'),:);
                            ObjectData{num_obj,1} = ptCloud(ismember(ptCloudLCS, clusterptCloudLCS.Location(labels_k==j,:), 'rows'),:);
                            if any(assignment(:,1)==j)
                                ObjectData{num_obj,2} = ObjectData_fromCamera{assignment(assignment(:,1)==j,2),2};
                                ObjectData{num_obj,3} = ObjectData_fromCamera{assignment(assignment(:,1)==j,2),3};
                                ObjectData{num_obj,4} = ObjectData_fromCamera{assignment(assignment(:,1)==j,2),4};
                                ObjectData{num_obj,5} = ObjectData_fromCamera{assignment(assignment(:,1)==j,2),5};
                            else
                                ObjectData{num_obj,2} = [];
                                ObjectData{num_obj,3} = [];
                                ObjectData{num_obj,4} = [];
                                ObjectData{num_obj,5} = [];
                            end
                        end
    
                    elseif Obj_num == 1 % クラスタ数とカメラで識別した物体数が同じ場合
                        num_obj = num_obj + 1;
                        ObjectData_fromLiDAR{num_obj,1} = ptCloudLCS(ismember(ptCloudLCS, clusterptCloudLCS.Location, 'rows'),:);
                        ObjectData{num_obj,1} = ptCloud(ismember(ptCloudLCS, clusterptCloudLCS.Location, 'rows'),:);
                        ObjectData{num_obj,2} = ObjectData_fromCamera{OLPlabel,2};
                        ObjectData{num_obj,3} = ObjectData_fromCamera{OLPlabel,3};
                        ObjectData{num_obj,4} = ObjectData_fromCamera{OLPlabel,4};
                        ObjectData{num_obj,5} = ObjectData_fromCamera{OLPlabel,5};
    
                    elseif Obj_num == 0 % クラスタの一部はカメラの画角に入っているが，YOLOで認識できていないor信頼度スコアが低い場合
                        num_obj = num_obj + 1;
                        ObjectData_fromLiDAR{num_obj,1} = ptCloudLCS(ismember(ptCloudLCS, clusterptCloudLCS.Location, 'rows'),:);
                        ObjectData{num_obj,1} = ptCloud(ismember(ptCloudLCS, clusterptCloudLCS.Location, 'rows'),:);
                        ObjectData{num_obj,2} = [];
                        ObjectData{num_obj,3} = [];
                        ObjectData{num_obj,4} = [];
                        ObjectData{num_obj,5} = [];
                    end
                    
        
                else % カメラの画角にクラスタ点群が入っていなければLiDARの点群をそのまま使う
                    num_obj = num_obj + 1;
                    ObjectData_fromLiDAR{num_obj,1} = ptCloudLCS(ismember(ptCloudLCS, clusterptCloudLCS.Location, 'rows'),:);
                    ObjectData{num_obj,1} = ptCloud(ismember(ptCloudLCS, clusterptCloudLCS.Location, 'rows'),:);
                    ObjectData{num_obj,2} = [];
                    ObjectData{num_obj,3} = [];
                    ObjectData{num_obj,4} = [];
                    ObjectData{num_obj,5} = [];
                    
                end
            else % カメラの画角に点群がなければLiDARの点群をそのまま使う
                num_obj = num_obj + 1;
                ObjectData_fromLiDAR{num_obj,1} = ptCloudLCS(ismember(ptCloudLCS, clusterptCloudLCS.Location, 'rows'),:);
                ObjectData{num_obj,1} = ptCloud(ismember(ptCloudLCS, clusterptCloudLCS.Location, 'rows'),:);
                ObjectData{num_obj,2} = [];
                ObjectData{num_obj,3} = [];
                ObjectData{num_obj,4} = [];
                ObjectData{num_obj,5} = [];          
            end
        end
    end
    if any(any(~cellfun(@isempty, ObjectData)))
        obj4 = ObjectData(:,4);
        isEmpty = cellfun(@isempty, obj4);
        nonEmpty = obj4(~isEmpty);
        % 非空要素を文字列化して比較
        S = cellfun(@(x) mat2str(x), nonEmpty, 'UniformOutput', false);
        [uniqStr, ~, sameidx] = unique(S);
        counts = histcounts(sameidx, 1:(numel(uniqStr)+1));
        dupGroups = find(counts > 1);
        % カメラで同一物体と識別されているクラスタをまとめる
        for i = dupGroups
            sameNonEmpty = (sameidx == i);            % 非空の中での位置
            sameCells = find(~isEmpty);               % 非空の元の行番号
            sameCells = sameCells(sameNonEmpty);      % 元の行インデックスに変換
            for j = 2 : size(sameCells,1)
                if any(min(ObjectData{sameCells(1), 1}(:,1)) <= ObjectData{sameCells(j), 1}(:,1)) && any(max(ObjectData{sameCells(1), 1}(:,1)) >= ObjectData{sameCells(j), 1}(:,1)) && ...
                   any(min(ObjectData{sameCells(1), 1}(:,2)) <= ObjectData{sameCells(j), 1}(:,2)) && any(max(ObjectData{sameCells(1), 1}(:,2)) >= ObjectData{sameCells(j), 1}(:,2))
                    ObjectData_fromLiDAR{sameCells(1), 1} = [ObjectData_fromLiDAR{sameCells(1), 1}; ObjectData_fromLiDAR{sameCells(j), 1}];
                    ObjectData_fromLiDAR{sameCells(j),1} = [];
                    ObjectData{sameCells(1), 1} = [ObjectData{sameCells(1), 1}; ObjectData{sameCells(j), 1}];
                    ObjectData{sameCells(j),1} = [];
                end
            end
        end
        ObjectData_fromLiDAR(cellfun(@isempty, ObjectData_fromLiDAR(:,1)),:) = [];
        ObjectData(cellfun(@isempty, ObjectData(:,1)),:) = [];
    
        Objectpt = cell2mat(ObjectData(:,1)); % ObjectDataに保存されている点群
    else
        Objectpt = [];
    end
    new_labels = zeros(size(Objectpt,1),1);
    labelsstart = 1;
    labelsend = 0;
    for i = 1 : size(ObjectData,1)
        labelsend = labelsend + size(ObjectData{i,1},1);
        new_labels(labelsstart:labelsend,1) = i * ones(size(ObjectData{i,1},1),1);
        labelsstart = labelsstart + size(ObjectData{i,1},1);
    end

end