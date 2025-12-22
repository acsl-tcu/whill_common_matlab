function [new_labels, point_cloud] = clustring_PCA(ptCloud,pms,Z_Comp)
%clustring_PCA この関数の概要をここに記述
%   詳細説明をここに記述
    
    % ダウンサンプリング
    % if not(isempty(ptCloud))
    %     ptCloud = pcdownsample(nonGPClouds,'gridAverage',pms.gridStep);
    % end
    point_cloud = ptCloud.Location;

    ptCloud_resolution = [ptCloud.Location(:,1),ptCloud.Location(:,2),ptCloud.Location(:,3)/Z_Comp]; % Z軸圧縮

    % -----クラスタリング(少数クラスタの除去)---------------------------
    [labels,numClusters] = ...
        pcsegdist(pointCloud(ptCloud_resolution),pms.minDistance,'NumClusterPoints',pms.numClusterPoints);

    % -----オブジェクトを識別(歩行者点群抽出)---------------------------
    new_labels = double(labels);
    count_cluster = 0;
    for j = 1:numClusters   % ラベルごとに識別
        pcCluster = ptCloud.Location(labels==j,:);
        [~,~,latent,~,explained,~] = pca(pcCluster);
        Contri_rate = explained/100;
        if any(Contri_rate< 0.007 | latent>0.5) % 主成分分析の寄与率と固有値の閾値で判別
            new_labels(labels == j) = 0;
        else
            count_cluster = count_cluster + 1;
            new_labels(labels == j) = count_cluster;
        end
    end
end