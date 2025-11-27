function [result] = gen_gridmap(pms)
    dx = (pms.estimate.xRoi(2) - pms.estimate.xRoi(1))/pms.gridNum(1);     % グリッド間隔　(マップ領域/グリッド数)
    dy = (pms.estimate.yRoi(2) - pms.estimate.yRoi(1))/pms.gridNum(2);
    xco = pms.observe.xRoi(1)-dx*(pms.gridNum(1)-1):dx:pms.observe.xRoi(2)+dx*(pms.gridNum(1)-1);   % グリッドマップのx座標
    yco = pms.observe.yRoi(1)-dy*(pms.gridNum(2)-1):dy:pms.observe.yRoi(2)+dy*(pms.gridNum(2)-1);   % グリッドマップのy座標
    [X,Y] = meshgrid(xco(2:end),yco(2:end));  % 2次元のグリッドマップの生成
    result.X = X';
    result.Y = Y';
    result.XColumn = reshape(result.X,[],1);    % ベクトル化
    result.YColumn = reshape(result.Y,[],1);
    result.U = zeros(size(result.X,1),size(result.X,2),2);
    result.V = zeros(size(result.X,1),size(result.X,2),2);
    result.Pu = zeros(2,2,size(result.X,1),size(result.X,2));
    result.Pv = zeros(2,2,size(result.X,1),size(result.X,2));
    % Get the map size
    result.Size = size(result.X);
end
