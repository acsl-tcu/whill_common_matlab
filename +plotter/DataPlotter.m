function DataPlotter(DATAPath,cfg)
%% Plot configuration
    close all;
    import plotter.*
    warning('off','all');
    set(0,'defaultAxesFontSize',17);
    set(0,'defaultTextFontSize',25);
    set(0,'defaultLineLineWidth',2);
    set(0,'defaultLineMarkerSize',10);
    set(0,'defaultLineMarkerFaceColor',[1 1 1]);
    set(0,'defaultFigureColor',[1 1 1]);
    set(groot,'defaultAxesTickLabelInterpreter','latex');
    set(groot,'defaulttextinterpreter','latex');
    set(groot,'defaultLegendInterpreter','latex');
    
    mode = cfg.modeNumber;
    
    if nargin == 2
        DATAdir = DATAPath;
    else
        clear variables; close all; clc;
        tmp     = matlab.desktop.editor.getActive;
        DATAdir = cd(fileparts(tmp.Filename));
    end
    
    Outputdir = strcat(DATAdir,'/result');
    mkdir(Outputdir,'jpeg');
    mkdir(Outputdir,'eps');
    mkdir(Outputdir,'fig');
    mkdir(Outputdir,'video');

    
    if cfg.isParallel
        keepAllSequences = 0; % 無効な時系列の保持
        plot3session_preprocesser(DATAdir,keepAllSequences) 
        load(strcat(DATAdir,filesep,"userLocal.mat"),"userLocal")
        T = seconds((userLocal.tNode - userLocal.tNode(1)));
    else
        plot_preprocesser(DATAdir,mode) 
        load(strcat(DATAdir,filesep,"userLocal.mat"),"userLocal")
        T = seconds((userLocal.Time - userLocal.Time(1)));
    end
    
    starttime = min(T);
    endtime   = max(T);
    Count     = 1;

    
    
    %% 計算時間
    [~, ax, lgd] = setFigure('Computation time',Count);
    switch mode
        case {0,1,2,3}
            CT(1) = 0;
            for i = 2:numel(T)
                CT(i) = T(i)-T(i-1); % Computation time
            end
            p = plot(T, CT);
            hline = refline([0 mean(CT)]);
            ax.XLim    = [starttime, endtime-starttime];
    end
    hline.Color = 'r';
    arrayfun(@(x) set(x,'LineWidth', 1.75), p);
    arrayfun(@(x) set(x,'LineStyle', '-'),  p);
    ax.XLabel.String = 'Time(s)';
    ax.YLabel.String = 'Computation Time(s)';
    lgd.String      = {'Computation ','Average'};
    Count = saveFigure('computation_Time',Count, Outputdir);
    
end
%% FUNCTION
function [fig, ax, lgd] = setFigure(Name, Count)
	figure(Count)
    fig = gcf;
    fig.Name = Name;
    fig.Color= [1., 1., 1.];
    ax  = gca;
    ax.FontSize = 16;
    ax.Box      = 'on';
    ax.XGrid    = 'on';
    ax.YGrid    = 'on';
    ax.NextPlot = 'add';
    ax.XLabel.Interpreter  = 'Latex';
    ax.YLabel.Interpreter  = 'Latex';
    lgd = legend;
    lgd.FontSize    = 13;
    lgd.Location    = 'northwest';
    lgd.Interpreter = 'Latex';
end
function ret = saveFigure(Name, Count, Outputdir)
	hold off;
	saveas(gcf,strcat(Outputdir,strcat('/eps/',Name)),'epsc');
    saveas(gcf,strcat(Outputdir,strcat('/jpeg/',Name)),'jpeg');
	savefig(gcf,strcat(Outputdir,strcat('/fig/',Name)));
	ret = Count + 1;
end