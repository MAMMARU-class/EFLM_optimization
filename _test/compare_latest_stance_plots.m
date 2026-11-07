function [outputFolder, sourceFolders] = compare_latest_stance_plots()
% Recreate stance plots for the latest completed approx, exact, and EFLM runs.
% Historical-log utility: reads saved results only, not the deleted models.
% Every corresponding panel uses the same y limits across the three models.

root = fileparts(fileparts(mfilename('fullpath')));
spec = struct( ...
    'model', {'elastic_approx','elastic_exact','EFLM'}, ...
    'file',  {'approx.png','exact.png','EFLM.png'});

runData = cell(1,numel(spec));
sourceFolders = cell(1,numel(spec));
for i = 1:numel(spec)
    sourceFolders{i} = latestCompletedRun(fullfile(root,'_logs',spec(i).model));
    loaded = load(fullfile(sourceFolders{i},'result.mat'),'result');
    if ~isfield(loaded.result,'final')
        error('EFLM:MissingFinalResult','No final result in %s',sourceFolders{i});
    end
    runData{i} = loaded.result.final;
end

% Panel order matches common/out/save_run_outputs.m.
values = cell(numel(spec),6);
for i = 1:numel(spec)
    r = runData{i}; b = r.body;
    values{i,1} = [b.x,b.z];
    values{i,2} = [b.Fx,b.Fz];
    values{i,3} = rad2deg(r.angles);
    values{i,4} = r.tau;
    values{i,5} = r.s*1000;
    values{i,6} = r.power;
end

sharedLimits = zeros(6,2);
for panel = 1:6
    allValues = [];
    for model = 1:numel(spec)
        allValues = [allValues; values{model,panel}(:)]; %#ok<AGROW>
    end
    sharedLimits(panel,:) = paddedLimits(allValues);
end

stamp = char(datetime('now','TimeZone','Asia/Tokyo','Format','yyyyMMdd_HHmmss_SSS'));
outputFolder = fullfile(root,'_logs','test',stamp);
mkdir(outputFolder);

fontSize = 25; % MATLAB's usual 10 pt default multiplied by 2.5.
for i = 1:numel(spec)
    r = runData{i}; b = r.body;
    f = figure('Visible','off','Color','white','Units','pixels',...
        'Position',[100 100 1500 1125]);
    cleanup = onCleanup(@()close(f));
    layout = tiledlayout(f,3,2,'TileSpacing','compact','Padding','compact');

    ax = nexttile(layout); plot(ax,b.t,values{i,1},'LineWidth',2.0);
    ylabel(ax,'position [m]'); legend(ax,{'x','z'},'Location','best');
    ylim(ax,sharedLimits(1,:));

    ax = nexttile(layout); plot(ax,b.t,values{i,2},'LineWidth',2.0);
    ylabel(ax,'GRF [N]'); legend(ax,{'Fx','Fz'},'Location','best');
    ylim(ax,sharedLimits(2,:));

    ax = nexttile(layout); plot(ax,b.t,values{i,3},'LineWidth',2.0);
    ylabel(ax,'angle [deg]'); legend(ax,{'hip','knee','ankle'},'Location','best');
    ylim(ax,sharedLimits(3,:));

    ax = nexttile(layout); plot(ax,b.t,values{i,4},'LineWidth',2.0);
    ylabel(ax,'torque [Nm]'); legend(ax,{'hip','knee'},'Location','best');
    ylim(ax,sharedLimits(4,:));

    ax = nexttile(layout); plot(ax,b.t,values{i,5},'LineWidth',2.0);
    ylabel(ax,'spring extension [mm]'); xlabel(ax,'time [s]');
    ylim(ax,sharedLimits(5,:));

    ax = nexttile(layout); plot(ax,b.t,values{i,6},'LineWidth',2.0);
    ylabel(ax,'power [W]'); xlabel(ax,'time [s]');
    ylim(ax,sharedLimits(6,:));

    set(findall(f,'Type','axes'),'FontSize',fontSize,'LineWidth',1.5);
    set(findall(f,'Type','legend'),'FontSize',fontSize);
    exportgraphics(f,fullfile(outputFolder,spec(i).file),'Resolution',220);
    clear cleanup
end

fprintf('Comparison plots saved to: %s\n',outputFolder);
for i = 1:numel(spec)
    fprintf('  %-14s <- %s\n',spec(i).file,sourceFolders{i});
end
end

function folder = latestCompletedRun(modelRoot)
entries = dir(modelRoot);
entries = entries([entries.isdir] & ~ismember({entries.name},{'.','..'}));
[~,order] = sort(string({entries.name}),'descend');
entries = entries(order);
for i = 1:numel(entries)
    candidate = fullfile(entries(i).folder,entries(i).name);
    if isfile(fullfile(candidate,'result.mat'))
        folder = candidate;
        return
    end
end
error('EFLM:NoCompletedRun','No result.mat found under %s',modelRoot);
end

function limits = paddedLimits(values)
values = values(isfinite(values));
if isempty(values)
    error('EFLM:PlotData','No finite values available for a plot panel.');
end
lo = min(values); hi = max(values); span = hi-lo;
if span == 0
    span = max(1,abs(lo));
end
padding = 0.05*span;
limits = [lo-padding,hi+padding];
end
