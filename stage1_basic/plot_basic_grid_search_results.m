function [outputFolder,sourceFolder] = plot_basic_grid_search_results(sourceFolder,outputFolder)
% Plot the complete set of basic-grid maps; infeasible cells are black.
% sourceFolder is the completed run; outputFolder is its grid_search_result.
root=fileparts(fileparts(mfilename('fullpath')));
if nargin<1 || isempty(sourceFolder)
 sourceFolder=latestCompletedBasic(fullfile(root,'_logs','stage1_basic'));
end
summaryFile=fullfile(sourceFolder,'summary.csv');
if ~isfile(summaryFile), error('EFLM:MissingSummary','Missing %s',summaryFile); end
rows=readtable(summaryFile);
[layers,baseRows]=basic_map_layers(sourceFolder,rows);
required={'vx0','thetaL','r0','feasible','cost','speed','CoT'};
if ~all(ismember(required,rows.Properties.VariableNames))
 error('EFLM:SummaryColumns','Missing required columns in %s',summaryFile);
end
params={'vx0','thetaL','r0'};
symbols={'$v_{x,0}\;[\mathrm{m/s}]$','$\theta_{\mathrm L}\;[\mathrm{deg}]$','$r_0\;[\mathrm{m}]$'};
short={'v_{x,0}','\theta_{\mathrm L}','r_0'};
units={'\mathrm{m/s}','\mathrm{deg}','\mathrm{m}'};
scale=[1 180/pi 1];
pairs=[1 2 3; 1 3 2; 2 3 1]; % x parameter, y parameter, fixed/selected parameter
metrics={'cost','speed','CoT'};
metricNames={'J_{\mathrm{cost}}','\bar v_x\;[\mathrm{m/s}]','\mathrm{CoT}'};
criterion={'minimum','maximum','minimum'};
extremum={'min','max','min'};
axisValues=cell(1,3);
for p=1:3, axisValues{p}=unique(baseRows.(params{p}))'; end
if any(cellfun(@isempty,axisValues)), error('EFLM:EmptyGrid','No basic grid rows found.'); end
if nargin<2 || isempty(outputFolder)
 stamp=char(datetime('now','TimeZone','Asia/Tokyo','Format','yyyyMMdd_HHmmss_SSS'));
 outputFolder=fullfile(root,'_logs','test',stamp);
end
if ~isfolder(outputFolder), mkdir(outputFolder); end

for m=1:3
 allRows=vertcat(layers.rows); metricData=allRows.(metrics{m});
 valueLimits=limitsFor(metricData(allRows.feasible==1 & isfinite(metricData)));
 % Three pairwise projections; select the independently best third parameter at each cell.
 f=newFigure(2400,750); cleanup=onCleanup(@()close(f));
 layout=tiledlayout(f,1,3,'TileSpacing','compact','Padding','compact');
 for p=1:3
  pair=pairs(p,:);
  ax=nexttile(layout);
  paint_basic_adaptive_map(ax,layers,params,axisValues,pair,metrics{m},extremum{m},[],false,...
   scale,symbols,valueLimits);
 end
 sgtitle(layout,sprintf('$\\mathrm{%s}\\; %s$',criterion{m},metricNames{m}),...
  'Interpreter','latex','FontSize',28);
 addColorbar(ax,['$' metricNames{m} '$']);
 exportgraphics(f,fullfile(outputFolder,sprintf('01_extrema_%s.png',metrics{m})),'Resolution',220);
 clear cleanup

 % Third parameter that attained each metric-specific optimum.
 f=newFigure(2400,750); cleanup=onCleanup(@()close(f));
 layout=tiledlayout(f,1,3,'TileSpacing','compact','Padding','compact');
 for p=1:3
  pair=pairs(p,:); third=pair(3);
  ax=nexttile(layout);
  paint_basic_adaptive_map(ax,layers,params,axisValues,pair,metrics{m},extremum{m},[],true,...
   scale,symbols,limitsFor(axisValues{third}*scale(third)));
  cb=colorbar(ax); cb.TickLabelInterpreter='latex'; cb.FontSize=22;
  cb.Label.String=symbols{third}; cb.Label.Interpreter='latex'; cb.Label.FontSize=24;
 end
 sgtitle(layout,sprintf('$\\mathrm{%s}\\; %s:\\;\\mathrm{selected\\ third\\ parameter}$',...
  criterion{m},metricNames{m}),'Interpreter','latex','FontSize',28);
 exportgraphics(f,fullfile(outputFolder,sprintf('02_selected_third_%s.png',metrics{m})),'Resolution',220);
 clear cleanup

 % Five (or the configured number of) fixed-third slices, side by side.
 for p=1:3
  pair=pairs(p,:); third=pair(3); fixed=axisValues{third};
  for page=1:ceil(numel(fixed)/7)
  subset=(page-1)*7+1:min(page*7,numel(fixed));
  f=newFigure(max(2400,560*numel(subset)+260),750); cleanup=onCleanup(@()close(f));
  layout=tiledlayout(f,1,numel(subset),'TileSpacing','compact','Padding','compact');
  for s=subset
   ax=nexttile(layout);
   paint_basic_adaptive_map(ax,layers,params,axisValues,pair,metrics{m},extremum{m},fixed(s),false,...
    scale,symbols,valueLimits);
   title(ax,sprintf('$%s=%.4g\\;[%s]$',short{third},fixed(s)*scale(third),units{third}),...
    'Interpreter','latex','FontSize',23);
  end
  sgtitle(layout,sprintf('$\\mathrm{%s}\\; %s$',criterion{m},metricNames{m}),...
   'Interpreter','latex','FontSize',28);
  addColorbar(ax,['$' metricNames{m} '$']);
  filename=sprintf('03_slices_%s_%s_%s.png',params{pair(1)},params{pair(2)},metrics{m});
  if numel(fixed)>7, filename=strrep(filename,'.png',sprintf('_page%02d.png',page)); end
  exportgraphics(f,fullfile(outputFolder,filename),'Resolution',220);
  clear cleanup
  end
 end
end
fprintf('Parameter maps saved to %s (source: %s)\n',outputFolder,sourceFolder);
end

function folder=latestCompletedBasic(logRoot)
entries=dir(logRoot); entries=entries([entries.isdir] & ~ismember({entries.name},{'.','..'}));
[~,order]=sort(string({entries.name}),'descend'); entries=entries(order);
for i=1:numel(entries)
 candidate=fullfile(entries(i).folder,entries(i).name);
 report=fullfile(candidate,'report.txt');
 if isfile(report) && isfile(fullfile(candidate,'summary.csv')) && ...
    contains(fileread(report),'Status: completed')
  folder=candidate; return;
 end
end
error('EFLM:NoBasicRun','No completed stage1_basic run found under %s',logRoot);
end

function f=newFigure(width,height)
f=figure('Visible','off','Color','white','Units','pixels',...
 'Position',[100 100 width height]);
end

function addColorbar(ax,label)
cb=colorbar(ax); cb.Layout.Tile='east'; cb.TickLabelInterpreter='latex'; cb.FontSize=22;
cb.Label.String=label; cb.Label.Interpreter='latex'; cb.Label.FontSize=24;
end

function limits=limitsFor(values)
values=values(isfinite(values));
if isempty(values), limits=[0 1]; return; end
limits=[min(values) max(values)];
if limits(1)==limits(2)
 padding=max(abs(limits(1))*.05,1e-6);
 limits=limits+[-padding padding];
end
end
