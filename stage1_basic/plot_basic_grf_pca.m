function plot_basic_grf_pca(rows,a,output)
% The minimum-cost feasible case supplies both its RGB and reconstruction rate.
params={'vx0','thetaL','r0'};
labels={'$v_{x,0}\;[\mathrm{m/s}]$','$\theta_{\mathrm L}\;[\mathrm{deg}]$','$r_0\;[\mathrm{m}]$'};
short={'v_{x,0}','\theta_{\mathrm L}','r_0'};
units={'\mathrm{m/s}','\mathrm{deg}','\mathrm{m}'};
scale=[1 180/pi 1]; pairs=[1 2 3;1 3 2;2 3 1];
axesValues=cell(1,3);
for p=1:3, axesValues{p}=unique(rows.(params{p}))'; end
directions={'horizontal','vertical'}; notation={'x','z'};
for d=1:2
 name=directions{d}; m=a.(name);
 if ~any(m.valid), continue; end
 fprintf('  GRF PCA figures: %s\n',name);
 % Exactly three modal curves, scaled to one standard deviation.
 f=newFigure(1400,800); cleanup=onCleanup(@()close(f));
 ax=axes(f); plot(ax,a.phase,m.coefficients.*m.sigma,'LineWidth',2.5);
 formatWaveAxes(ax,sprintf('$\\Delta\\hat F_%s$',notation{d}));
 legend(ax,{'$\mathrm{PC1}$','$\mathrm{PC2}$','$\mathrm{PC3}$'},...
  'Interpreter','latex','FontSize',23,'Location','best');
 title(ax,sprintf('$%s:\\;\\mathrm{one\\ standard\\ deviation\\ modes}$',notation{d}),...
  'Interpreter','latex','FontSize',26);
 exportgraphics(f,fullfile(output,['pca_01_components_' name '.png']),'Resolution',220);
 clear cleanup

 f=newFigure(2400,750); cleanup=onCleanup(@()close(f));
 layout=tiledlayout(f,1,3,'TileSpacing','compact','Padding','compact');
 for j=1:3
  ax=nexttile(layout); delta=m.coefficients(:,j)*m.sigma(j);
  plot(ax,a.phase,m.mean,'k-','LineWidth',2.5); hold(ax,'on');
  plot(ax,a.phase,m.mean'+delta,'-','Color',[.85 .15 .15],'LineWidth',2.5);
  plot(ax,a.phase,m.mean'-delta,'--','Color',[.15 .3 .8],'LineWidth',2.5);
  formatWaveAxes(ax,sprintf('$\\hat F_%s$',notation{d}));
  title(ax,sprintf('$\\mathrm{PC%d}:\\;%.1f\\%%$',j,m.explained(j)),...
   'Interpreter','latex','FontSize',24);
 end
 legend(ax,{'$\mu$','$\mu+\sigma_j\phi_j$','$\mu-\sigma_j\phi_j$'},...
  'Interpreter','latex','FontSize',21,'Location','best');
 exportgraphics(f,fullfile(output,['pca_02_mean_plus_minus_' name '.png']),'Resolution',220);
 clear cleanup

 for kind=1:2
  f=newFigure(2400,750); cleanup=onCleanup(@()close(f));
  layout=tiledlayout(f,1,3,'TileSpacing','compact','Padding','compact');
  for p=1:3
   pair=pairs(p,:); selected=selectCases(rows,params,axesValues,pair,[]);
   ax=nexttile(layout);
   paint(ax,selected,m,kind,axesValues(pair(1:2)),scale(pair(1:2)),labels(pair(1:2)));
  end
  if kind==1
   titleText=sprintf('$%s:\\;\\mathrm{minimum\\ cost},\\quad R=PC1,\\ G=PC2,\\ B=PC3$',notation{d});
   filename=['pca_03_best_cost_rgb_' name '.png'];
  else
   titleText=sprintf('$%s:\\;\\mathrm{minimum\\ cost},\\quad\\mathrm{PC1\\!:\\!3\\ reconstruction}$',notation{d});
   filename=['pca_04_best_cost_cumulative_' name '.png'];
   cb=colorbar(ax); cb.Layout.Tile='east'; cb.TickLabelInterpreter='latex'; cb.FontSize=22;
   cb.Label.String='$\eta_{1:3}\;[\%]$'; cb.Label.Interpreter='latex'; cb.Label.FontSize=24;
  end
  sgtitle(layout,titleText,'Interpreter','latex','FontSize',26);
  exportgraphics(f,fullfile(output,filename),'Resolution',220);
  clear cleanup
 end

 for p=1:3
  pair=pairs(p,:); third=pair(3); fixed=axesValues{third};
  f=newFigure(max(2400,560*numel(fixed)+260),750); cleanup=onCleanup(@()close(f));
  layout=tiledlayout(f,1,numel(fixed),'TileSpacing','compact','Padding','compact');
  for j=1:numel(fixed)
   selected=selectCases(rows,params,axesValues,pair,fixed(j)); ax=nexttile(layout);
   paint(ax,selected,m,1,axesValues(pair(1:2)),scale(pair(1:2)),labels(pair(1:2)));
   title(ax,sprintf('$%s=%.4g\\;[%s]$',short{third},fixed(j)*scale(third),units{third}),...
    'Interpreter','latex','FontSize',23);
  end
  sgtitle(layout,sprintf('$%s:\\;\\mathrm{minimum\\ cost},\\quad R=PC1,\\ G=PC2,\\ B=PC3$',notation{d}),...
   'Interpreter','latex','FontSize',26);
  filename=sprintf('pca_05_slices_%s_%s_%s.png',params{pair(1)},params{pair(2)},name);
  exportgraphics(f,fullfile(output,filename),'Resolution',220);
  clear cleanup
 end
end
end

function selected=selectCases(rows,params,values,pair,fixed)
x=values{pair(1)}; y=values{pair(2)}; selected=zeros(numel(y),numel(x));
valid=rows.feasible==1 & isfinite(rows.cost);
if ~isempty(fixed), valid=valid & rows.(params{pair(3)})==fixed; end
for iy=1:numel(y)
 for ix=1:numel(x)
  candidates=find(valid & rows.(params{pair(1)})==x(ix) & rows.(params{pair(2)})==y(iy));
  if isempty(candidates), continue; end
  [~,order]=sortrows([rows.cost(candidates),rows.caseId(candidates)],[1 2]);
  selected(iy,ix)=candidates(order(1));
 end
end
end

function paint(ax,selected,m,kind,values,scale,labels)
x=values{1}*scale(1); y=values{2}*scale(2); present=selected>0;
if kind==1
 rgb=zeros([size(selected),3]);
 for j=1:3
  channel=zeros(size(selected)); channel(present)=m.rgb(selected(present),j);
  channel(present & ~isfinite(channel))=128/255; rgb(:,:,j)=channel;
 end
 image(ax,x,y,rgb);
else
 value=nan(size(selected)); value(present)=m.caseCumulative(selected(present),3);
 h=imagesc(ax,x,y,value); h.AlphaData=double(isfinite(value));
 colormap(ax,parula(256)); clim(ax,[0 100]);
 undefined=present & ~isfinite(value);
 if any(undefined,'all')
  hold(ax,'on');
  neutral=repmat(reshape([.5 .5 .5],1,1,3),[size(selected),1]);
  h=image(ax,x,y,neutral); h.AlphaData=double(undefined);
 end
end
ax.Color=[0 0 0]; ax.YDir='normal'; ax.XTick=x; ax.YTick=y;
if numel(x)>1, dx=(x(end)-x(1))/(numel(x)-1); else, dx=max(abs(x(1))*.1,1e-3); end
if numel(y)>1, dy=(y(end)-y(1))/(numel(y)-1); else, dy=max(abs(y(1))*.1,1e-3); end
xlim(ax,[x(1)-dx/2 x(end)+dx/2]); ylim(ax,[y(1)-dy/2 y(end)+dy/2]);
xlabel(ax,labels{1},'Interpreter','latex'); ylabel(ax,labels{2},'Interpreter','latex');
ax.FontSize=25; ax.LineWidth=1.5; ax.TickLabelInterpreter='latex'; ax.Box='on'; ax.Layer='top';
end

function f=newFigure(w,h)
f=figure('Visible','off','Color','white','Units','pixels','Position',[100 100 w h]);
end

function formatWaveAxes(ax,label)
xlabel(ax,'$s=t/T_{\mathrm{st}}$','Interpreter','latex');
ylabel(ax,label,'Interpreter','latex'); xlim(ax,[0 1]); grid(ax,'on');
ax.FontSize=25; ax.LineWidth=1.5; ax.TickLabelInterpreter='latex'; ax.Box='on';
end
