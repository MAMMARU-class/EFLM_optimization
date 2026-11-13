function paint_basic_adaptive_map(ax,layers,params,baseAxes,pair,metric,extremum,fixed,showThird,scale,symbols,limits)
% Phase1 cells cover the full original range. Later phases refine in place.
width=zeros(1,3);
for p=1:3
 values=baseAxes{p};
 if numel(values)>1
  width(p)=median(diff(values));
 else
  width(p)=max(abs(values)*0.1,1e-3);
 end
end
[xy,values]=bestCells(layers(1).rows,params,pair,metric,extremum,fixed,width,1,showThird,scale);
base=nan(numel(baseAxes{pair(2)}),numel(baseAxes{pair(1)}));
for i=1:size(xy,1)
 [~,ix]=min(abs(baseAxes{pair(1)}-xy(i,1)));
 [~,iy]=min(abs(baseAxes{pair(2)}-xy(i,2)));
 base(iy,ix)=values(i);
end
x=baseAxes{pair(1)}*scale(pair(1)); y=baseAxes{pair(2)}*scale(pair(2));
basic_grid_image(ax,x,y,base,double(isfinite(base)));
ax.Color=[0 0 0]; ax.YDir='normal'; colormap(ax,parula(256)); clim(ax,limits);
hold(ax,'on');
for layer=2:numel(layers)
 [xy,values]=bestCells(layers(layer).rows,params,pair,metric,extremum,fixed,...
  width,layers(layer).division,showThird,scale);
 if isempty(xy), continue; end
 half=width(pair(1:2)).*scale(pair(1:2))/(2*layers(layer).division);
 centers=xy.*scale(pair(1:2));
 good=isfinite(values);
 drawCells(ax,centers(good,:),half,values(good),false);
 drawCells(ax,centers(~good,:),half,zeros(sum(~good),1),true);
end
hold(ax,'off');
ax.XTick=x; ax.YTick=y;
xlabel(ax,symbols{pair(1)},'Interpreter','latex');
ylabel(ax,symbols{pair(2)},'Interpreter','latex');
ax.FontSize=25; ax.LineWidth=1.5; ax.TickLabelInterpreter='latex';
ax.Box='on'; ax.Layer='top';
end

function [xy,values]=bestCells(rows,params,pair,metric,extremum,fixed,width,division,showThird,scale)
if ~isempty(fixed)
 third=rows.(params{pair(3)});
 if division==1
  keep=abs(third-fixed)<1e-10;
 else
  % Refined children belong to their nearest original third-parameter slice.
  keep=abs(third-fixed)<=width(pair(3))/2+1e-10;
 end
 rows=rows(keep,:);
end
if isempty(rows), xy=zeros(0,2); values=zeros(0,1); return; end
points=[rows.(params{pair(1)}),rows.(params{pair(2)})];
[xy,~,groups]=unique(points,'rows');
values=nan(size(xy,1),1);
data=rows.(metric);
valid=rows.feasible==1 & isfinite(data);
third=rows.(params{pair(3)});
for i=1:size(xy,1)
 candidates=find(groups==i & valid);
 if isempty(candidates), continue; end
 if strcmp(extremum,'max')
  [~,best]=max(data(candidates));
 else
  [~,best]=min(data(candidates));
 end
 index=candidates(best);
 if showThird, values(i)=third(index)*scale(pair(3));
 else, values(i)=data(index); end
end
end

function drawCells(ax,centers,half,values,black)
if isempty(centers), return; end
x=centers(:,1)'; y=centers(:,2)';
xx=[x-half(1);x+half(1);x+half(1);x-half(1)];
yy=[y-half(2);y-half(2);y+half(2);y+half(2)];
if black
 patch(ax,xx,yy,'k','EdgeColor','none');
else
 patch(ax,xx,yy,zeros(size(xx)),repmat(reshape(values,1,[]),4,1),...
  'FaceColor','flat','EdgeColor','none');
end
end
