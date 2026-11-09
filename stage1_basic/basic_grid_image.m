function h=basic_grid_image(ax,x,y,data,alpha)
% Flat cells at actual coordinates; unlike imagesc, supports irregular grids.
xe=edges(x); ye=edges(y); [xx,yy]=meshgrid(xe,ye);
data=data([1:end end],[1:end end],:);
if nargin<5, alpha=ones(numel(y),numel(x)); end
alpha=alpha([1:end end],[1:end end]);
h=surface(ax,xx,yy,zeros(size(xx)),data,'FaceColor','flat','EdgeColor','none',...
 'CDataMapping','scaled','FaceAlpha','flat','AlphaData',alpha,'AlphaDataMapping','none');
view(ax,2); xlim(ax,xe([1 end])); ylim(ax,ye([1 end]));
end
function e=edges(v)
v=v(:)';
if numel(v)==1
 d=max(abs(v)*.05,1e-3); e=v+[-d d];
else
 e=[v(1)-(v(2)-v(1))/2,(v(1:end-1)+v(2:end))/2,v(end)+(v(end)-v(end-1))/2];
end
end
