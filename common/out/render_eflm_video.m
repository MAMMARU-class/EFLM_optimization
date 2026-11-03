function render_eflm_video(result,folder)
% Saved-result-only rendering. Old EFLM colors; no leg is shown in flight.
r=result.final; p=result.parameters; cfg=result.settings; b=r.body;
dest=fullfile(folder,'animation'); if ~isfolder(dest), mkdir(dest); end
vid=VideoWriter(fullfile(dest,'motion.mp4'),'MPEG-4'); vid.FrameRate=cfg.fps;
open(vid); cv=onCleanup(@()close(vid));
fig=figure('Visible','off','Position',[100 100 960 640]); cf=onCleanup(@()close(fig));
minimumSeconds=4;
if isfield(cfg,'minimumVideoSeconds'), minimumSeconds=cfg.minimumVideoSeconds; end
duration=max(minimumSeconds,cfg.cycles*b.T/cfg.playbackSpeed);
playbackSpeed=cfg.cycles*b.T/duration;
renderInfo=struct('direction','XDir reverse: positive simulation x displayed left',...
 'flight','body only; leg hidden','rigidLinkColor','black','springColor','blue',...
 'playbackSpeed',playbackSpeed,'fps',cfg.fps,'createdAt',char(datetime('now')));
save(fullfile(dest,'render_settings.mat'),'renderInfo');
ax=axes(fig); hold(ax,'on');
xlimits=[b.x(1)-.18,b.x(1)+cfg.cycles*b.distance+.18];
plot(ax,xlimits,[0 0],'k-');
link=plot(ax,NaN,NaN,'k-','LineWidth',3.9);
spring=plot(ax,NaN,NaN,'b-','LineWidth',3.2);
active=plot(ax,NaN,NaN,'ro','MarkerFaceColor','r','MarkerEdgeColor','k','MarkerSize',9);
passive=plot(ax,NaN,NaN,'ko','MarkerFaceColor','w','MarkerSize',8);
body=plot(ax,NaN,NaN,'bo','MarkerFaceColor','b','MarkerSize',10);
axis(ax,'equal'); xlim(ax,xlimits); ylim(ax,[-.05,.48]); grid(ax,'on');
set(ax,'XDir','reverse'); xlabel(ax,'x [m] (forward displayed left)'); ylabel(ax,'z [m]');
isDiagnostic=isfield(result,'status') && contains(result.status,'diagnostic');
flightPreviewSaved=false;
for tt=linspace(0,cfg.cycles*b.T,max(2,ceil(duration*cfg.fps)))
 cycle=min(cfg.cycles-1,floor(tt/b.T)); t=tt-cycle*b.T;
 if t<=b.Ts
  x=interp1(b.t,b.x,t); z=interp1(b.t,b.z,t);
  angles=interp1(b.t,r.angles(:,1:2),t); s=interp1(b.t,r.s,t);
  model='elastic_approx'; if isfield(cfg,'model'), model=cfg.model; end
  label=['Stance: optimized ' model];
  if isDiagnostic, label='Stance: diagnostic trajectory (not optimized)'; end
  if ~isDiagnostic && ~result.valid, label='Stance: infeasible optimization candidate'; end
 else
  x=b.x(end)+b.vx(end)*(t-b.Ts); z=b.z(end)+b.vz(end)*(t-b.Ts)-p.g*(t-b.Ts)^2/2;
  label='Flight: ballistic body (leg hidden)';
 end
 x=x+cycle*b.distance;
 if t<=b.Ts
  [~,~,pts]=eflm_geometry(angles(1),angles(2),s,p);
  pts=squeeze(pts)'; pts=pts+[x z];
  set(link,'XData',[pts([1 3 4],1);NaN;pts([5 6],1)],...
   'YData',[pts([1 3 4],2);NaN;pts([5 6],2)],'Visible','on');
  set(active,'XData',pts([1 3],1),'YData',pts([1 3],2),'Visible','on');
  set(passive,'XData',pts([2 4 5],1),'YData',pts([2 4 5],2),'Visible','on');
  delta=pts(5,:)-pts(2,:); lengthSpring=norm(delta);
  normal=[-delta(2),delta(1)]/lengthSpring;
  u=linspace(.1,.9,13); offset=(-1).^(0:12)*min(.012,max(.004,.06*lengthSpring));
  offset([1,end])=0;
  coil=pts(2,:)+[0,u,1]'*delta+[0,offset,0]'*normal;
  set(spring,'XData',coil(:,1),'YData',coil(:,2),'Visible','on');
 else
  set([link,spring,active,passive],'Visible','off');
 end
 set(body,'XData',x,'YData',z);
 title(ax,sprintf('%s | t = %.5f s | playback %.4fx',label,tt,playbackSpeed));
 drawnow; writeVideo(vid,getframe(fig));
 if tt==0, exportgraphics(ax,fullfile(dest,'stance_preview.png')); end
 if t>b.Ts && ~flightPreviewSaved
  exportgraphics(ax,fullfile(dest,'flight_preview.png')); flightPreviewSaved=true;
 end
end
end
