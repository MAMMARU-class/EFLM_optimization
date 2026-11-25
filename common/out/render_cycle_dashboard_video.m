function render_cycle_dashboard_video(result,folder)
% One gait cycle: forward motion above six time plots.
r=result.final; b=r.body; p=result.parameters; cfg=result.settings;
dest=fullfile(folder,'animation'); if ~isfolder(dest), mkdir(dest); end
fps=cfg.fps; seconds=2*max(4,b.T/cfg.playbackSpeed);
fig=figure('Visible','off','Color','white','Units','pixels','Position',[50 50 1600 850]);
figGuard=onCleanup(@()close(fig)); %#ok<NASGU>
layout=tiledlayout(fig,4,2,'TileSpacing','compact','Padding','compact');
motion=nexttile(layout,1,[1 2]); hold(motion,'on');
xx=[b.x;b.flightX]; zz=[b.z;b.flightZ];
motionXCenter=(min(xx)+max(xx))/2;
motionYLimits=[min([0;zz])-.05,max(zz)+.08];
ylim(motion,motionYLimits);
axis(motion,'equal');
% Keep a metric aspect ratio while centering the one-step travel in the
% available wide panel. Empty space on both sides is intentional.
drawnow;
panelPixels=getpixelposition(motion);
halfWidth=max((max(xx)-min(xx))/2+.04,...
 diff(motionYLimits)*panelPixels(3)/panelPixels(4)/2);
xlim(motion,motionXCenter+[-halfWidth halfWidth]);
axis(motion,'manual'); grid(motion,'on'); ylabel(motion,'z [m]');
plot(motion,xlim(motion),[0 0],'k-','LineWidth',1.5);
link=plot(motion,NaN,NaN,'k-','LineWidth',3);
spring=plot(motion,NaN,NaN,'b-','LineWidth',2.5);
joints=plot(motion,NaN,NaN,'ko','MarkerFaceColor','w','MarkerSize',6);
body=plot(motion,NaN,NaN,'bo','MarkerFaceColor','b','MarkerSize',8);
lineWidth=1.5; plotAxes=gobjects(6,1); cursors=gobjects(6,1);
flightT=b.Ts+b.flightTime;
series={ [b.t;b.Ts+b.flightTime(2:end)], [b.x;b.flightX(2:end)], [b.z;b.flightZ(2:end)];...
 [b.t;b.Ts+b.flightTime(2:end)], [b.Fx;zeros(numel(b.flightTime)-1,1)], [b.Fz;zeros(numel(b.flightTime)-1,1)];...
 b.t,rad2deg(r.angles(:,1)),rad2deg(r.angles(:,2));...
 b.t,r.tau(:,1),r.tau(:,2);...
 b.t,1000*r.s,[];...
 b.t,r.power(:,1),r.power(:,2)};
labels={'COM position [m]','GRF [N]','Joint angle [deg]','Torque [N m]','Spring [mm]','Power [W]'};
for j=1:6
 ax=nexttile(layout,j+2); plotAxes(j)=ax; hold(ax,'on');
 plot(ax,series{j,1},series{j,2},'LineWidth',lineWidth);
 if ~isempty(series{j,3}), plot(ax,series{j,1},series{j,3},'LineWidth',lineWidth); end
 xlim(ax,[0 b.T]); ylabel(ax,labels{j}); grid(ax,'on');
 yl=ylim(ax);
 patch(ax,[b.Ts b.T b.T b.Ts],[yl(1) yl(1) yl(2) yl(2)],...
  [.7 .7 .7],'FaceAlpha',.25,'EdgeColor','none','HandleVisibility','off');
 ylim(ax,yl);
 cursors(j)=plot(ax,[0 0],yl,'r-','LineWidth',3*lineWidth,'HandleVisibility','off');
 if j>=5, xlabel(ax,'time [s]'); end
end
vid=VideoWriter(fullfile(dest,'one_cycle_dashboard.mp4'),'MPEG-4'); vid.FrameRate=fps;
open(vid); videoGuard=onCleanup(@()close(vid)); %#ok<NASGU>
for t=linspace(0,b.T,max(2,ceil(seconds*fps)))
 if t<=b.Ts
  x=interp1(b.t,b.x,t); z=interp1(b.t,b.z,t);
  angles=interp1(b.t,r.angles(:,1:2),t); s=interp1(b.t,r.s,t);
  [~,~,local]=eflm_geometry(angles(1),angles(2),s,p);
  pts=squeeze(local)'+[x z];
  set(link,'XData',[pts([1 3 4],1);NaN;pts([5 6],1)],...
   'YData',[pts([1 3 4],2);NaN;pts([5 6],2)],'Visible','on');
  set(joints,'XData',pts(:,1),'YData',pts(:,2),'Visible','on');
  set(spring,'XData',pts([2 5],1),'YData',pts([2 5],2),'Visible','on');
 else
  f=t-b.Ts; x=b.x(end)+b.vx(end)*f;
  z=b.z(end)+b.vz(end)*f-p.g*f*f/2;
  set([link,spring,joints],'Visible','off');
 end
 set(body,'XData',x,'YData',z);
 for j=1:6, set(cursors(j),'XData',[t t]); end
 title(motion,sprintf('One gait cycle | t = %.3f / %.3f s',t,b.T));
 drawnow; writeVideo(vid,getframe(fig));
end
end
