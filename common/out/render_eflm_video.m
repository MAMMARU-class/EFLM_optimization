function render_eflm_video(result,folder)
% Saved-result-only rendering. Flight joint motion is visualization, not dynamics.
r=result.final; p=result.parameters; cfg=result.settings; b=r.body;
dest=fullfile(folder,'animation'); if ~isfolder(dest), mkdir(dest); end
vid=VideoWriter(fullfile(dest,'motion.mp4'),'MPEG-4'); vid.FrameRate=cfg.fps;
open(vid); cv=onCleanup(@()close(vid));
fig=figure('Visible','off','Position',[100 100 960 640]); cf=onCleanup(@()close(fig));
for tt=linspace(0,cfg.cycles*b.T,max(2,ceil(cfg.cycles*b.T*cfg.fps/cfg.playbackSpeed)))
 cycle=min(cfg.cycles-1,floor(tt/b.T)); t=tt-cycle*b.T;
 if t<=b.Ts
  x=interp1(b.t,b.x,t); z=interp1(b.t,b.z,t);
  angles=interp1(b.t,r.angles(:,1:2),t); s=interp1(b.t,r.s,t);
  label='Stance: optimized approximate model';
 else
  u=(t-b.Ts)/b.Tf; blend=u*u*(3-2*u);
  x=b.x(end)+b.vx(end)*(t-b.Ts); z=b.z(end)+b.vz(end)*(t-b.Ts)-p.g*(t-b.Ts)^2/2;
  angles=(1-blend)*r.angles(end,1:2)+blend*r.angles(1,1:2); s=0;
  label='Flight: ballistic body; illustrative joint interpolation';
 end
 x=x+cycle*b.distance;
 [~,~,pts]=eflm_geometry(angles(1),angles(2),s,p);
 pts=squeeze(pts)'; pts=pts+[x z];
 clf(fig); hold on;
 plot([x-.5 x+.5],[0 0],'k-');
 for edge=[1 3;3 4;4 6;4 5;5 2;2 3]'
  plot(pts(edge,1),pts(edge,2),'o-','LineWidth',3);
 end
 plot(x,z,'ko','MarkerFaceColor','k','MarkerSize',12);
 axis equal; xlim([x-.4 x+.4]); ylim([-.06 .48]); grid on;
 title(sprintf('%s | t = %.3f s | playback %.2fx',label,tt,cfg.playbackSpeed)); xlabel('x [m]'); ylabel('z [m]');
 drawnow; writeVideo(vid,getframe(fig));
end
end
