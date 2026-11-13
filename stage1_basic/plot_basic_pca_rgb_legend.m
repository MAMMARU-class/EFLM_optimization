function plot_basic_pca_rgb_legend(output)
% Shared RGB cube for signed PC scores scaled independently to [-1,1].
f=figure('Visible','off','Color','white','Units','pixels',...
 'Position',[100 100 1350 1050]);
cleanup=onCleanup(@()close(f)); %#ok<NASGU>
ax=axes(f,'Position',[.13 .18 .74 .72]); hold(ax,'on');
[u,v]=meshgrid(linspace(-1,1,129));
for side=[-1 1]
 for channel=1:3
  xyz={u,u,u}; xyz{channel}=side*ones(size(u));
  other=setdiff(1:3,channel);
  xyz{other(1)}=u; xyz{other(2)}=v;
  rgb=cat(3,round(255*(1+xyz{1})/2)/255,...
   round(255*(1+xyz{2})/2)/255,...
   round(255*(1+xyz{3})/2)/255);
  surface(ax,xyz{1},xyz{2},xyz{3},rgb,...
   'FaceColor','texturemap','EdgeColor','none');
 end
end
axis(ax,'equal'); axis(ax,[-1 1 -1 1 -1 1]);
view(ax,35,25); grid(ax,'on'); box(ax,'on');
ax.XTick=[-1 0 1]; ax.YTick=[-1 0 1]; ax.ZTick=[-1 0 1];
ax.FontSize=25; ax.LineWidth=1.5; ax.TickLabelInterpreter='latex';
xlabel(ax,'$u_1=s_1/M_1\quad(R=PC1)$','Interpreter','latex','FontSize',27);
ylabel(ax,'$u_2=s_2/M_2\quad(G=PC2)$','Interpreter','latex','FontSize',27);
zlabel(ax,'$u_3=s_3/M_3\quad(B=PC3)$','Interpreter','latex','FontSize',27);
title(ax,'$\mathrm{Signed\ PCA\ score\ RGB\ legend}$',...
 'Interpreter','latex','FontSize',30);
annotation(f,'textbox',[.09 .025 .82 .075],'EdgeColor','none',...
 'HorizontalAlignment','center','Interpreter','latex','FontSize',23,...
 'String','$M_j=\max|s_j|,\qquad RGB_j=\mathrm{round}\left[255(1+u_j)/2\right]$');
exportgraphics(f,fullfile(output,'pca_00_rgb_legend.png'),'Resolution',220);
end
