function plot_basic_result(result,filename)
% One PNG, with the established Stage1 plot styling.
r=result.final; b=r.body;
f=figure('Visible','off','Color','white','Units','pixels',...
 'Position',[100 100 1500 1125]); clean=onCleanup(@()close(f));
layout=tiledlayout(f,3,2,'TileSpacing','compact','Padding','compact');

ax=nexttile(layout); plot(ax,b.t,[b.x b.z],'LineWidth',2);
ylabel(ax,'$\mathrm{position}\;[\mathrm{m}]$','Interpreter','latex');
legend(ax,{'$x$','$z$'},'Location','best','Interpreter','latex');

ax=nexttile(layout); plot(ax,b.t,[b.Fx b.Fz],'LineWidth',2);
ylabel(ax,'$\mathrm{GRF}\;[\mathrm{N}]$','Interpreter','latex');
legend(ax,{'$F_x$','$F_z$'},'Location','best','Interpreter','latex');

ax=nexttile(layout); plot(ax,b.t,rad2deg(r.angles),'LineWidth',2);
ylabel(ax,'$\mathrm{angle}\;[\mathrm{deg}]$','Interpreter','latex');
legend(ax,{'$\mathrm{hip}$','$\mathrm{knee}$','$\mathrm{ankle}$'},...
 'Location','best','Interpreter','latex');

ax=nexttile(layout); plot(ax,b.t,r.tau,'LineWidth',2);
ylabel(ax,'$\mathrm{torque}\;[\mathrm{N\,m}]$','Interpreter','latex');
legend(ax,{'$\mathrm{hip}$','$\mathrm{knee}$'},'Location','best','Interpreter','latex');

ax=nexttile(layout); plot(ax,b.t,r.s*1000,'LineWidth',2);
ylabel(ax,'$d_s\;[\mathrm{mm}]$','Interpreter','latex');
xlabel(ax,'$\mathrm{time}\;[\mathrm{s}]$','Interpreter','latex');

ax=nexttile(layout); plot(ax,b.t,r.power,'LineWidth',2);
ylabel(ax,'$\mathrm{power}\;[\mathrm{W}]$','Interpreter','latex');
xlabel(ax,'$\mathrm{time}\;[\mathrm{s}]$','Interpreter','latex');

set(findall(f,'Type','axes'),'FontSize',25,'LineWidth',1.5,...
 'TickLabelInterpreter','latex');
set(findall(f,'Type','legend'),'FontSize',25,'Interpreter','latex');
exportgraphics(f,filename,'Resolution',220);
end
