function save_run_outputs(result,folder)
r=result.final; b=r.body;
data=[b.t,b.x,b.z,b.vx,b.vz,b.Fx,b.Fz,r.s,r.sd,r.l,r.ld,r.q,r.qd,...
 r.angles,r.angularVelocity,r.tau,r.power,r.rpm,r.torqueLower,r.torqueUpper,...
 r.equilibriumResidual,r.exactToeResidual];
names={'t','x','z','vx','vz','Fx','Fz','ds','ds_dt','l','l_dt','q','q_dt',...
 'qh','qk','qa','dqh','dqk','dqa','tauH','tauK','powerH','powerK','rpmH','rpmK',...
 'tauLowerH','tauLowerK','tauUpperH','tauUpperK','geometryResidual','forceResidual','toeErrorX','toeErrorZ'};
writetable(array2table(data,'VariableNames',names),fullfile(folder,'trajectory.csv'));
writetable(array2table([b.Ts+b.flightTime,b.flightX,b.flightZ,b.flightVx,b.flightVz],...
 'VariableNames',{'t','x','z','vx','vz'}),fullfile(folder,'flight.csv'));
variableNames={'AmpH','thetaL','v','Duty'};
if numel(result.bestVariables)==5, variableNames{5}='r0'; end % Old logs remain readable.
writetable(array2table([result.bestVariables,b.speed,r.cot,r.energy,b.distance,b.T,b.Ts,b.Tf,r.maxViolation,result.valid],...
 'VariableNames',[variableNames,{'speed','CoT','energy','distance','period','stanceTime','flightTime','maxViolation','valid'}]),...
 fullfile(folder,'summary.csv'));
if isfield(r,'constraintChannels')
 names={'torqueUpperH','torqueUpperK','torqueLowerH','torqueLowerK',...
 'angleUpperH','angleUpperK','angleLowerH','angleLowerK','springUpper','springLower',...
 'rpmH','rpmK','height'};
 writetable(array2table([b.t,r.constraintChannels],'VariableNames',[{'t'},names]),...
  fullfile(folder,'constraint_residuals.csv'));
end
plots=fullfile(folder,'plots'); if ~isfolder(plots), mkdir(plots); end
% Match the reference comparison plots in _logs/test/20261105_161428_342.
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
exportgraphics(f,fullfile(plots,'stance.png'),'Resolution',220);
savefig(f,fullfile(plots,'stance.fig'));
end
