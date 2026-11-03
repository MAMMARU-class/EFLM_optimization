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
f=figure('Visible','off'); clean=onCleanup(@()close(f));
tiledlayout(3,2);
nexttile; plot(b.t,[b.x b.z]); ylabel('position [m]'); legend('x','z');
nexttile; plot(b.t,[b.Fx b.Fz]); ylabel('GRF [N]'); legend('Fx','Fz');
nexttile; plot(b.t,rad2deg(r.angles)); ylabel('angle [deg]'); legend('hip','knee','ankle');
nexttile; plot(b.t,r.tau); ylabel('torque [Nm]'); legend('hip','knee');
nexttile; plot(b.t,r.s*1000); ylabel('spring extension [mm]'); xlabel('time [s]');
nexttile; plot(b.t,r.power); ylabel('power [W]'); xlabel('time [s]');
exportgraphics(f,fullfile(plots,'stance.png')); savefig(f,fullfile(plots,'stance.fig'));
end
