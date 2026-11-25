function validate_independent_grf()
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')),fullfile(root,'stage1_point_mass'));
for kind={'bspline','hermite'}
 [cfg,p]=grf_optimization_settings(kind{1},struct('useParallel',false,'video',false));
 u=linspace(0,1,401)'; op=grf_operators(kind{1},11,u);
 assert(max(abs(sum(op.B,2)-1))<1e-12);
 assert(max(abs(sum(op.I,2)-u))<1e-12);
 assert(max(abs(sum(op.J,2)-u.^2/2))<1e-12);
 if strcmp(kind{1},'hermite')
  nodes=grf_operators('hermite',11,linspace(0,1,11));
  assert(max(abs(nodes.B-eye(11)),[],'all')<1e-12);
  expected=zeros(1,11); expected(1:2)=[-10 10];
  assert(max(abs(op.D(1,:)-expected))<1e-12);
  expected=zeros(1,11); expected(end-1:end)=[-10 10];
  assert(max(abs(op.D(end,:)-expected))<1e-12);
 end
 h=1e-6; mid=[.137;.427;.783];
 before=grf_operators(kind{1},11,mid-h); after=grf_operators(kind{1},11,mid+h);
 center=grf_operators(kind{1},11,mid);
 assert(max(abs((after.B-before.B)/(2*h)-center.D),[],'all')<1e-6);
 assert(max(abs((after.I-before.I)/(2*h)-center.B),[],'all')<1e-7);
 assert(max(abs((after.J-before.J)/(2*h)-center.I),[],'all')<1e-7);
 problem=struct('lb',zeros(size(cfg.lower)),'ub',ones(size(cfg.upper)),...
  'x0',(cfg.initial-cfg.lower)./(cfg.upper-cfg.lower));
 stream=RandStream('mt19937ar','Seed',42);
 starts=grf_start_points(stream,100,cfg,problem,true,true);
 for row=1:size(starts,1)
  a=cfg.lower+starts(row,:).*(cfg.upper-cfg.lower);
  b=prescribed_grf_motion(a,p,121,cfg);
  assert(max(abs(b.periodic))<1e-10);
  assert(abs(b.horizontalGRF.normalizedImpulse)<1e-12);
  assert(abs(b.verticalGRF.normalizationResidual)<1e-12);
  assert(max(b.horizontalGRF.shapeConstraints)<1e-10);
  assert(b.T>=cfg.minGaitDuration-1e-12);
  assert(max(abs([b.Fx([1 end]);b.Fz([1 end])]))<1e-10);
  if strcmp(kind{1},'bspline')
   assert(max(abs(b.Fx)-p.frictionCoefficient*b.Fz)<1e-10);
  else
   % Analytic interval minima must detect every sampled violation.
   assert(max(b.verticalGRF.shapeConstraints)+1e-10>=max([0;-b.Fz;abs(b.Fx)-p.frictionCoefficient*b.Fz])/(p.m*p.g));
  end
 end
 problem.lb=max(0,problem.x0-.15); problem.ub=min(1,problem.x0+.15);
 narrow=grf_start_points(stream,20,cfg,problem,false,true);
 assert(all(narrow>=problem.lb-1e-12 & narrow<=problem.ub+1e-12,'all'));
 for row=1:size(narrow,1)
  b=prescribed_grf_motion(cfg.lower+narrow(row,:).*(cfg.upper-cfg.lower),p,121,cfg);
  assert(max(abs(b.periodic))<1e-10 && max(b.horizontalGRF.shapeConstraints)<1e-10);
 end
 tab=build_exact_geometry_cache(p); found=false;
 % At least one mechanically evaluable case verifies constraint dimensions.
 for row=1:size(starts,1)
  a=cfg.lower+starts(row,:).*(cfg.upper-cfg.lower);
  try, r=sim_cycle_stage1_point_mass(a,p,tab,cfg,121);
  catch ME, if startsWith(ME.identifier,'EFLM:'), continue; else, rethrow(ME); end, end
  assert(numel(r.constraintEnvelope)==cfg.constraintChannelCount+cfg.shapeConstraintCount+4);
  assert(numel(r.constraints)==121*cfg.constraintChannelCount+cfg.shapeConstraintCount+4);
  toeVector=r.pointsLocal(:,:,4)-r.pointsLocal(:,:,6);
  toeAngle=atan2(toeVector(:,2),toeVector(:,1));
  assert(max(abs(r.constraintChannels(:,17)+toeAngle/pi))<1e-10);
  found=true; break;
 end
 assert(found,'No mechanically evaluable test case.');
 fprintf('%s: integration, periodicity, starts, cone, toe and constraint dimensions passed.\n',kind{1});
end
end
