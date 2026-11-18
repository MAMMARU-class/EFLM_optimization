function points=bspline_start_chunk(folder,label,chunk,total,cfg,problem)
% Keep all large start-point data outside optimizer callback workspaces.
first=(chunk-1)*cfg.save_interval+1;
last=min(total,first+cfg.save_interval-1); count=last-first+1;
path=fullfile(folder,sprintf('%s_points_%05d.mat',label,chunk));
nv=numel(cfg.lower);
if isfile(path)
 saved=load(path); points=saved.points;
 if saved.total~=total || saved.first~=first || size(points,1)~=count || size(points,2)~=nv
  error('EFLM:ResumeSettings','Start-point chunk does not match settings.');
 end
 return;
end
legacy=fullfile(folder,[label '_starts.mat']);
if isfile(legacy)
 % Old v7 files cannot be sliced without loading the entire array.
 % Convert once in this isolated function; never send that array to workers.
 streamPath=fullfile(folder,[label '_starts_stream.mat']);
 if ~isfile(streamPath)
  convertLegacy(legacy,streamPath,label,problem.x0,total,nv);
 end
 source=matfile(streamPath);
 if ~isequal(size(source,'starts'),[total nv])
  error('EFLM:ResumeSettings','Saved starts do not match settings.');
 end
 points=source.starts(first:last,:);
else
 % A private per-chunk stream makes restarting independent of solver RNG use.
 seed=mod(double(cfg.seed)+chunk-1+double(strcmp(label,'narrow'))*1000000000,2^32);
 stream=RandStream('mt19937ar','Seed',seed);
 if strcmp(label,'broad')
  lo=cfg.lower; width=cfg.upper-lo;
  physical=lo+rand(stream,count,nv).*width;
  op=bspline_operators(cfg.bsplineXN,1); w=op.I(2:end-1);
  c=physical(:,cfg.index.fx); c=c-(c*w')/(w*w')*w;
  c=c./max(1,max(abs(c),[],2)/cfg.bsplineForceBound);
  physical(:,cfg.index.fx)=c;
  z=physical(:,cfg.index.fz); physical(:,cfg.index.fz)=z./sum(z,2);
  if first==1, physical(1,:)=cfg.initial; end
  points=(physical-lo)./width;
 else
  points=problem.lb+rand(stream,count,nv).*(problem.ub-problem.lb);
  if first==1, points(1,:)=problem.x0; end
 end
end
temporary=[path '.tmp'];
save(temporary,'points','first','total','-v7'); movefile(temporary,path,'f');
end

function convertLegacy(path,destination,label,winner,total,nv)
if strcmp(label,'narrow')
 saved=load(path,'starts','broadWinner');
else
 saved=load(path,'starts');
end
if ~isequal(size(saved.starts),[total nv])
 error('EFLM:ResumeSettings','Legacy starts do not match settings.');
end
if strcmp(label,'narrow') && ~isequal(saved.broadWinner,winner)
 error('EFLM:ResumeSettings','Narrow starts belong to a different broad winner.');
end
temporary=[destination '.tmp'];
save(temporary,'-struct','saved','-v7.3');
movefile(temporary,destination,'f');
end
