function analysis = analyze_basic_grf_pca(folder)
% Analyze saved feasible GRFs without rerunning optimization or leg geometry.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root,'common')));
saved=load(fullfile(folder,'settings.mat'),'cfg','p');
rows=readtable(fullfile(folder,'summary.csv'));
phase=linspace(0,1,101)';
count=height(rows); fx=nan(count,numel(phase)); fz=fx;
areaX=nan(count,1); areaZ=areaX; stanceTime=areaX;
for i=find(rows.feasible==1)'
 item=load(fullfile(folder,'cases',sprintf('%06d.mat',rows.caseId(i))),'record');
 cfg=basic_record_config(item.record,saved.cfg);
 body=prescribed_motion_basic(item.record.variables,saved.p,3,cfg);
 x=sampleCubic(body.horizontalGRF.coefficients,phase)*body.horizontalGRF.amplitude;
 z=sampleCubic(body.verticalGRF.coefficients,phase)*body.verticalGRF.amplitude;
 areaX(i)=trapz(phase,abs(x)); areaZ(i)=trapz(phase,z); stanceTime(i)=body.Ts;
 if isfinite(areaX(i)) && areaX(i)>0, fx(i,:)=x'/areaX(i); end
 if isfinite(areaZ(i)) && areaZ(i)>0, fz(i,:)=z'/areaZ(i); end
end
analysis=struct('version',1,'phase',phase,'caseIds',rows.caseId,...
 'feasible',rows.feasible==1,'stanceTime',stanceTime,'areaX',areaX,'areaZ',areaZ,...
 'createdAt',char(datetime('now','TimeZone','Asia/Tokyo')),...
 'horizontal',fitShapePCA(fx),'vertical',fitShapePCA(fz));
analysis.meanGlobalCumulative=mean([analysis.horizontal.cumulative;analysis.vertical.cumulative],1);
output=fullfile(folder,'grid_search_result');
if ~isfolder(output), mkdir(output); end
save(fullfile(output,'grf_pca.mat'),'analysis','-v7');
writeData(output,rows,analysis);
plot_basic_grf_pca(rows,analysis,output);
updateReport(fullfile(folder,'report.txt'),analysis,[]);
ranked=sortrows(rows(rows.feasible==1,:),{'cost','caseId'});
for rank=1:min(saved.cfg.topResults,height(ranked))
 filename=fullfile(folder,sprintf('report_no%d.txt',rank));
 if ~isfile(filename)
  result=reconstruct_basic_case(folder,ranked.caseId(rank));
  write_basic_rank_report(result,folder,rank,ranked.caseId(rank));
 end
 updateReport(filename,analysis,ranked.caseId(rank));
end
fprintf('GRF PCA: %d horizontal / %d vertical trajectories; outputs: %s\n',...
 nnz(analysis.horizontal.valid),nnz(analysis.vertical.valid),output);
end

function y=sampleCubic(coef,s)
n=size(coef,1); segment=min(n,floor(s*n)+1); u=s*n-(segment-1);
c=coef(segment,:); y=((c(:,1).*u+c(:,2)).*u+c(:,3)).*u+c(:,4);
end

function model=fitShapePCA(waves)
[n,p]=size(waves); valid=all(isfinite(waves),2);
model=struct('waves',waves,'valid',valid,'mean',nan(1,p),...
 'coefficients',zeros(p,3),'sigma',zeros(1,3),'scores',nan(n,3),...
 'explained',nan(1,3),'cumulative',nan(1,3),...
 'contribution',nan(n,3),'caseCumulative',nan(n,3),'rgb',nan(n,3),...
 'meanCaseCumulative',nan(1,3),'rgbScoreLimit',zeros(1,3),'rank',0,'status','insufficient_variation');
if ~any(valid), return; end
model.mean=mean(waves(valid,:),1); centered=waves(valid,:)-model.mean;
model.scores(valid,:)=0; model.rgb(valid,:)=128/255;
if nnz(valid)<2, return; end
[~,S,V]=svd(centered,'econ'); eigenvalues=diag(S).^2/(nnz(valid)-1);
total=sum(eigenvalues);
if total<=eps(max(1,norm(waves(valid,:),'fro')^2)), return; end
model.rank=nnz(eigenvalues>max(eigenvalues)*max(size(centered))*eps);
q=min(3,size(V,2)); coefficients=V(:,1:q);
% Fix the arbitrary SVD sign to make repeated reports visually consistent.
for j=1:q
 [~,peak]=max(abs(coefficients(:,j)));
 if coefficients(peak,j)<0, coefficients(:,j)=-coefficients(:,j); end
end
model.coefficients(:,1:q)=coefficients;
model.sigma(1:q)=sqrt(eigenvalues(1:q))';
model.scores(valid,1:q)=centered*coefficients;
model.explained=zeros(1,3); model.explained(1:q)=100*eigenvalues(1:q)'/total;
model.cumulative=cumsum(model.explained);
energy=sum(centered.^2,2); ids=find(valid);
defined=energy>eps(max(1,max(energy)));
selected=ids(defined);
model.contribution(selected,:)=100*model.scores(selected,:).^2./energy(defined);
model.caseCumulative(selected,:)=cumsum(model.contribution(selected,:),2);
model.meanCaseCumulative=mean(model.caseCumulative(selected,:),1,'omitnan');
% Signed scores: the same symmetric range is used across every map of a PC.
model.rgbScoreLimit=max(abs(model.scores(valid,:)),[],1);
for j=1:3
 if model.sigma(j)>sqrt(total)*1e-12 && model.rgbScoreLimit(j)>0
  signed=model.scores(valid,j)/model.rgbScoreLimit(j);
  model.rgb(valid,j)=round(255*(1+min(1,max(-1,signed)))/2)/255;
 end
end
model.status='completed';
end

function writeData(folder,rows,a)
t=rows(rows.feasible==1,{'caseId','gridId','vx0','thetaL','r0','k','cost'});
mask=rows.feasible==1;
names={'horizontal','vertical'}; prefix={'x','z'};
for d=1:2
 m=a.(names{d});
 for j=1:3
  t.(sprintf('%s_PC%d_score',prefix{d},j))=m.scores(mask,j);
  t.(sprintf('%s_PC%d_contribution_percent',prefix{d},j))=m.contribution(mask,j);
  t.(sprintf('%s_PC1_to_%d_cumulative_percent',prefix{d},j))=m.caseCumulative(mask,j);
  t.(sprintf('%s_PC%d_RGB_channel',prefix{d},j))=round(255*m.rgb(mask,j));
 end
end
writetable(t,fullfile(folder,'grf_pca_scores.csv'));
t=table(a.phase,'VariableNames',{'phase'});
for d=1:2
 m=a.(names{d}); t.([prefix{d} '_mean'])=m.mean';
 for j=1:3
  delta=m.sigma(j)*m.coefficients(:,j);
  t.(sprintf('%s_PC%d_one_sigma',prefix{d},j))=delta;
  t.(sprintf('%s_PC%d_plus',prefix{d},j))=m.mean'+delta;
  t.(sprintf('%s_PC%d_minus',prefix{d},j))=m.mean'-delta;
 end
end
writetable(t,fullfile(folder,'grf_pca_waveforms.csv'));
end

function updateReport(filename,a,caseId)
if ~isfile(filename), return; end
text=fileread(filename); marker='GRF PCA analysis';
location=strfind(text,marker);
if ~isempty(location), text=text(1:location(1)-1); end
fid=fopen(filename,'w','n','UTF-8');
if fid<0, error('EFLM:ReportOpen','Cannot write %s',filename); end
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',strtrim(text));
write_basic_pca_report(fid,a,caseId);
end
