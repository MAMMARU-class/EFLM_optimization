function write_basic_pca_report(fid,a,caseId)
% Global variance fractions and per-trajectory centered-shape fractions differ.
if nargin<3, caseId=[]; end
fprintf(fid,'\nGRF PCA analysis\n----------------\n');
fprintf(fid,'Analysis time: %s\n',a.createdAt);
fprintf(fid,'Phase: s=t/Tstance; %d equally spaced samples including endpoints.\n',numel(a.phase));
fprintf(fid,'Normalization: trapz(s,abs(Fx))=1; trapz(s,Fz)=1; separate centered PCA by SVD.\n');
fprintf(fid,'No pointwise variance standardization. All saved feasible cases have equal weight.\n');
fprintf(fid,'Global contribution: eigenvalue / sum(eigenvalues).\n');
fprintf(fid,'Case contribution: score_j^2 / sum((wave-mean).^2); cumulative sums over PCs.\n');
fprintf(fid,'RGB: R=PC1, G=PC2, B=PC3 SIGNED scores; each channel uses its own max(abs(score)).\n');
fprintf(fid,'Channel=round(255*(1+score/maxAbsScore)/2): -max->0, zero->128, +max->255.\n');
fprintf(fid,'One common RGB scale per direction across all maps; 256 levels (0-255).\n');
fprintf(fid,'Loading sign: largest absolute loading is positive. Scores retain their sign.\n');
fprintf(fid,'Black=no feasible case; missing PCA data are gray. Mean-wave scores map to neutral gray.\n');
fprintf(fid,'Mean horizontal/vertical global cumulative PC1 / PC1-2 / PC1-3 [%%]: %s\n',rowText(a.meanGlobalCumulative));
names={'horizontal','vertical'};
for d=1:2
 m=a.(names{d});
 fprintf(fid,'\n%s GRF\n',names{d});
 fprintf(fid,'  Status: %s; usable feasible cases: %d; numerical rank: %d\n',m.status,nnz(m.valid),m.rank);
 fprintf(fid,'  Global PC1 / PC2 / PC3 contribution [%%]           : %s\n',rowText(m.explained));
 fprintf(fid,'  Global cumulative PC1 / PC1-2 / PC1-3 [%%]         : %s\n',rowText(m.cumulative));
 fprintf(fid,'  Mean case cumulative PC1 / PC1-2 / PC1-3 [%%]      : %s\n',rowText(m.meanCaseCumulative));
 fprintf(fid,'  Shape variations: mean +/- sqrt(lambda_j)*loading_j.\n');
 fprintf(fid,'  sqrt(lambda_1), sqrt(lambda_2), sqrt(lambda_3)    : %s\n',rowText(m.sigma));
 fprintf(fid,'  RGB symmetric score limits (PC1,PC2,PC3)        : %s\n',rowText(m.rgbScoreLimit));
 if ~isempty(caseId)
  i=find(a.caseIds==caseId,1);
  if ~isempty(i)
   fprintf(fid,'  This case PC1 / PC2 / PC3 scores                  : %s\n',rowText(m.scores(i,:)));
   fprintf(fid,'  This case RGB [0-255]                            : %s\n',rowText(round(255*m.rgb(i,:))));
   fprintf(fid,'  This case PC1 / PC2 / PC3 contribution [%%]        : %s\n',rowText(m.contribution(i,:)));
   fprintf(fid,'  This case cumulative PC1 / PC1-2 / PC1-3 [%%]      : %s\n',rowText(m.caseCumulative(i,:)));
  end
 end
end
fprintf(fid,'\nPlots: grid_search_result/pca_*.png\n');
fprintf(fid,'Numerical results: grid_search_result/grf_pca.mat, grf_pca_scores.csv, grf_pca_waveforms.csv\n');
fprintf(fid,'Mode variations are descriptive; they need not satisfy all physical constraints.\n');
end

function s=rowText(x)
s=strtrim(sprintf('%.12g ',x));
end
