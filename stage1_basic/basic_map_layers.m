function [layers,baseRows]=basic_map_layers(sourceFolder,currentRows)
% Keep the phase1 grid as the canvas and add only the searched later cells.
name=string(getLastFolder(sourceFolder));
if ~startsWith(name,"phase")
 layers=struct('rows',currentRows,'division',1);
 baseRows=currentRows;
 return
end
runFolder=fileparts(sourceFolder);
baseFile=fullfile(runFolder,'phase1','summary.csv');
if ~isfile(baseFile)
 error('EFLM:MissingPhase1','Adaptive maps require %s',baseFile);
end
baseRows=readtable(baseFile);
layers=struct('rows',baseRows,'division',1);
if name=="phase1", return; end
phase2File=fullfile(runFolder,'phase2','summary.csv');
if ~isfile(phase2File), error('EFLM:MissingPhase2','Missing %s',phase2File); end
phase2Rows=readtable(phase2File);
% Phase2 uses the same basic grid. Keep the best result from either phase.
layers(end+1)=struct('rows',[baseRows;phase2Rows],'division',1);
if name=="phase2", return; end
number=sscanf(char(name),'phase3.%d');
if isempty(number), error('EFLM:InvalidPhase','Unsupported phase folder %s',name); end
for i=1:number
 phaseFile=fullfile(runFolder,sprintf('phase3.%d',i),'summary.csv');
 if ~isfile(phaseFile), error('EFLM:MissingPhase','Missing %s',phaseFile); end
 if i==number, phaseRows=currentRows; else, phaseRows=readtable(phaseFile); end
 layers(end+1)=struct('rows',phaseRows,'division',3^i);
end
end

function name=getLastFolder(path)
[~,name,extension]=fileparts(char(path));
name=[name extension];
end
