function cfg=basic_refinement_config(base,parent,k,basic,radius)
% Resample a parent's shape and box widths when changing knot count/crossing.
old=parent.configuration; a=parent.variables; cfg=base; cfg.grfK=k;
width=old.upper-old.lower; center=cfg.initial; mappedWidth=cfg.upper-cfg.lower;
center(1:6)=a(1:6); mappedWidth(1:6)=width(1:6);
oldNodes=setdiff(2:old.grfXN-1,parent.k);
newNodes=setdiff(2:cfg.grfXN-1,k);
q=(newNodes-1)/(cfg.grfXN-1); crossing=(k-1)/(cfg.grfXN-1);
oldCrossing=(parent.k-1)/(old.grfXN-1);
u=q; before=q<crossing;
u(before)=q(before)*oldCrossing/crossing;
u(~before)=oldCrossing+(q(~before)-crossing)*(1-oldCrossing)/(1-crossing);
f=zeros(1,old.grfXN); f(oldNodes)=a(old.index.fx);
w=interp1((oldNodes-1)/(old.grfXN-1),width(old.index.fx),...
 linspace(0,1,old.grfXN),'linear','extrap'); w=max(0,w);
scale=(old.grfXN-1)/(cfg.grfXN-1);
center(cfg.index.fx)=interp1(linspace(0,1,old.grfXN),f,u)*scale;
mappedWidth(cfg.index.fx)=interp1(linspace(0,1,old.grfXN),w,u)*scale;
z=[0 a(old.index.fz) 0]; qz=(1:cfg.grfZN-2)/(cfg.grfZN-1);
sz=(old.grfZN-1)/(cfg.grfZN-1);
center(cfg.index.fz)=interp1(linspace(0,1,old.grfZN),z,qz)*sz;
center(cfg.index.fz)=center(cfg.index.fz)/sum(center(cfg.index.fz));
wz=[width(old.index.fz(1)),width(old.index.fz),width(old.index.fz(end))];
mappedWidth(cfg.index.fz)=interp1(linspace(0,1,old.grfZN),wz,qz)*sz;
for pair={'betaXStart','betaXEnd','betaZStart','betaZEnd'}
 name=pair{1}; factor=scale; if contains(name,'Z'), factor=sz; end
 center(cfg.index.(name))=a(old.index.(name))*factor;
 mappedWidth(cfg.index.(name))=width(old.index.(name))*factor;
end
center=min(cfg.upper,max(cfg.lower,center));
center=seed_horizontal_impulse(center,cfg,false);
center(1:3)=basic;
cfg.initial=center;
cfg.lower=max(cfg.lower,center-radius*mappedWidth);
cfg.upper=min(cfg.upper,center+radius*mappedWidth);
cfg.lower(1:3)=basic; cfg.upper(1:3)=basic;
cfg.freeIndices=find(cfg.upper>cfg.lower+1e-14);
cfg.freeIndices=setdiff(cfg.freeIndices,1:3);
cfg.adaptiveLocal=true;
end
