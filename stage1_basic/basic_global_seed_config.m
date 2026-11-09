function cfg=basic_global_seed_config(base,parent,k,basic)
% Global phase box with the resampled parent retained only as the first seed.
cfg=basic_refinement_config(base,parent,k,basic,.20);
cfg.lower=base.lower; cfg.upper=base.upper;
cfg.initial=min(cfg.upper,max(cfg.lower,cfg.initial));
cfg.initial(1:3)=basic;
cfg.freeIndices=base.freeIndices;
cfg.adaptiveLocal=false;
end
