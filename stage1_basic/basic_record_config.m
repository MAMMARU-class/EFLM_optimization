function cfg=basic_record_config(record,fallback)
% Retained parents can have a different discretization and search box.
cfg=fallback;
if isfield(record,'configuration'), cfg=record.configuration; end
cfg.grfK=record.k;
end
