function J = stage1_target_cost(speed,cot,cfg)
% Dimensionless squared relative errors; w1 is speed, w2 is CoT.
J=cfg.w1*((speed-cfg.targetSpeed)/cfg.targetSpeed).^2 ...
 +cfg.w2*((cot-cfg.targetCoT)/cfg.targetCoT).^2;
end
