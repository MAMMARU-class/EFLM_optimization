function [lo,hi,rpm] = motor_envelope(dq,p)
% Same signed four-quadrant envelope as old; no torque clipping.
rpm=dq*30/pi;
hi=p.tauMax*ones(size(rpm)); lo=-hi;
idx=rpm>p.rpmConstant & rpm<p.rpmLimit;
hi(idx)=p.tnSlope*(p.rpmZero-rpm(idx));
hi(rpm>=p.rpmLimit)=-p.tauMax;
idx=rpm < -p.rpmConstant & rpm > -p.rpmLimit;
lo(idx)=-p.tnSlope*(rpm(idx)+p.rpmZero);
lo(rpm<=-p.rpmLimit)=p.tauMax;
end
