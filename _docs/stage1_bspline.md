# Shared GRF optimization (layout 4)

`run_stage1_bspline()` uses cubic open-uniform B-splines.
`run_stage1_optimization()` uses Hermite / Catmull-Rom curves.
Both use 11 controls in each axis, zero endpoints and the same search settings:
500,000 broad starts at 121 stance nodes, 10,000 narrow starts at 1,201 nodes,
1,201 final verification nodes, 20 process workers, checkpoints every 2,000
completed starts. The objective and mechanical model are unchanged.

The 23 variables are vx0, thetaL, r0, T_st, D, 9 actual horizontal controls
[N], and 9 nonnegative vertical shape controls. Two linear equalities leave
21 independent dimensions. There is no crossing index, absolute-impulse
parameter, independent endpoint slope, or stance-time multiplier.
Hermite interior slopes are half the neighboring difference; both endpoint
slopes are one-sided differences in local segment coordinates, in both axes.

## Time and force equations

T_st is independent, with bounds [0.05,0.25] s and initial value 0.15 s.
D remains in [0.3,0.6], initially 0.5. T=T_st/D, T_fl=T-T_st.
Only T>=0.25 s is imposed; the old separate 0.2 s minima do not apply.
With u=t/T_st, shared basis phi_i, w_i=integral(phi_i,0,1), and
v_i=integral(u*phi_i,0,1):

- sum(z_i)=1 and z_i>=0, with endpoint z_i=0.
- A_z=mg/(D*sum(w_i*z_i)), F_z=A_z*sum(z_i*phi_i).
- F_x=sum(x_i*phi_i), with sum(w_i*x_i)=0.
- v_z(0)=g*(T_st*sum(v_i*z_i)/sum(w_i*z_i)-T/2).

The optimizer enforces both equalities directly. Basis integrals and moments
are exact polynomial integrals; the control sum alone is not a force integral.
COM positions and velocities use analytic integrals of these bases.

mu=0.4; |x_i|<=mu*A_z*z_i. There is no independent +/-50 N force restriction.
A redundant finite box |x_i|<=mu*mg/(D_min*w_i) provides normalized solver
coordinates without removing any feasible control vector. Sampled trajectory
friction constraints remain. B-spline convex hulls guarantee continuous cone
containment from control constraints. Hermite additionally checks exact cubic
minima of F_z, mu*F_z-F_x and mu*F_z+F_x on every segment.

Broad starts normalize positive random z controls and project random actual Fx
controls onto their friction box intersected with the zero-impulse hyperplane.
Narrow starts also respect the local box and vertical normalization. Initial
starts satisfy these algebraic conditions, not necessarily mechanical limits.
Time samples obey T>=0.25. The first start retains the configured initial state;
the first narrow start retains the broad winner.

Joint limits, spring travel, torque-speed envelope, rpm limits, and v_z(0)<=0
remain. COM height is constrained during stance and flight (the minimum of the
concave ballistic flight is at an endpoint). During stance, the angle from
world +x to the vector from contact to the adjacent toe joint is in [0,pi].
This constrains the straight toe link to remain above the ground. Mechanical
constraints are checked at the specified nodes, not continuously in time.

## Save, stop and resume

Log roots remain `_logs/stage1_bspline` and `_logs/stage1_point_mass`, with
timestamp subfolders and source/settings snapshots. Completed chunks and
their exact starts are saved atomically. The unfinished chunk is rerun after
an interruption; completed chunks are reused. A private random stream per
chunk keeps new start generation independent of solver RNG consumption.
Progress and feasible percentages update on one Command Window line; warnings
are temporarily suppressed on the client and workers and restored afterward.

```matlab
run_stage1_bspline(struct('resumeFolder','<run folder>'))
run_stage1_optimization(struct('resumeFolder','<run folder>'))
```

After stopping a run, add `'finishBroadEarly',true` to switch to narrow search
from the best completed broad checkpoint. At least one completed chunk is
required. The saved early-stop marker also governs subsequent normal resumes.
Saved settings are authoritative: do not combine resumeFolder with search
overrides. Dense polishing restarts from the stored narrow winner.

Existing layout-3 B-spline and layout-2 Hermite logs retain their old equations
when resumed. They are never reinterpreted as layout 4; early broad stop for
legacy Hermite is not supported. To use the new equations, start a new run.
