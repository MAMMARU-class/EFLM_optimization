# B-spline comparison

Run `run_stage1_bspline()`; use a struct to override named configuration fields.
The Hermite entry point remains `run_stage1_optimization()`.
Broad and narrow MultiStart use 100-start chunks by default (`save_interval`).
Completed chunks are saved atomically under `checkpoints/`, alongside the exact
start points. Progress/feasible percentages update in the Command Window.
To resume after interruption, run
`run_stage1_bspline(struct('resumeFolder','<existing run folder>'))`.
Saved settings control the resumed run; an interrupted chunk is rerun from its
start. Broad and narrow completed chunks are reused. Dense polishing restarts
from the saved narrow winner if interrupted.

Reference: `_logs/stage1_point_mass/20261109_165106_197`.
Basic bounds, objective, spring/motor model, minimum stance/flight times (0.2 s each),
seed, SQP tolerances/iteration limits, 20 workers, 121 broad/narrow nodes and 1201
final nodes are retained. Broad starts are 22000 total, narrow starts are 1000.
No discrete crossing index is used.

Cubic open-uniform splines have zero first/last coefficients. Horizontal: 16
coefficients, 14 interior force coefficients in N, one linear zero-integral
constraint, hence 13 shape/amplitude degrees of freedom. Vertical: 11
coefficients, 9 nonnegative interior coefficients and sum=1, hence 8 shape degrees
of freedom. With 5 basic/timing variables, effective dimension is 26, matching
the reference's 29 variables minus 3 equalities. Degrees of freedom match, not
the number of control points. Additional mechanical constraints still apply.

The horizontal coefficient bound is a configurable +/-100 N trial bound. It is
not equivalent to the old Ix bound [0.1,4] N s. Crossing topology is unrestricted.
Vertical amplitude satisfies total impulse mg*T. Basis integrals and first
moments are evaluated using exact polynomial Gauss quadrature per knot span,
cached for each fixed evaluation grid. No absolute-value quadrature is used in
the objective/constraints.

Let Q=integral_0^1 (1-u)*Fx(u) du / m and L=2*abs(r0*sin(thetaL)).
Solve Q*TsBase^2+vx0*TsBase-L=0 using
TsBase=2*L/(vx0+sqrt(vx0^2+4*Q*L)). This selects the smaller positive root when
Q<0 (positive endpoint derivative of displacement with respect to duration).
Nonpositive discriminants are rejected. Then Ts=alpha*TsBase, Tf=Ts*(1/D-1).
Since force amplitude is fixed in N, actual stance displacement is
vx0*Ts+Q*Ts^2, not alpha*L in general. This differs from impulse-controlled
Hermite time scaling and must be considered when interpreting comparisons.

The actual absolute horizontal impulse is computed once after optimization by
splitting cubic spans at real roots, and saved in the result and report.

Output: settings, result, source snapshot, trajectory/flight/constraint CSV,
plots and video, and coefficient CSV files. Coefficients are not sampled curve
values; full knot vectors and the vertical scale are in result.final.body.bspline.
The reference's final valid flag is false; compare constraint violation as well
as objective and trajectories.
