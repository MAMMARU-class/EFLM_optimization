# B-spline comparison

Run `run_stage1_bspline()`; use a struct to override named configuration fields.
The Hermite entry point remains `run_stage1_optimization()`.
Broad and narrow MultiStart use 100-start chunks by default (`save_interval`).
Completed chunks are saved atomically under `checkpoints/`, alongside the exact
start points. Progress/feasible percentages update in the Command Window.
To resume after interruption, run
`run_stage1_bspline(struct('resumeFolder','<existing run folder>'))`.
After stopping the running MATLAB optimization, use
`run_stage1_bspline(struct('resumeFolder','<existing run folder>', ...
'finishBroadEarly',true))` to finish broad search at its last contiguous,
completed checkpoint and continue to narrow search. The planned broad start
count in `settings.mat` remains unchanged. The early-stop decision is saved in
`checkpoints/broad_early_stop.mat`, so an interrupted narrow search resumes
from the same broad cutoff even with the ordinary resume command. An unfinished
broad chunk is never counted. At least one completed broad chunk is required.
Saved settings control the resumed run; an interrupted chunk is rerun from its
start. Broad and narrow completed chunks are reused. Dense polishing restarts
from the saved narrow winner if interrupted.

Start points are generated and saved one `save_interval` chunk at a time.
Optimizer callbacks never retain the full start-point matrix. Each new chunk
uses a deterministic private random stream; new runs therefore differ from
the earlier all-at-once random sequence. Existing runs retain their exact
saved points: legacy v7 start files are converted once to v7.3 for partial
reads. That conversion temporarily needs memory for one full legacy array
and disk space for an additional copy, but does not distribute it to workers.

Reference: `_logs/stage1_point_mass/20261109_165106_197`.
Basic bounds, objective, spring/motor model, minimum stance/flight times (0.2 s each),
seed, SQP tolerances/iteration limits, and 20 workers are retained. Broad search
uses 121 nodes; narrow search and final verification use 1201 nodes.
No discrete crossing index is used.

Cubic open-uniform splines have zero first/last coefficients. Horizontal: 16
coefficients, 14 interior force coefficients in N, one linear zero-integral
constraint, hence 13 shape/amplitude degrees of freedom. Vertical: 11
coefficients, 9 nonnegative interior coefficients and sum=1, hence 8 shape degrees
of freedom. With 5 basic/timing variables, effective dimension is 26, matching
the reference's 29 variables minus 3 equalities. Degrees of freedom match, not
the number of control points. Additional mechanical constraints still apply.

The horizontal coefficient bound is +/-50 N, so convex-hull containment keeps
the full horizontal GRF in [-50,50] N. Vertical coefficients remain normalized
shape variables; no 80 N bound is imposed on the resulting vertical GRF.
The horizontal bound is not equivalent to the old Ix bound [0.1,4] N s.
Crossing topology is unrestricted.
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
