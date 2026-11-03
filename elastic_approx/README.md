# elastic_approx

MATLAB **R2026b**, Optimization Toolbox and Global Optimization Toolbox are required.
Run from the project root:

```matlab
addpath('elastic_approx');
validate_elastic_approx;
[result, folder] = main_elastic_approx;
```

The implementation is confined to `common` and `elastic_approx`. Historical sources,
`rigid`, and `elastic_exact` are unchanged. Units are SI. Total mass is 1.93741375 kg;
spring damping is zero. Numeric physical values are taken from the old parameter files.

## Computation

The massless-leg DAE is reduced exactly in its differential part: prescribed sinusoidal
forces are analytically integrated for body translation. The two coupled algebraic
equations (reference geometry and spring equilibrium) are solved simultaneously at all
stance times with a damped Newton method. No iterative ODE integrator is necessary.
A precomputed one-dimensional geometry spline is an acceleration of geometry evaluation,
not a grid search over optimization variables. Joint rates follow implicit differentiation;
motor torques use equations (17) and (18). The full Jacobian and residuals are saved.

Broad and narrow stages both use MATLAB MultiStart with SQP/fmincon. Variables are scaled
to the unit box. The broad bounds are `[0 N, .5 deg, .03 m/s, .10]` through
`[50 N, 35 deg, 2 m/s, .90]`.
Both stance and flight must last at least 0.2 seconds. The narrow box is +/-15% of each broad interval about
the broad-stage solution. Defaults are 16 and 5 starts, seed 20261103. This does not
certify an unrestricted global optimum. Bound proximity is recorded for review.

Reference leg length is 0.340 m. The added minimum durations made the 0.300 m initial
posture violate the hip lower limit during midstance; the new nominal length admits
`[.5 N, 8 deg, 2*.340*sin(8 deg)/.2 m/s, .5]` with both durations 0.2 s.
Physical link lengths remain unchanged. Time nodes cluster near touchdown/liftoff.
Broad/narrow optimization uses the maximum sampled violation for each constraint type.
Final refinement selects individual active time constraints and adds new violated times,
avoiding the large SQP subproblems of a full time-by-constraint matrix. A validated
feasible incumbent is preserved. Final constraints are checked on 1201 stance nodes;
feasibility between sample times is not certified.

The solver stops a failed candidate and returns a penalty. Unexpected programming errors
are rethrown. Each stage has a wall-time budget; each candidate has a separate budget.
Global-stage checkpoints survive later-stage failure. Objective and constraints share
their last evaluation; plots and movies are generated only after optimization.

## Reproducibility and visualization

Results are saved to `_logs/elastic_approx/<timestamp>/`. `result.mat` contains trajectories,
parameters, solver settings, starts, RNG states, stage results, residuals, torques, rates,
power, integrated work and geometry. `source/` stores the actual source used in the run.
CSV files and MATLAB figures are supplemental exports. Regenerate plots/video with:

```matlab
addpath(genpath('common'),genpath('elastic_approx'));
recreate_outputs(fullfile(folder,'result.mat'));
```

The MP4 depicts the EFLM closed-link shape. The approximate SLIP geometry and exact
closed-link rendering differ by the approximation error, saved in `exactToeResidual`.
Flight body motion is ballistic and the leg is hidden during flight. The display x axis
is reversed for leftward motion without changing simulation coordinates. Rigid links
are black; the elastic link is a blue zigzag spring, following the old renderer.
No swing dynamic feasibility is claimed.

`smoke_outputs_elastic_approx` checks serialization/plot/video without either optimization
toolbox and writes `diagnostic_result.mat`, explicitly marked **NOT optimized**.
