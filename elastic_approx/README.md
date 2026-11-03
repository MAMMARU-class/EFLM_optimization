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
to the unit box. The initial broad bounds are `[0 N, 3 deg, .15 m/s, .45]` through
`[20 N, 25 deg, 2.5 m/s, .90]`. The narrow box is +/-15% of each broad interval about
the broad-stage solution. Defaults are 16 and 5 starts, seed 20261103. This does not
certify an unrestricted global optimum. Bound proximity is recorded for review.

Reference leg length is 0.300 m. The initially proposed 0.277 m made even a 3-degree
touchdown violate the 55-degree hip lower limit, independent of speed. The revised length
admits the starting point `[2.5 N, 5 deg, .8 m/s, .8]` in the high-density diagnostic.
Final constraints are checked on 1201 stance nodes; a violated dense mesh triggers an
additional constrained refinement. Feasibility between sample times is not certified.

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
Flight body motion is ballistic; flight joint interpolation is explicitly illustrative,
excluded from energy and constraint evaluation. No swing dynamic feasibility is claimed.

`smoke_outputs_elastic_approx` checks serialization/plot/video without either optimization
toolbox and writes `diagnostic_result.mat`, explicitly marked **NOT optimized**.
