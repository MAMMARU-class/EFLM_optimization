# Hermite optimization progress and restart

`run_stage1_optimization` uses `broadStarts=220000` total broad starts,
`localStarts=10000`, and `save_interval=1000` by default.
Broad starts are divided evenly across the selected k values, with any remainder
assigned to the first k values. Hermite impulse-compatible initial points are
retained. The narrow search uses the best k found by the broad search.
Broad search uses 121 nodes; narrow search and final verification use 1201 nodes.

Progress and feasible percentages update on one line. Warnings are disabled
during optimization and restored on exit. Failure details go to `report.txt`.

Each k and the narrow search save their initial points and completed chunks in
the run folder's `checkpoints` directory. Resume with:

```matlab
[result, folder] = run_stage1_optimization(struct('resumeFolder', 'FULL_RUN_FOLDER'));
```

Saved settings are authoritative. Completed chunks are loaded; an interrupted
chunk is rerun from its beginning. Dense refinement restarts from the saved
narrow result. Runs created before this checkpoint system cannot be resumed.
The same random seed is used, but the Hermite and B-spline parameterizations
require different initial point coordinates.
