# stage1 の目標値に基づく目的関数

現在の `stage1_point_mass` では、次の無次元の目的関数を最小化する。旧 `mean_velocity` / `initial_velocity` は archive に保存されている。

\[
\min J_{\mathrm{cost}}
=w_1\left(\frac{\bar v-v_{\mathrm{target}}}{v_{\mathrm{target}}}\right)^2
+w_2\left(\frac{\mathrm{CoT}-\mathrm{CoT}_{\mathrm{target}}}{\mathrm{CoT}_{\mathrm{target}}}\right)^2.
\]

速度は1周期の移動距離を周期時間で割った平均水平速度であり、初期水平速度ではない。目標値は `targetSpeed=4` m/s、`targetCoT=0.3`。目標値で割った相対誤差を二乗することで、異なる単位の指標を比較し、符号による誤差の相殺を防ぐ。目標値を上回る場合も下回る場合も、同じ相対誤差なら同じコストになる。したがって、CoT が目標より低い場合にも差に応じたコストを与える。

プロジェクト直下の `run_stage1_optimization.m` の `overrides.targetSpeed`、`overrides.targetCoT`、`overrides.w1`、`overrides.w2` から調整できる。現在の実行プリセットの重みは `w1=2`、`w2=1`。引数に構造体を渡す場合は、未指定項目が `optimization_settings.m` の既定値になるため、重みも明示する。

```matlab
options = struct('targetSpeed',4,'targetCoT',0.3,'w1',2,'w2',1);
[result,folder] = run_stage1_optimization(options);
```

目標値は正の有限値、重みは非負の有限値とし、少なくとも一方の重みを正にする。目標値は等式制約ではなく、機構・モータなどの制約の下で近づける評価基準である。計算失敗時は既存のペナルティを使用する。設定値は `settings.mat`、`result.mat` と `report.txt` に保存される。

実装は `common/function/stage1_target_cost.m`。立脚時間倍率の設定は [stage1_stance_multiplier.md](stage1_stance_multiplier.md) を参照。
