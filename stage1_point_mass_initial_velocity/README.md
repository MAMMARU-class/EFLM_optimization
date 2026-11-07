# stage1_point_mass_initial_velocity

stage1_point_mass のコピーを基にした単質点の最適化モデルです。脚の幾何、床反力、目的関数、制約、最適化手順は共通です。

## 立脚時間

`Ts = 2*r0*sin(thetaL)/v_x0`

立脚時間の計算はこのフォルダの `prescribed_motion_*.m` にあります。`sim_cycle_*.m` がこれを呼びます。

## 実行

```matlab
[result, folder] = run_stage1_optimization('initial_velocity');
```

ログは `_logs/stage1_point_mass_initial_velocity/<日時>/` に保存されます。
探索範囲などの上書きはプロジェクト直下の `run_stage1_optimization.m` と `options` 引数で設定します。
元の `stage1_point_mass/` と過去のログは変更しません。
