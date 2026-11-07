# stage1_point_mass — 単質点モデルの最適化

立脚時間は初期速度から求めた基準時間に、最適化する倍率を掛けて生成します。

`Ts = stanceScale * 2*r0*sin(thetaL)/v`

## 実行と設定

```matlab
[result,folder] = run_stage1_optimization();
```

`run_stage1_optimization.m` 内の探索範囲、初期値、目標値、重みを編集します。各最適化変数は `[lower, upper, initial]` の順で設定します。

```matlab
overrides.AmpH       = [1, 30, 4];
overrides.thetaL     = deg2rad([1, 80, 50]);
overrides.v          = [0.4, 7.0, 1.0];
overrides.Duty       = [0.01, 0.60, 0.10];
overrides.r0         = [0.22, 0.34, 0.30];
overrides.stanceScale = [0.4, 2.5, 1.0];
[result,folder] = run_stage1_optimization(overrides);
```

| 変数の順序 | 内容 |
|---|---|
| 1 | 水平床反力振幅 AmpH [N] |
| 2 | 初期脚角度 thetaL [rad] |
| 3 | 初期水平速度 v [m/s] |
| 4 | Duty [-] |
| 5 | 初期脚長 r0 [m] |
| 6 | 立脚時間倍率 stanceScale [-]、実行プリセット範囲0.4〜2.5、初期値1 |

構造体引数では必要な項目だけ上書きできます。例えば `options.stanceScale=[0.5,2.0,1.0]` とし、ほかの変数を省略できます。省略項目は `common/parameters/optimization_settings.m` の値になります。従来の `options.lower`、`options.upper`、`options.initial` は受け付けません。

結果は `_logs/stage1_point_mass/<日時>/` に保存します。目的関数は目標速度・CoTに対する相対誤差の重み付き二乗和です。

詳細は [_docs/stage1_stance_multiplier.md](../_docs/stage1_stance_multiplier.md) を参照してください。
