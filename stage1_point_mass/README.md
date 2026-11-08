# stage1_point_mass — 単質点モデルの最適化

立脚時間は床反力形状の積分から求めた平均速度ベースの基準時間に、最適化する倍率を掛けて生成します。

`L=2*r0*sin(thetaL)`, `gamma=A*(CH-(n-1)*S)/(m*(n-1)^2)` として、
`Ts = stanceScale * 2*L/(v+sqrt(v^2-4*gamma*L))`。
力積残差 `S` を含むので探索途中の候補にも適用します。平方根内が負の候補は計算失敗のペナルティとします。

## 実行と設定

```matlab
[result,folder] = run_stage1_optimization();
```

`run_stage1_optimization.m` 内の探索範囲、初期値、目標値、重みを編集します。各最適化変数は `[lower, upper, initial]` の順で設定します。

```matlab
overrides.AmpH       = [1, 30, 4];
overrides.grfN       = 7;             % 等間隔Hermite制御点数
overrides.grfBetaEndpoint = [-0.4, 0, 0]; % 始点・終点共通の無次元傾き beta [下限, 上限, 初期値]
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
| 6 | 立脚時間倍率 stanceScale [-] |
| 7以降 | 床反力形状の自由な比率 n-4 個、共通端点傾き1個 |

水平床反力は `Fx=AmpH*f(t)` とし、区分3次Hermite補間で `0→負→正→0` の形状を生成します。
始点と終点に共通の無次元傾き beta を `[-0.4,0]` で最適化します。内部 beta は前後の点から計算し、最大値点 `j` は0です。物理時間の傾きは `b=beta/dt`。軌道中の水平速度下限制約は設けません。
力積ゼロ、符号順序、`abs(f)<=1`、摩擦円錐を制約とします。
zero-crossing の点番号 k と最大絶対値の点番号 j は広域段階で探索し、最良の組合せで狭域探索を行います。
`starts` は全組合せへの総開始点数であり、`(grfN-4)*(grfN-3)` 以上にしてください。
詳細と積分式は [水平床反力の仕様](../_docs/stage1_horizontal_hermite_grf.md) に記載しています。

構造体引数では必要な項目だけ上書きできます。例えば `options.stanceScale=[0.5,2.0,1.0]` とし、ほかの変数を省略できます。省略項目は `common/parameters/optimization_settings.m` の値になります。従来の `options.lower`、`options.upper`、`options.initial` は受け付けません。

結果は `_logs/stage1_point_mass/<日時>/` に保存します。目的関数は目標速度・CoTに対する相対誤差の重み付き二乗和です。

詳細は [_docs/stage1_stance_multiplier.md](../_docs/stage1_stance_multiplier.md) を参照してください。
