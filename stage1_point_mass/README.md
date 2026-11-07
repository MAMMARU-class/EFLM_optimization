# stage1_point_mass — 単質点モデルによる運動最適化

旧 elastic_exact を第1段階の単質点モデルとして改名しました。総質量を胴体の1点に集約し、質量のない脚は伸びを含むEFLMの厳密な閉リンク幾何で計算します。

## 実行方法

プロジェクト直下で実行します。

```matlab
[result, folder] = run_stage1_optimization();
```

探索範囲・初期値・重み・開始点数・ワーカー数は、プロジェクト直下の `run_stage1_optimization.m` で編集できます。引数による上書きも可能です。

```matlab
options.lower = [1, deg2rad(1), 0.4, 0.01, 0.22];
options.upper = [30, deg2rad(80), 7.0, 0.60, 0.34];
options.initial = [4, deg2rad(50), 1.0, 0.10, 0.29];
options.starts = 1000;
options.localStarts = 200;
options.useParallel = true;
options.parallelWorkers = 16;
options.w1 = 2;
options.w2 = 1;
[result, folder] = run_stage1_optimization(options);
```

引数を渡した場合、未指定項目には `common/parameters/optimization_settings.m` の値が適用されます。

| 項目 | 内容 |
|---|---|
| 変数順 | `[AmpH, thetaL, v, Duty, r0]` |
| 単位 | N、rad、m/s、無次元、m |
| 新規ログ | `_logs/stage1_point_mass/<日時>/` |
| 保存内容 | MAT、CSV、report.txt、グラフ、動画、設定、ソース |
| 目的関数 | `w2*CoT - w1*meanSpeed` を最小化 |
| 最適化 | 広域MultiStart → 狭域MultiStart → 高密度制約確認・修正 |
| 環境 | R2026b、Optimization / Global Optimization / Parallel Computing Toolbox |
| 再描画 | `recreate_outputs(fullfile(folder,'result.mat'))` |
| モデル検証 | `validate_stage1_point_mass` |

## 計算方法

立脚中の水平床反力は `Fx=-AmpH*sin(2*pi*t/Ts)` とします。平均水平速度は `v-AmpH*Ts/(2*pi*m)` であり、水平移動量を `2*r0*sin(thetaL)` と一致させて立脚時間を求めます。

\[
 A=\mathrm{AmpH},\quad B=2\pi m v,\quad C=4\pi m r_0\sin\theta_L,\qquad
 T_s=\frac{2C}{B+\sqrt{B^2-4AC}}.
\]

二次方程式の短い方の解を採用します。この形は `AmpH=0` でも従来の `Ts=2*r0*sin(thetaL)/v` へ戻ります。判別式が負の候補は計算失敗として既存のペナルティ処理に渡します。立脚時間の新たな探索範囲は追加していません。

MultiStartの時間指定から作る開始点には逆関係 `v=2*r0*sin(thetaL)/Ts+AmpH*Ts/(2*pi*m)` を使います。時間については短い方の解が採用され、探索範囲への切り詰めによっても当初の指定時間から変わる場合があります。遊脚時間は引き続き `Tf=Ts*(1/Duty-1)` です。

与圧状態を `s=0` とし、`Fs=Fpre+ks*s` を使います。ストッパー反力を `lambda` とすると、

```text
R(k,s) - r = 0
Fs(s) + Fr*Rs(k,s) + tauR*beta_s(k,s) - lambda = 0
s >= 0, lambda >= 0, s*lambda = 0
```

を解きます。ばねエネルギーは `Fpre*s+ks*s^2/2`。胴体軌道は規定床反力から解析積分し、脚姿勢は全時刻をベクトル化したNewton法で求めます。関節速度は陰関数微分、関節トルクは仮想仕事から求めます。

`exact_polar_geometry.m` と `build_exact_geometry_cache.m` の exact は、幾何に一次近似を使わないことを表します。

今後はこの結果を初期解として多体モデルで再最適化する予定です。多体モデルとスタートモーション生成はまだ未実装です。既存の `_logs/elastic_approx`、`_logs/elastic_exact`、`_logs/EFLM` はそのまま保持します。
