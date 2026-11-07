# 立脚時間倍率を含む stage1 最適化

最適化変数は `[AmpH, thetaL, v, Duty, r0, stanceScale]` の6つ。

\[
T_{s,0}=\frac{2r_0\sin\theta_L}{v_{x0}},\qquad
T_s=\gamma T_{s,0},\qquad \gamma=\mathrm{stanceScale}>0.
\]

倍率1で従来の初期速度ベースに一致する。現在の実行プリセットでは、倍率の探索範囲は0.4〜2.5、初期値は1。`run_stage1_optimization.m` の `overrides.stanceScale=[lower, upper, initial]` で変更する。

遊脚時間は `Tf=Ts*(1/Duty-1)`。床反力の周期、鉛直力積に基づく初期鉛直速度、胴体軌道、関節運動は倍率を適用した時間で再計算する。立脚・遊脚とも0.2秒以上の制約は維持する。倍率を変えると実際の立脚水平移動量は一般に `2*r0*sin(thetaL)` と一致しない。

\[
\Delta x=v_{x0}T_s-\frac{\mathrm{AmpH}\,T_s^2}{2\pi m}.
\]

目的関数は目標速度4 m/s、目標CoT 0.3に対する相対誤差の重み付き二乗和。目標値と重みは実行ファイルから変更できる。

実行は `run_stage1_optimization()` または設定構造体を渡す。各最適化変数は `overrides.AmpH`、`thetaL`、`v`、`Duty`、`r0`、`stanceScale` に `[lower, upper, initial]` を設定する。必要な変数だけ指定でき、省略した変数には `optimization_settings.m` の既定値を使う。従来の `lower`、`upper`、`initial` 配列は廃止した。モデル選択の文字列引数も廃止し、`stage1_point_mass` 専用とする。出力は `_logs/stage1_point_mass/<日時>/`。`summary.csv`、`result.mat`、`report.txt` に倍率を保存し、`result.final.body.TsBase` に基準時間を保存する。

運動生成は `stage1_point_mass/prescribed_motion_stage1.m`。`archive/` 内の initial_velocity / mean_velocity は変更しない。過去の実行は各ログに保存されたソースと設定から再現する。

なお、平均速度ベースの従来式も閉形式の平方根計算であり、反復して平均速度を求める処理ではない。今回の変更は立脚時間の自由度を追加するためのもの。
