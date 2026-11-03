# 厳密EFLMモデルと共通最適化入口

`equation_of_motion.md` の式(19a)、(20)を実装する。胴体軌道、ばね構成則、目的関数、制約、広域・狭域MultiStartと最終高密度修正はapproxと共通である。脚質量はゼロ。`rigid` の実装は本変更の対象外。

## 座標と未知数の削減

`h=q_h` は股関節角、`k=q_k` は膝関節角、`s=ds` はばね伸び。股関節角ゼロでの厳密足先ベクトルを `Y0(k,s)` とし、`R=norm(Y0)`、`beta=atan2(-Y0_z,Y0_x)` とする。脚角 `theta=h+beta` なので、股関節角は `h=theta-beta(k,s)` で厳密に消去できる。

各時刻で解く2式は

$$
R(k,s)-r=0,\qquad F_s(s)+F_rR_s(k,s)+\tau_r\beta_s(k,s)=0.
$$

`R_s`、`beta_s` は現在の伸びで評価する。一次近似の `f(l),g(l)` は使用しない。Newton初期値用の自然長姿勢テーブルは使うが、収束した解の幾何・トルクには補間を使わない。機構分岐は旧EFLMと同じ、膝角の数値探索分岐はapproxと同じ12～148度とし、物理的な関節制限は別途制約で評価する。

## 速度とトルク

2式のヤコビアンを `M` とすると

$$
M\begin{bmatrix}\dot k\\\dot s\end{bmatrix}
=\begin{bmatrix}\dot r\\-\dot F_r R_s-\dot\tau_r\beta_s\end{bmatrix},
\qquad \dot h=\dot\theta-\beta_k\dot k-\beta_s\dot s.
$$

股・膝トルクは仮想仕事から

$$
\tau_h=\tau_r,\qquad \tau_k=R_kF_r+\beta_k\tau_r.
$$

これらは弾性エネルギー `Us(s)` を含む、脚質量ゼロのラグランジュ方程式の代数部分と整合する。`Fs=dUs/ds` であり、ばね伸びが代数変数でも `dUs/dt=Fs*sd` は仕事収支に含まれる。検証では `Pact=G・Ydot+Fs*sd` を確認する。ここで `G` は股から足先へのベクトルに共役な力で、胴体へ作用する床反力の負号。

## 共通実行

```matlab
[result, folder] = run_elastic_optimization('approx');
[result, folder] = run_elastic_optimization('exact');
% 個別の設定を渡す場合
[result, folder] = run_elastic_optimization('exact', options);
```

引数省略時はapprox。設定構造体だけを渡す従来形式もapproxとして扱う。明示した設定構造体は `optimization_settings` の既定値を上書きし、引数なし用の手動プリセットとは区別する。既存の手動プリセット（脚長範囲0.22～0.32 mを含む）は新しいルート実行関数へ移設した。

共通処理は `common/optimization/main_elastic.m`、モデル選択は `optimize_two_stage.m` の評価関数ハンドルだけで行う。保存先は `_logs/elastic_approx/日時` または `_logs/elastic_exact/日時`。レポートと動画にモデル名を表示し、ソース、パラメータ、幾何テーブル、全軌道を保存する。厳密な共通順運動学 `eflm_geometry.m` は `common/function` に移設した。

## 検証結果（2026-11-03）

R2026bで `validate_elastic_approx`、`validate_elastic_exact` が成功。exactは12001点の軌道で、厳密Cartesian幾何とその複素ステップ微分から独立に足先・トルク・ばね釣り合い・速度を照合した。

| 検査 | 最大誤差 |
|---|---:|
| 足先位置 | 約2.0e-16 m |
| 関節トルク | 約1.2e-14 Nm |
| ばね釣り合い | 約3.8e-13 N |
| 仕事率収支 | 約3.5e-14 W |
| 関節角速度の時間差分比較（内部区間） | 約4.0e-7 rad/s |

ウォームアップ後、同一入力を10回評価した平均値：121点はexact約5.2 ms、approx約5.1 ms。1201点はexact約11.8 ms、approx約11.4 ms。入力や収束回数に依存し、全探索の速度を保証する値ではない。

広域3・狭域2開始点で両モデルの共通入口を確認し、共に `valid=1`。exactは動画生成まで成功。確認ログ：`_logs/elastic_exact/20261103_212738_678`、`_logs/elastic_approx/20261103_212754_535`。これは動作確認用の小規模探索であり、通常設定による最適解の探索ではない。
