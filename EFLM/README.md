# 5一般座標のEFLM

urls の `motion_equation.m` と `parameter.m` に従い、一般座標を
`[x,z,theta_h,theta_k,theta_a]` とするモデルです。脚の慣性は含めません。
`elastic_exact` の呼び出しによる代用ではなく、足首角も独立未知数として解きます。

## 実行

```matlab
[result, folder] = run_elastic_optimization('EFLM');
% 設定を明示するとき（共通デフォルトに上書き）
options = struct('starts',80,'localStarts',20,'w1',2,'w2',1);
[result, folder] = run_elastic_optimization('EFLM',options);
```

ルート実行ファイルは変更していません。共通の振り分け先にEFLMを追加しています。
引数なしは従来どおりapproxです。実行中の別モデルを切り替える操作ではありません。
結果は `_logs/EFLM/日時` に保存します。

| 項目 | 内容 |
|---|---|
| 探索変数 | 水平力振幅、入射角、速度 [m/s]、duty、初期脚長 |
| 探索範囲・重み | ルート実行ファイルの設定、またはoptionsで上書き |
| 最適化 | 共通の広域MultiStart→狭域MultiStart→密な時間点で再検証・修正 |
| ロボット設定 | `common/parameters/robot_parameters.m` |
| 制約 | 共通のモータ・関節角・伸び・高さ・立脚/遊脚時間制約 |
| 保存 | MAT、CSV、履歴、report.txt、グラフ、EFLM形状の動画 |

## ファイル構成

| ファイル/フォルダ | 役割 |
|---|---|
| `main_EFLM.m` | 直接実行用の入口 |
| `motion_equation_EFLM5.m` | urlsと同じL=T-Uからの記号的導出（確認用、最適化では呼ばない） |
| `sim_cycle_EFLM5.m` | 胴体運動、関節姿勢、速度・トルク、評価・制約 |
| `kinematics/eflm5_geometry.m` | 独立した3関節角からの幾何とヤコビアン |
| `function/eflm5_balance.m` | ラグランジュ式の角度成分・接地残差 |
| `function/eflm5_jacobian.m` | 連立方程式のヤコビアン |
| `parameters/build_eflm5_cache.m` | 初期推定専用の幾何テーブル |
| `validate_EFLM5.m` | 独立な差分検証と速度測定 |

式とurlsからの設定変更は `_docs/EFLM_five_coordinate_model.md` を参照してください。

## 検証

R2026bで数値式・関節速度・仕事率を検証し、3+2開始点の小規模最適化で
実行可能解、結果保存、動画生成を確認しています。大規模探索は実行していません。
121点/1201点の評価は既定入力で約17 ms/39 msでした（環境・入力に依存）。
`motion_equation_EFLM5` は確認用でSymbolic Math Toolboxが必要です。
現在のR2026bには未導入のため記号導出関数の実行検証は未実施です。
通常の最適化と `validate_EFLM5` はSymbolic Math Toolboxを必要としません。
