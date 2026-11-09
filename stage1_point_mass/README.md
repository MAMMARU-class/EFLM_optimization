# stage1_point_mass — 単質点モデルの最適化

水平床反力を絶対力積 Ix と正規化した形状で決める。離散探索は零交差点 k のみ。広域MultiStart → 最良kの近傍で狭域MultiStart → 密な軌道で最終確認・修正、の順で実行する。

## 実行

```matlab
[result, folder] = run_stage1_optimization();
```

プロジェクト直下の同名ファイルに既定設定がある。引数を渡す場合、省略項目は common/parameters/optimization_settings.m の値を使う。

```matlab
overrides.vx0 = [1, 3.5, 2.2];
overrides.Ix = [0.1, 4, 1.13];
overrides.grfXN = 15;
overrides.grfZN = 9;
overrides.restrictGrfK = true;
overrides.grfKSelection = [7 8 9 10];
overrides.startsPerK = 2000;
[result, folder] = run_stage1_optimization(overrides);
```

## パラメータと探索幅

連続量の設定は [lower, upper, initial]。制御点数・開始点数・候補kは単一値または一覧。

| 分類 | 設定名 | 意味・単位 |
|---|---|---|
| 基本 | vx0 | 接地時の水平速度 [m/s] |
| 基本 | thetaL | 接地時の脚角度 [rad]（度からは deg2rad を使用） |
| 基本 | r0 | 初期脚長さ [m] |
| 時間 | stanceScale | 基準立脚時間に掛ける倍率 alpha |
| 時間 | duty | 立脚時間 / 1周期 |
| 水平床反力 | Ix | 水平床反力の絶対力積 [N s]。振幅Axは従属量 |
| 水平床反力 | grfXN | 等間隔制御点数、既定15 |
| 水平床反力 | grfXBetaStart / grfXBetaEnd | 独立な無次元端点傾き、既定は双方 [-0.5,0,0] |
| 鉛直床反力 | grfZN | 水平とは独立な制御点数、既定9 |
| 鉛直床反力 | grfZBetaStart / grfZBetaEnd | 既定 [0,0.5,0] / [-0.5,0,0] |
| 探索 | restrictGrfK / grfKSelection | trueなら指定k、falseなら3:grfXN-2 |
| 探索 | startsPerK | 各kの広域開始点数。総数は候補k数との積 |
| 探索 | localStarts | 最良kを固定した狭域開始点数 |
| 目的関数 | targetSpeed / targetCoT | 目標平均速度・CoT |
| 目的関数 | w1 / w2 | 速度誤差・CoT誤差の重み |

現行ルート既定値は全11個のk × 2,000開始点 = 22,000開始点、狭域1,000開始点、20ワーカー。
AmpH、v、Duty、grfN、grfBetaEndpoint、grfPairSelection、総数指定のstartsは旧インターフェースであり、新規実行では受け付けない。

## 計算と保存

内部の変数順は「基本3個 → 時間2個 → 水平grfXN個 → 鉛直grfZN個」。水平値は n−3 個の非負の大きさを最適化し、kより前で負、後で正にする。内部傾きは全点で中央差分。水平の大きさの和1・正味力積0・鉛直の値の和1を等式制約にする。

Ix、形状、初期状態から平均立脚速度を解析計算し、基準時間を L/meanV とする。最終時間を alpha 倍した後に振幅を再計算する。位置・速度もHermite多項式から解析積分する。区間極値で符号を判定し、摩擦・モータ・関節角・ばね・高さ・期間制約も評価する。

結果は _logs/stage1_point_mass/日時/ に保存。reportのパラメータは上記4分類で表示する。CSV・result.matには制御点、傾き、振幅、力積、軌道を保存し、再描画・復元できる。既存ログは変更しない。

詳細は [水平絶対力積方式](../_docs/stage1_horizontal_impulse_grf.md) と [鉛直自由軌道](../_docs/stage1_vertical_hermite_grf.md) を参照。
