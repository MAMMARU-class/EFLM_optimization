# elastic_exact

EFLMのばね長 `L3 + ds` を含む厳密な閉リンク幾何で、脚姿勢・ばね伸び・関節トルクを求めます。脚質量ゼロ、規定床反力による胴体の解析積分、ばね構成則、目的関数、制約、MultiStartの広域→狭域→高密度修正はapproxと共通です。

```matlab
[result, folder] = run_elastic_optimization('exact');
```

共通実行関数 `run_elastic_optimization.m` の設定を編集するか、引数で上書きします。

```matlab
options.lower = [0, deg2rad(1), .1, .1, .25];
options.upper = [30, deg2rad(60), 3, .8, .34];
options.initial = [.5, deg2rad(8), .4, .5, .29];
options.w1 = 2;
options.w2 = 1;
[result, folder] = run_elastic_optimization('exact', options);
```

| 項目 | 内容 |
|---|---|
| 変数順 | `[AmpH, thetaL, v, Duty, r0]`。角度rad、脚長m、速度m/s |
| モデル選択 | `'approx'` または `'exact'` |
| 出力 | `_logs/elastic_exact/日時/`。MAT、CSV、レポート、動画、ソースを保存 |
| 再描画 | `recreate_outputs(fullfile(folder,'result.mat'))` |
| 検証 | `validate_elastic_exact` |

## 計算方法

`k` は膝関節角、`h` は股関節角、`s=ds` はばね伸びです。股関節角ゼロでの厳密な足先位置から、脚長 `R(k,s)` と脚角 `beta(k,s)` を計算します。回転対称性により `h=theta-beta` を消去でき、反復未知数は `(k,s)` の2つです。

```
R(k,s) - r = 0
Fs(s) + Fr*Rs(k,s) + tauR*beta_s(k,s) = 0
```

これは `_docs/equation_of_motion.md` の式(19a)を2変数化したものです。伸びゼロ近傍への一次展開は使いません。幾何の一次導関数は解析式、Newton行列に必要な二次導関数は複素ステップ微分で評価します。数値微分誤差・収束許容誤差はありますが、幾何モデルの線形化は行いません。

全時刻をベクトル化した減速Newton法で解き、陰関数微分から関節角速度とばね伸び速度を計算します。関節トルクは厳密な幾何ヤコビアンによる仮想仕事から求めます。参照テーブルはNewton初期値だけに使い、最終解と力変換は厳密式で評価します。

機構が成立しない反復点はラインサーチで戻し、解けない候補は共通最適化器のペナルティで扱います。ばね・関節の物理制約はapproxと同じです。

検証では足先一致、Cartesianヤコビアンから独立に計算した関節トルクとばね釣り合い、`Pact = G・Ydot + Fs*sd`、時間差分と角速度の一致を確認します。
