# elastic_approx：実行ガイド

EFLMの一次近似弾性モデルを、規定床反力のもとで連続最適化するプログラムです。広域MultiStart、狭域MultiStart、高密度軌道での制約再確認の順に実行します。

## 必要な環境

| 項目 | 要件 |
|---|---|
| MATLAB | R2026b |
| Toolbox | Optimization Toolbox、Global Optimization Toolbox |
| 単位 | SI単位系。速度は m/s、角度はプログラム内部で rad |
| MATLAB Project | このプロジェクトを開いた状態では、全ファイルを参照可能 |

## 最も簡単な実行方法

プロジェクト直下の実行関数を使います。

```matlab
[result, folder] = run_elastic_optimization('approx');
```

`folder` は今回の保存先です。保存先は常に次の形式になります。

同じ最適化手法で厳密モデルを使う場合は `run_elastic_optimization('exact')` とします。共通の実行設定は `run_elastic_optimization.m` で編集します。旧名 `run_elastic_approx_optimization` は互換呼び出しとして残り、`'exact'` の指定にも対応します。

```text
_logs/elastic_approx/YYYYMMDD_HHMMSS_SSS/
```

どの作業フォルダから呼び出した場合も、プロジェクト直下の `_logs` に保存されます。

## 探索範囲を変更して実行する

探索変数の並びはすべて共通です。

| 列 | 変数 | 意味 | 単位 |
|---:|---|---|---|
| 1 | `AmpH` | 水平床反力振幅 | N |
| 2 | `thetaL` | 接地脚角度 | rad |
| 3 | `v` | 初期水平速度 | m/s |
| 4 | `Duty` | 立脚時間比 | - |
| 5 | `r0` | 接地開始時の脚長 | m |

既定の広域探索範囲は次のとおりです。

| 変数 | 下限 | 上限 | 既定初期値 |
|---|---:|---:|---:|
| `AmpH` | 0 N | 50 N | 0.5 N |
| `thetaL` | 0.5 deg | 35 deg | 8 deg |
| `v` | 0.03 m/s | 2.0 m/s | `2*p.leg*sin(8 deg)/0.2`（約0.404 m/s） |
| `Duty` | 0.10 | 0.90 | 0.50 |
| `r0` | 0.25 m | 0.34 m | `robot_parameters().leg`（現在0.290 m） |

任意の探索範囲へ上書きする例です。角度だけは `deg2rad` でradへ変換します。

```matlab
options = struct;
options.lower = [0,  deg2rad(1),  0.10, 0.20, 0.25];
options.upper = [30, deg2rad(25), 1.50, 0.80, 0.34];
options.initial = [0.5, deg2rad(8), 0.473, 0.50, 0.29];
options.w1 = 10;  % 平均速度の重み
options.w2 = 1;   % CoTの重み

[result, folder] = run_elastic_optimization('approx', options);
```

`initial` は必ず `lower` と `upper` の間に置きます。指定していない項目は既定値が使われます。

`r0` により初期位置と立脚時間を計算します：`x0=-r0*sin(thetaL)`、`z0=r0*cos(thetaL)`、`Ts=2*r0*sin(thetaL)/v`。リンク寸法は固定です。正規化には `p.lengthScale=0.295 m`（指定範囲0.25～0.34 mの中央値）を固定して使います。`p.leg` は初期候補の脚長を与えるだけで、探索中の脚長は5番目の変数で決まります。

## MultiStartの設定を変更する

| 設定名 | 既定値 | 意味 |
|---|---:|---|
| `starts` | 16 | 広域MultiStartの開始点数 |
| `localStarts` | 5 | 狭域MultiStartの開始点数 |
| `localRadius` | 0.15 | 広域幅に対する狭域探索の片側幅 |
| `algorithm` | `sqp-legacy` | MultiStart内で使うfminconアルゴリズム |
| `w1` | 10 | 平均速度 [m/s] に掛ける重み |
| `w2` | 1 | CoTに掛ける重み |
| `seed` | 20261103 | 乱数シード |
| `stageSeconds` | 43200 s（12時間） | 広域・狭域それぞれの時間上限 |
| `maxIterations` | 120 | 1回のSQPの最大反復数 |
| `maxEvaluations` | 1400 | 1回のSQPの最大評価回数 |
| `nodes` | 121 | 最適化中の立脚評価点数 |
| `finalNodes` | 1201 | 最終制約確認の立脚評価点数 |

探索範囲と開始点数を同時に変える例です。

```matlab
options.lower = [0, deg2rad(2), 0.10, 0.25, 0.25];
options.upper = [20, deg2rad(20), 1.20, 0.75, 0.34];
options.initial = [0.5, deg2rad(8), 0.473, 0.50, 0.29];
options.starts = 32;
options.localStarts = 10;
options.localRadius = 0.10;
options.w1 = 5;
options.w2 = 1;

[result, folder] = run_elastic_optimization('approx', options);
```

最大化する評価値は `w1 * 平均速度 - w2 * CoT` です。`fmincon`には符号を反転した `w2 * CoT - w1 * 平均速度` を渡します。重みを変えると最適化の優先度が変わり、別の解になることがあります。

すべての既定値は [optimization_settings.m](../common/parameters/optimization_settings.m) にあります。通常はこのファイルを書き換えず、プロジェクト直下の実行関数へ `options` を渡します。

## 固定された主な制約

| 制約 | 条件 |
|---|---|
| 立脚時間 | 0.2 s以上 |
| 遊脚時間 | 0.2 s以上 |
| 股関節角 | 55–155 deg |
| 膝関節角 | 16–144 deg |
| ばね伸び | 0–15 mm |
| モータ | 旧モデルの関節軸TN特性。トルクはクリップしない |

## 最適化の経路

| 段階 | 処理 |
|---:|---|
| 1 | 指定された広域範囲をMultiStartで探索 |
| 2 | 段階1の解を中心に狭域MultiStartを実行 |
| 3 | 1201点の軌道で制約を再確認し、必要な時刻を追加してlegacy SQPで修正 |
| 4 | 最終軌道、グラフ、動画、実行ソース、レポートを保存 |

MultiStartは有限範囲内で複数の局所最適化を実行する方法です。数学的な大域最適性を保証するものではありません。最適解が探索上限・下限に近い場合は、`lower` と `upper` を広げて再実行します。

## ログと `report.txt`

各実行フォルダには `report.txt` が作られます。実行元にかかわらず、次の内容を確認できます。

| 記録内容 | 説明 |
|---|---|
| 実行経路 | 呼び出し元、プロジェクトルート、広域→狭域→最終修正の経路 |
| 探索条件 | 上下限、初期値、開始点数、乱数シード、時刻点数 |
| 段階別統計 | 成功開始点数、反復数、関数評価回数、所要時間 |
| 最終値 | 最適化変数、速度、CoT、仕事、期間、最大制約違反 |
| 失敗情報 | 例外名、メッセージ、発生関数と行番号 |

主な保存ファイルは次のとおりです。

| ファイル／フォルダ | 内容 |
|---|---|
| `report.txt` | 人が読むための実行レポート |
| `result.mat` | 設定、開始点、全軌道、制約、仕事、最適化結果 |
| `summary.csv` | 最終結果の要約 |
| `optimization_history.csv` | 評価ごとの変数・目的関数・制約違反 |
| `trajectory.csv` | 最終立脚軌道の詳細 |
| `constraint_residuals.csv` | 時刻ごとの制約残差 |
| `plots/` | PNGとMATLAB Figure |
| `animation/motion.mp4` | EFLM形状の動画。遊脚中は脚を非表示 |
| `source/` | 実行時に使ったソースのスナップショット |

動画やグラフは、保存済みの `result.mat` から再生成できます。

```matlab
recreate_outputs(fullfile(folder, 'result.mat'));
```

## 実装上の要点

| 項目 | 内容 |
|---|---|
| 胴体運動 | 規定床反力を解析積分 |
| 弾性計算 | 幾何式とばね釣り合いを減速Newton法で同時に解く |
| ばね | `ds=0` を与圧状態とし、反力 `3.4 N + ks*ds`。ゼロ伸びの片側ストッパーを含む。減衰なし |
| CoT | 立脚中の股・膝の絶対機械仕事のみ |
| 遊脚 | 胴体の弾道計算のみ |
| 動画 | 表示上は左進行。遊脚中は脚を描画しない |

詳細な式と符号は [_docs/equation_of_motion.md](../_docs/equation_of_motion.md)、実装経緯は [_docs/implementation_run_notes.md](../_docs/implementation_run_notes.md) を参照してください。
