# stage1_basic：基本パラメータのグリッドサーチ

基本パラメータを格子値に固定し、各格子点・各零交差点 k で広域 MultiStart を実行する。
時間・床反力パラメータは連続最適化する。狭域探索、密な軌道での修正、昇格・自動除外は実施しない。

## 実行

MATLAB R2026b でプロジェクトを開き、プロジェクト直下の `run_stage1_basic.m` の設定を変更して実行する。

```matlab
[summary, folder] = run_stage1_basic();
```

引数を渡す場合は、実行窓口の設定のうち指定した項目だけを上書きする。

```matlab
options.vx0 = [2.2 0.4 5];
options.thetaL = [deg2rad(50) deg2rad(10) 5];
options.r0 = [0.30 0 1];
options.restrictGrfK = true;
options.grfKSelection = [7 8 9];
options.startsPerK = 20;
options.topResults = 3;
[summary, folder] = run_stage1_basic(options);
```

Optimization Toolbox / Global Optimization Toolbox、並列実行時は Parallel Computing Toolbox が必要。
並列化は各ケース内部の MultiStart に適用し、ケースを順番に実行する。並列プールと幾何初期値用テーブルは一度だけ準備する。

## 設定

| 項目 | 指定形式・既定値 | 意味 |
|---|---|---|
| vx0 | [2.2, 0.4, 5] | 初期水平速度 [m/s] の中心値・片側への探索幅・奇数の格子点数 |
| thetaL | [deg2rad(50), deg2rad(10), 5] | 初期脚角 [rad] の中心値・片側への探索幅・奇数の格子点数 |
| r0 | [0.30, 0.05, 5] | 初期脚長 [m] の中心値・片側への探索幅・奇数の格子点数 |
| stanceScale | [0.5, 2, 1] | 時間倍率の下限・上限・初期値 |
| duty | [0.3, 0.6, 0.5] | Duty比の下限・上限・初期値 |
| Ix | [0.1, 4, 1.13] | 水平絶対力積 [N s] の下限・上限・初期値 |
| grfXN / grfZN | 15 / 9 | 水平・鉛直の制御点数 |
| grfXBetaStart / grfXBetaEnd | [-0.5, 0, 0] | 水平端点傾きの下限・上限・初期値 |
| grfZBetaStart / grfZBetaEnd | [0, 0.5, 0] / [-0.5, 0, 0] | 鉛直端点傾きの下限・上限・初期値 |
| restrictGrfK | false | trueならgrfKSelectionだけを走査 |
| grfKSelection | [7 8 9 10] | 走査するk。制限なしの場合は3:n-2 |
| startsPerK | 2000 | **基本格子点・kの組み合わせ1個あたり**の開始点数 |
| nodes | 121 | 最適化・結果判定・再構成に使う時間点数 |
| topResults | 3 | 全ケースから実行可能解をコスト順に選び、動画・図を生成する件数 |
| parallelWorkers | 20 | プロセスワーカー数 |
| targetSpeed / targetCoT | 4 / 0.3 | 目的関数の目標値 |
| w1 / w2 | 5 / 1 | 速度誤差・CoT誤差の重み |

基本パラメータは `[中心値 探索幅 格子点数]` とし、中心値±探索幅の両端を含む等間隔の格子に置く。格子点数は奇数のみ指定でき、中心値が必ず格子点に含まれる。`[value 0 1]` で1値に固定する。
thetaLは中心値と探索幅のみ`deg2rad`で変換し、点数は変換しない。
基本3変数はソルバーの変数から除外するため、格子値から変化しない。
初期値や探索箱を狭める方式ではない。

実行窓口の基本パラメータ設定では125格子点。`k`の数と`startsPerK`を掛けた回数のローカル最適化を実行する。
`startsPerK` と格子点数を実行窓口で調整できる。

## 判定と保存

各開始点の最終候補を比較し、実行可能な候補同士ではコスト最小を採用する。
実行可能解がない場合は、計算できた候補の中で最大制約違反が最小のものを残す。
各開始点の候補は選別にだけ使い、全開始点の履歴は保存しない。

| status | 意味 |
|---|---|
| feasible_at_evaluation_nodes | 設定した時間点および解析的な床反力形状条件で、最大違反が許容値以下 |
| no_feasible_solution_found | 計算可能な候補はあるが、今回の探索では制約を満たせなかった |
| no_evaluable_solution | 最終候補の軌道計算が成立しなかった。原因文字列を保存 |

失敗は「この計算予算で解を発見できなかった」という意味。数学的な実行不可能性の証明ではない。
今回の結果による自動的な候補除外・昇格は行わない。
121点での実行可能性は時間連続の全機構制約を保証するものではない。

出力先：`_logs/stage1_basic/日時/`

| ファイル | 内容 |
|---|---|
| settings.mat | 共通のモデルパラメータ、探索設定、格子値、変数順序、バージョン情報 |
| cases/000001.mat など | 各組み合わせの最終変数、k、基本値、評価値、判定、終了情報、集計値のみ |
| summary.csv | 全組み合わせの基本値・k・コスト・制約違反・平均速度・CoT・時間など |
| report.txt | 開始日時・実行時間、設定、完了件数、集計、上位ケースID |
| report_no1.txt など | 出力対象の各順位について、最終変数・床反力制御点・運動指標・制約違反を記載 |
| grid_search_result/ | 基本パラメータとコスト・速度・CoTの関係を示す15枚のカラーマップ。実行可能解がない格子は黒 |
| source/ | 再構成用のソースを実行全体で一度だけ保存 |
| plots/no1_parameters.png など | 上位の立脚期グラフ |
| animation/no1_parameters.mp4 など | 上位の動画 |

順位は**全格子点・全kの実行可能ケース**についてのコスト昇順。同一基本格子点の異なるkが複数順位に入ることもある。
実行可能ケースが指定件数より少なければ、その件数だけ出力する。0件なら動画・図は作らない。
`topResults=0` で図・動画の生成を省略、`video=false` なら図のみ。
FIG、プレビュー画像、render_settings、各ケースの時系列CSV、MultiStartオブジェクトや反復履歴は保存しない。

ケースが完了するたびにMATと一覧を保存する。中断時にも完了済みケースは残る。
中断されたケースの途中経過と、自動再開機能は今回の対象に含まない。

## 後から軌道を生成する

```matlab
result = reconstruct_basic_case(folder, caseId);
% result.final.body / angles / tau / power などで自由に解析
plot_basic_result(result, fullfile(folder, 'additional_plot.png'));
```

長期的な再現には保存先の `source/common` と `source/stage1_basic` をパスに追加し、保存した実装を用いる。
すべての探索変数は `record.variables`、並び順は `settings.mat` 内の `cfg.variableNames` に保存する。
最終変数は次回の最適化の初期候補にも使える。

既存の単質点モデルの物理・幾何式を維持し、stage1_basic内では関数名を分けて旧モデルとのパス衝突を防いでいる。
