# 運動生成モデル

現在の実装は **stage1_point_mass（単質点モデル）** です。旧 elastic_exact の計算方法を引き継いでいます。実行方法とモデルの概要は [stage1_point_mass/README.md](../../stage1_point_mass/README.md) を参照してください。

| 項目 | 現在の名称 |
|---|---|
| モデルフォルダ | `stage1_point_mass/` |
| 実行入口 | `run_stage1_optimization.m` |
| シミュレーション | `sim_cycle_stage1_point_mass.m` |
| 新規ログ | `_logs/stage1_point_mass/<日時>/` |

今後は単質点の最適化結果を初期解として、多体モデルで再最適化する予定です。現時点で第2段階は未実装です。

以下は旧3モデル比較の記録です。approx と5一般座標EFLMの実装は本ブランチから削除しました。過去の実行条件・コードは既存ログの `settings.mat` と `source/` に保存されています。

| 過去の文書 | 内容 |
|---|---|
| [00_common_pipeline.md](00_common_pipeline.md) | 共通最適化・解析積分・高速化 |
| [01_elastic_approx.md](01_elastic_approx.md) | 一次近似モデル |
| [02_elastic_exact.md](02_elastic_exact.md) | 単質点モデルの元となる厳密幾何 |
| [03_EFLM_five_coordinates.md](03_EFLM_five_coordinates.md) | 旧5一般座標モデル |
| [preload_handling.md](preload_handling.md) | 旧3モデルの与圧則の比較 |
