# stage1 の摩擦円錐制約

足先と床の静止摩擦係数は `common/parameters/robot_parameters.m` の
`p.frictionCoefficient = 0.4` とする。立脚期の各評価時刻で

\[
|F_x(t)|\le\mu F_z(t),\qquad F_z(t)\ge0
\]

を課す。静止摩擦とは足先が床に対して滑らない状態の摩擦であり、胴体が静止している必要はない。

SQPには絶対値を避けた、次の3つの無次元残差を渡す。すべて0以下が実行可能である。

\[
c_+=\frac{F_x-\mu F_z}{mg},\qquad
c_-=\frac{-F_x-\mu F_z}{mg},\qquad
c_z=\frac{-F_z}{mg}.
\]

`stage1_point_mass/sim_cycle_stage1_point_mass.m` の制約チャンネルに追加し、
広域・狭域探索では時系列の最大残差、最終修正では選択された時刻の残差を使う。
最終判定では密な軌道の全評価点を確認する。連続時間全体の厳密な保証ではなく、
既存の `constraintTolerance` に従う評価点での判定である。

`constraint_residuals.csv` に `frictionPositive`、`frictionNegative`、
`normalForce` を保存する。床反力をクリップしたり、滑り運動へ切り替えたりはしない。
過去のログと、保存用の initial_velocity / mean_velocity モデルは変更しない。
