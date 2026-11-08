# Stage1 鉛直床反力の自由軌道

`grfZN` は水平方向とは独立な制御点数（既定9）。`grfZBetaStart=[0,0.5,0]`、`grfZBetaEnd=[-0.5,0,0]` はそれぞれ無次元端点傾きの `[lower,upper,initial]`。`run_stage1_optimization.m` から変更する。

等間隔の n 点で端点値を0とし、内部の n−2 個の値 f_i と独立な端点傾き2個を最適化する。内部値は非負、和は1（等式制約）。追加変数は n 個、有効自由度は n−1。内部傾きは beta_i=(f_(i+1)−f_(i−1))/2、時間傾きは beta_i/dt、dt=Ts/(n−1)。鉛直方向に最大値点 j はない。

形状の区分3次Hermite係数から、値・時間微分・1回積分・2回積分を解析計算する。各区間の端点と、微分の実根のうち区間内部にある全ての点を評価し、区間最小値が非負という制約を課す。制御点間の負の床反力も禁止する。

形状面積と時間一次モーメントの係数は

    S = sum(f_i) + (beta_1-beta_n)/12
    C = sum((i-1)*f_i) + beta_1/30 + sum(beta_interior)/15 - (5*n-7)*beta_n/60

である。sum(f_i)=1 は探索途中に未達の場合もあるため、S の計算には実際の和を使う。S<=0 は計算失敗として扱う。

    Az = m*g*T/(dt*S)
    vz0 = g*(C*dt/S-T/2)
    Fz = Az*f(t)
    vz = vz0 + Az/m*integral(f) - g*t
    z = z0 + vz0*t + Az/m*doubleIntegral(f) - g*t^2/2

これにより探索途中も鉛直力積 mgT と高さ周期性が整合する。初期鉛直速度には vz0/(g*T)<=0 を課す（厳密な負値でなく、数値許容差を持つ非正制約）。離床時の高さは接地時と一致する必要がない。遊脚は離床状態からの弾道計算。

既存の摩擦・関節角・モータ・ばね・高さ・期間制約を引き続き評価する。広域開始点の内部鉛直値は和1に正規化して生成する。水平の (k,j) 走査と2段階MultiStartの構成は同じ。

最終形状は vertical_grf_control_points.csv、区間非負残差は vertical_grf_shape_constraints.csv に保存する。Az,S,C を含む復元情報は result.mat の final.body.verticalGRF に保存するが、report.txt には Az,S,C を記載しない。
