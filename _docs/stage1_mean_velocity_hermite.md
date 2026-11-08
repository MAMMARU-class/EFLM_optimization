# 平均速度・Hermiteモーメントに基づく立脚時間

形状生成は `horizontal_grf_shape.m`、時間決定は `prescribed_motion_stage1.m`。
共通端点の無次元傾きは `overrides.grfBetaEndpoint=[-0.4,0,0]`。
内部は beta_i=(f_(i+1)-f_(i-1))/2、j点は0。物理時間の傾きは b_i=beta_i/dt。

\[
S=\sum_{i=2}^{n-1}f_i+\frac{\beta_1-\beta_n}{12},\qquad
C_H=\sum_{i=2}^{n-1}(i-1)f_i+\frac{\beta_1}{30}
+\frac1{15}\sum_{i=2}^{n-1}\beta_i-\frac{5n-7}{60}\beta_n.
\]

力積は dt S、時間一次モーメントは dt^2 C_H。共通端点なので S の端点項は0。
探索途中では S が0とは限らないため、水平移動量の一般式を使う。

\[
L(T)=v_{x0}T-\gamma T^2,\qquad
\gamma=\frac{A[C_H-(n-1)S]}{m(n-1)^2},\quad L_0=2r_0\sin\theta_L.
\]

\[
D=v_{x0}^2-4\gamma L_0,\qquad
T_0=\frac{2L_0}{v_{x0}+\sqrt D},\qquad T_s=\mathrm{stanceScale}\,T_0.
\]

v_x0>0、L_0>0、D>=0 の短い方の正根を使用する。gamma=0 でも同じ式で L_0/v_x0 に帰着する。
D<0 は `EFLM:StanceTime` として既存の失敗ペナルティへ渡す。判別式の追加マージンは設けない。
倍率1なら力積未達の候補でも立脚水平移動量が L_0 と一致する。倍率適用後は一般に一致しない。

ユーザー指定により、軌道中の水平速度下限制約は設けない。初期速度の正値条件と既存の周期・距離等の制約は維持する。

形状と係数は候補ごとに一度生成し、基準時間計算と軌道積分で共用する。
平方根は1回で時間を決定し、積分累積も配列演算する。脚の反復計算を追加することはない。
`result.mat` には S、CH、beta、gamma、判別式、基準時間を保存する。
制御点CSVの slope は物理単位 s^-1、`horizontal_grf_beta.csv` は無次元 beta。
旧ログのソース・結果は変更しない。旧 grfSlopeEndpoint を無次元値として読み替えず、設定名も変更した。
