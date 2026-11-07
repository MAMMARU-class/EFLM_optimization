> この文書は旧3モデル比較の記録です。現在のモデルは [stage1_point_mass](../../stage1_point_mass/README.md) です。旧実装は過去ログ内の source/ またはGit履歴を参照してください。

# `EFLM`：5一般座標モデル

## 一般座標と運動方程式

`urls` のEFLMと同じ構成で、一般座標を

\[
 q=[x,z,h,k,a]^\mathsf{T}
\]

とする。`x,z` は胴体の並進、`h,k,a` は股・膝・足首角である。足首角を独立座標とする点が2つのreduced modelと異なる。リンクの質量・慣性は無視する。従って、運動エネルギーは胴体の `m(ẋ²+ż²)/2`、重力と線形ばねのポテンシャルは `mgz+k_s s²/2` である。一般化した式の形は

\[
 M(q)\ddot q+\nabla U
 =\tau+J_{toe}^{\mathsf T}F_{GRF}+\tau_{extra},
\]

で、並進の2式は `m ẍ=F_x`、`m z̈=F_z-mg`。角度3式は質量ゼロのため加速度を含まない代数方程式となる。コードは共通の床反力を解析積分して胴体軌道を先に計算し、その軌道に対する接地姿勢を各時刻で解く。`urls` の運動方程式構造を維持しながら、最適化の反復中にDAEソルバーを走らせない実装である。詳細な元資料は [`EFLM_five_coordinate_model.md`](../EFLM_five_coordinate_model.md)、共通の最適化は [`00_common_pipeline.md`](00_common_pipeline.md) を参照。

実装の中心は [`sim_cycle_EFLM5.m`](../../EFLM/sim_cycle_EFLM5.m)、[`eflm5_geometry.m`](../../EFLM/kinematics/eflm5_geometry.m)、[`eflm5_balance.m`](../../EFLM/function/eflm5_balance.m)、[`eflm5_jacobian.m`](../../EFLM/function/eflm5_jacobian.m) である。

## 足先とばねの幾何

胴体を原点にした足先ベクトル `e(h,k,a)` は

\[
 e_x=L_1\cos h+L_2\cos(h+k)+(L_4-L_B)\cos(h+k-a),
\]

\[
 e_z=-L_1\sin h-L_2\sin(h+k)-(L_4-L_B)\sin(h+k-a).
\]

ばねリンクの長さ `ℓ` と伸び `s` は

\[
 \ell^2=L_2^2+L_B^2+L_D^2
 +2L_2(L_D\cos k-L_B\cos a)
 -2L_BL_D\cos(k-a),\qquad s=\ell-L_3.
\]

`e` の角度ヤコビアン `J_toe=[∂e/∂h,∂e/∂k,∂e/∂a]` と `∂s/∂q` を解析式で計算する。足先は地面に固定し、胴体から足先への目標ベクトル `Y=(-x,-z)` に `e=Y` を課す。ここでは `a` から `s` が決まる。approx/exactのように `s` を独立未知数として閉リンク角から `a` を消去する方法とは、計算変数の取り方が異なる。

## 与圧と角度3式の釣り合い

線形ばねポテンシャルの角度勾配は

\[
 \nabla_{h,k,a}U_{spring}=k_s s\,\nabla_{h,k,a}s.
\]

接地反力による一般化力は `J_toe^T[F_x,F_z]^T`。さらに `urls` に由来する与圧の**追加一般化トルク**を使う。`P(s)=F_pre(3u²−2u³)`、`u=clamp(s/δe,0,1)` として

\[
 \tau_{extra}=\begin{bmatrix}
 0\\
 \displaystyle P(s)\frac{L_B}{\ell}\left(L_2\sin k-L_D\sin(k-a)\right)\\
 \displaystyle-P(s)\frac{L_D}{\ell}\left(L_2\sin k-L_B\sin(k-a)\right)
 \end{bmatrix}.
\]

コードは

\[
 \tau_{required}=k_s s\nabla s-J_{toe}^{\mathsf T}F_{GRF}-\tau_{extra}
\]

を評価する。足首にはアクチュエータを置かないので `τ_required,a=0`。これと足先固定の2式を合わせた3式を、`h,k,a` の3未知数で解く。解いた後、`τ_h,τ_k` をモータの必要トルクとする。

**この追加トルクは、`P(s)` を保存的なばねポテンシャルに加えて `−P(s)∇s` とする式とは一致しない。** EFLMの診断出力には `spring_response(s,p,'smoothDiagnostic')` を使うが、その `springForce`,`springEnergy` は与圧を含む運動方程式のポテンシャル・力としては解釈しない。実際に式へ入る保存的な項は `k_s s` と `k_s s²/2`、与圧は上の追加トルクである。コードは `linearSpringEnergy`、`additionalSpringTorque`、`additionalSpringPower` を別々に保存する。これは既存の `urls` の式を優先するためのモデル差であり、最適化結果の差を読む際には幾何の表現差と区別する必要がある。

## 速度・仕事・残差

3本の残差を `E(h,k,a,t)=0` とし、複素ステップ法によって3×3の角度ヤコビアン `J_E=∂E/∂(h,k,a)` を作る。収束した式を時間微分して

\[
 J_E\begin{bmatrix}\dot h\\\dot k\\\dot a\end{bmatrix}
 =\begin{bmatrix}
 -\dot x\\-\dot z\\
 J_{toe,x,a}\dot F_x+J_{toe,z,a}\dot F_z
 \end{bmatrix}.
\]

この連立系から3角速度を求め、`ṡ=∇s·[ḣ,k̇,ȧ]` を計算する。仕事とCoTはほかの2モデルと同様、股・膝だけの絶対機械仕事から計算する。足首角は姿勢・ばね伸び・角速度の計算には使うが、足首モータの仕事は加えない。`generalizedCoordinates`、`generalizedVelocity`、`lagrangeResidual`、足先固定残差、追加トルクとそのパワーを結果に残す。足先固定と受動足首条件が成立することを、代数残差で確認できる。

## 速度向上のための実装

- 共通の胴体運動を解析積分し、角度3式だけを解く。記号的なDAE指数低減や立脚の時間積分を探索点ごとに実行しない。式の構造は5一般座標だが、反復で解くのは3角の代数系である。
- 伸び0の機構を4097点で作った逆補間表は、初期の膝角・足首角の推定にのみ使う。解の判定・トルクは現在の角度で厳密幾何を直接評価する。表は最適化変数の格子探索ではない。
- 足先とばねの1階幾何ヤコビアンを解析式にし、残差全体の3×3ヤコビアンは複素ステップで列ごとに求める。角度3式を時刻ごとに手作業で解く代わりに、全時刻の3×3系を `pagemldivide` でまとめて解く。
- 減衰Newton法で残差の悪化点だけ更新幅を縮め、閉リンクの同じ組立分枝にいることを確認する。不正な姿勢や収束失敗はペナルティ候補として扱う。
- 可視化用の `s=0` 参照座標 `l,q` や動画は、最適化の代数反復を終えた後に計算する。飛行中は胴体の弾道運動だけを計算・描画する。

このモデルは反復で3変数と閉リンク分枝確認を扱うため、1回の姿勢評価は2変数モデルより重くなり得る。最適化の総処理時間は候補の収束性や反復数にも依存する。
