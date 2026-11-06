# 3モデルにおける与圧の扱い

この文書は、現行実装の `elastic_approx`、`elastic_exact`、`EFLM` で与圧がどの式に入るかを比較する。いずれも同じ設定値 `F_pre = 3.4 N`、立ち上がり幅 `δ_e = 5×10⁻⁵ m = 0.05 mm`、線形ばね定数 `k_s = 17430 N/m` を使う。値は [`robot_parameters.m`](../../common/parameters/robot_parameters.m) にある。

## 共通する立ち上がり関数

`s = ds` をばね伸びとし、

\[
u=\operatorname{clamp}(s/\delta_e,0,1),\qquad
\rho(s)=3u^2-2u^3,\qquad P(s)=F_{pre}\rho(s)
\]

とする。`s = 0` で `P = 0`、`s ≥ δ_e` で `P = F_pre` となり、その間は連続かつ端点で傾きが0の曲線で立ち上がる。`k_s s` は3モデルとも伸びに対する線形項として使う。ただし、**`P(s)` を機構へ作用させる方法が異なる**。

| モデル | 与圧の作用 | 解く条件での位置 | 与圧を含む保存エネルギー |
|---|---|---|---|
| [`elastic_approx`](01_elastic_approx.md) | ばね軸方向の反力 `F_s = k_s s + P(s)` | 一次近似した伸び方向の釣り合い | あり |
| [`elastic_exact`](02_elastic_exact.md) | 同じ反力 `F_s = k_s s + P(s)` | 厳密幾何での伸び方向の釣り合い | あり |
| [`EFLM`](03_EFLM_five_coordinates.md) | `k_s s` はポテンシャルの勾配、`P(s)` は `urls` 由来の追加一般化トルク | 股・膝・足首の角度方程式 | 実装上、与圧分はなし |

## `elastic_approx` と `elastic_exact`：保存的なばね力

両モデルでは [`spring_response.m`](../../common/function/spring_response.m) が

\[
F_s(s)=k_s s+P(s)
\]

を返し、その積分をばねポテンシャル `U_s` とする。許容範囲 `s ≥ 0` では

\[
U_s(s)=\frac{1}{2}k_s s^2+
\begin{cases}
F_{pre}\delta_e\left(u^3-\frac{1}{2}u^4\right),&0\le s\le\delta_e,\\
F_{pre}\left(s-\frac{\delta_e}{2}\right),&s\ge\delta_e.
\end{cases}
\]

従って `dU_s/ds = F_s`。ばねの伸び速度を求めれば、蓄積エネルギーの変化率は `dU_s/dt = F_s ṡ` である。ばねへのこの仕事をCoTへ別途足すことはせず、姿勢と股・膝のトルク・角速度を通じてモータ仕事へ反映する。

2モデルの差は与圧則ではなく幾何にある。`elastic_approx` はゼロ伸び周りの `l(k),f(k),g(k)` を使い、伸び方向の釣り合いを

\[
F_s-F_r f-T_r g=0
\]

とする。`elastic_exact` は現在の伸びで計算した足先距離 `R(k,s)` と向き `β(k,s)` を使い、

\[
F_s+F_r R_s+T_r\beta_s=0
\]

とする。ここで `F_r,T_r` は床反力を脚長・脚角方向に分けた一般化力である。一次近似では `R_s ≈ -f`、`β_s ≈ -g` なので、両式の対応が分かる。

## `EFLM`：線形ばねと追加トルク

5一般座標のEFLMでは、ばねリンク長 `ℓ(h,k,a)` から `s=ℓ-L_3` を求める。運動方程式の保存的なばね項は

\[
U_{lin}=\frac{1}{2}k_s s^2,\qquad
\nabla_{h,k,a}U_{lin}=k_s s\,\nabla_{h,k,a}s
\]

であり、与圧 `P(s)` は別の一般化トルク `τ_extra` として右辺に加える。[`eflm5_balance.m`](../../EFLM/function/eflm5_balance.m) の式をそのまま書くと、

\[
\tau_{extra}=P(s)\begin{bmatrix}
0\\
\dfrac{L_B}{\ell}\bigl(L_2\sin k-L_D\sin(k-a)\bigr)\\
-\dfrac{L_D}{\ell}\bigl(L_2\sin k-L_B\sin(k-a)\bigr)
\end{bmatrix}.
\]

必要な関節トルクは `τ_required = k_s s ∇s − J_toeᵀ F_GRF − τ_extra`。足首を受動関節とし、`τ_required,a = 0` を姿勢を決める条件に含める。このため与圧は受動足首の釣り合いも変える。

この `τ_extra` は、同じ `P(s)` をばね伸び方向の保存力として作用させた場合の `−P(s)∇s` と**一致しない**。例えばコードの `∂s/∂k` は

\[
\frac{\partial s}{\partial k}
=\frac{-L_2L_D\sin k+L_BL_D\sin(k-a)}{\ell}
\]

なので、`−P(s)∂s/∂k` と上の `τ_extra,k` は係数・角度依存が異なる。滑らかな立ち上がり関数とパラメータをそろえても、`elastic_exact` と `EFLM` は同じ力学モデルにはならない。

`EFLM` の結果には共通の `spring_response` 由来の `springForce`、`springEnergy` も診断・出力用に保存される。ただし、**EFLMの運動方程式で用いた全ばね反力・全ポテンシャルとして読まないこと**。実際に使った線形ばねエネルギーは `linearSpringEnergy`、追加作用は `additionalSpringTorque` と `additionalSpringPower` に分けて保存される。後者は追加トルクと角速度の内積であり、共通ばねポテンシャルの時間微分と同一視できない。

## 最適化結果を比較するとき

同じ探索変数を3モデルに入れた場合、床反力から解析積分する胴体軌道は共通である。一方、与圧の入り方は脚の釣り合い、ばね伸び、関節角・角速度、必要トルクを変え、モータ制約とCoTに影響する。その結果、**最適化後の探索変数が異なれば胴体軌道も異なり得る**。

`elastic_approx` と `elastic_exact` の比較では与圧則は共通なので、主に幾何の一次近似の影響を見られる。`elastic_exact` と `EFLM` の比較には、座標と姿勢の解法に加えて、上記の与圧トルク則の差が含まれる。最適化結果だけからこれらの寄与を分離することはできない。
