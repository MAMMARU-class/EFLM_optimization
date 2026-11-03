# urls由来の5一般座標EFLM

## 導出と計算

urls/motion_equation.m と同じく `q=[x,z,h,k,a]` とし、

\[
T=\tfrac12m(\dot x^2+\dot z^2),\quad
U=mgz+\tfrac12 k_s s^2,\quad L=T-U,
\]
\[
\ell=\sqrt{L_2^2+L_B^2+L_D^2+2L_2(L_D\cos k-L_B\cos a)-2L_BL_D\cos(k-a)},\quad s=\ell-L_3.
\]

足先位置を e(q)、接地力を G とすると、urlsと同じ式の構造は

\[
\frac{d}{dt}\frac{\partial L}{\partial\dot q}-\frac{\partial L}{\partial q}
= [0,0,\tau_h,\tau_k,0]^T+e_q^TG+\tau_{ls}+\tau_{st}.
\]

本プロジェクトに合わせて胴体姿勢の固定オフセットphiと股関節から質点へのオフセットLgを0とします。
脚の質量・慣性はurls同様含めず、総質量を胴体質点に置きます。
e(q)=0の2成分と、上式の足首成分を3関節角について同時に解き、残りの角度成分からモータトルクを計算します。
体幹成分はm*xdd=Fx、m*zdd=Fz-mgなので共通の解析積分を使用します。
これは5座標式の代数的な消去であり、脚姿勢の一次近似ではありません。
urlsの毎評価での記号処理・reduceDAEIndex・ode15sは繰り返しません。
関節速度は接地式と足首つり合い式を時間微分して同じ3×3ヤコビアンで求めます。

## urlsの追加バネトルクを保持する点

urls/parameter.m の追加トルクは、次の式を**そのまま保持**しています。

\[
\tau_{ls,k}=f_{extra}\frac{L_B}{\ell}(L_2\sin k-L_D\sin(k-a)),\qquad
\tau_{ls,a}=-f_{extra}\frac{L_D}{\ell}(L_2\sin k-L_B\sin(k-a)).
\]

注意：この式は一般に `-f_extra*∂ell/∂(k,a)` と一致しません。
したがって、exactの保存的な弾性モデルと同一ではなく、与圧の仕事を単純に共通のバネポテンシャルだけで説明できません。
ユーザー指定の「urlsと同じ運動方程式」を優先し、ここを無断で修正式に置き換えていません。
出力MATには `linearSpringEnergy`、`additionalSpringTorque`、`additionalSpringPower` を別々に保存し、
`generalizedCoordinates`、`generalizedVelocity`、`lagrangeResidual`、`contactResidual` も記録します。
共通表示用の `springEnergy` は共通バネ則の積分値であり、このモデルのUそのものではありません。
CSVのforceResidual列はこのモデルでは足首トルク残差[Nm]です（approx/exactの力残差[N]とは異なる）。

## パラメータを共通化するための変更

| urls原版 | 本プロジェクトのEFLM |
|---|---|
| 旧寸法・旧質量・km/h | 共通寸法・総質量・m/s |
| 線形弾性エネルギーは正の伸びのみ | 共通設定どおりks*sを全段階に使用。負の伸びは計算継続用で最終制約違反 |
| 常時の与圧fp_ls | 共通の3u²−2u³立ち上がり。f_extra=preload*rho(s/rise) |
| 減衰cfd*dell/dt | 共通パラメータの減衰ゼロ |
| 伸び端の非線形反力・関節ストッパ | 共通方針どおり反力なし、最適化の不等式制約 |
| 初期整合性だけ追加バネ力を外す別式 | s=0で滑らかな与圧も0のため全時間で同一式 |

脚寸法は固定、初期脚長のみ最適化します。飛行中は弾道運動のみ、脚は描画しません。
初期推定用補間を除き機構の幾何は直接計算します。最終解は同じ組立分枝にあることを確認します。
