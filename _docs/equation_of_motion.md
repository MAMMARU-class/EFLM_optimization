# GM09：EFLMとSLIPモデルの運動方程式

基準は [`../out/GM_09.pdf`](../out/GM_09.pdf)（全2ページ）の式(1)〜(18)である。以下ではPDFの記号と符号を採用し、式どうしの関係を補足する。`ds` は仮想バネの変位を表す一つの変数名であり、微分記号ではない。その仮想変位は $\delta(ds)$ と記す。追加した厳密モデルの節では同じ変数を簡潔に $s\equiv ds$ と表記する。

## 0. 図から確認した座標・符号規約

計算では、`eflm_derivatives.png` および既存の順運動学コードと整合する次の規約を用いる。

- $+x$ は右、$+z$ は上。
- $\boldsymbol Y=[Y_x,Y_z]^T$ は股関節（SLIP質点）から足先へ向かうベクトル。
- $r=\|\boldsymbol Y\|$。
- 脚角は $+x$ 軸から下向きを正として

$$
\theta=\operatorname{atan2}(-Y_z,Y_x)
$$

  とする。
- EFLMの正の関節角・関節トルクは `model_definition_angle_torque_EFLM.png` の矢印に従う。
- バネ変位は

$$
s=L_{\mathrm{spring}}-L_3
$$

  とする。伸びを正とするため、バネが機構へ及ぼす復元力は $F_s=-k_s s$ である。一方、弾性エネルギーをラグランジュ方程式の左辺へ入れれば、質量を持たない $s$ 座標の釣り合いは $k_s s=Q_s$ となる。この2つは同じ符号規約である。

`model_definition_*.png`、`EFLM_stand.png`、`SLIP_stand.png` は $+x$ を左向きに描いており、上記の計算座標を左右反転した図である。これらの図から角度・トルクの回転方向は読み取れるが、$F_x$ や $Y_x$ の符号を計算式へ移す際は左右反転が必要である。特に `eflm_derivatives.png` と他の図の $x$ 軸をそのまま混在させてはならない。

この文書では、仮想脚の幾何に使う $(x,z)$ は原則として $\boldsymbol Y$ の成分を表す。胴体の世界座標 $(x_b,z_b)$ を一般化座標にする場合、立脚中の固定足先 $(x_t,z_t)$ に対して

$$
\boldsymbol Y=
\begin{bmatrix}x_t-x_b\\z_t-z_b\end{bmatrix},\qquad
\delta\boldsymbol Y=-\delta
\begin{bmatrix}x_b\\z_b\end{bmatrix}
$$

である。したがって、$\boldsymbol Y$ に共役な一般化力と胴体並進座標に共役な一般化力は符号が反対になる。

## 1. EFLMの運動方程式

PDFの式(1)〜(6)では、一般化座標、能動入力、床反力による一般化力を次のように置く。

$$
\boldsymbol q_{\mathrm{EFLM}}=
\begin{bmatrix}x&z&q_h&q_k&q_a\end{bmatrix}^{T},\qquad
\boldsymbol Q=\boldsymbol Q_{\mathrm{act}}+\boldsymbol Q_{\mathrm{GRF}},
$$

$$
\boldsymbol Q_{\mathrm{act}}=
\begin{bmatrix}0&0&\tau_h&\tau_k&\tau_a\end{bmatrix}^{T},\qquad
\boldsymbol Q_{\mathrm{GRF}}=
\begin{bmatrix}
F_x\\F_z\\J_{\mathrm{toe}}^{T}\begin{bmatrix}F_x\\F_z\end{bmatrix}
\end{bmatrix}.
$$

$J_{\mathrm{toe}}$ は足先位置の関節角 $(q_h,q_k,q_a)$ に関するヤコビアンである。PDFのエネルギーとラグランジュ方程式は

$$
L=T-U=\frac12m\|\boldsymbol v\|^2
-\left(mgz+\frac12k_s(ds)^2\right),\qquad
\frac{d}{dt}\frac{\partial L}{\partial\dot{\boldsymbol q}_{\mathrm{EFLM}}}
-\frac{\partial L}{\partial\boldsymbol q_{\mathrm{EFLM}}}
=\boldsymbol Q.
$$

立脚中は足先固定条件

$$
x_{\mathrm{toe}}=x_0,\qquad z_{\mathrm{toe}}=0
$$

を加える。PDFに記載された構成は、重心並進の微分方程式2本と、関節に関する釣り合い3本・足先固定2本の代数方程式である。したがって合計は微分方程式2本、代数方程式5本となる。PDFのこの節では、$ds$ をEFLMの関節角から計算する機構式までは示していない。

## 2. SLIPモデル：出力側座標での一般式

PDFの式(7)〜(10)では、一般化座標を $\boldsymbol q_{\mathrm{SLIP}}=[x,z,ds]^T$ とする。ここでは $(x,z)=(Y_x,Y_z)$、すなわち股関節から足先へのベクトルの成分として読む。脚長を $r=\sqrt{x^2+z^2}$、脚角度を $\theta=\operatorname{atan2}(-z,x)$ と置くと、座標の変分は

$$
\delta r=\frac{x}{r}\delta x+\frac{z}{r}\delta z,\qquad
\delta\theta=\frac{z}{r^2}\delta x-\frac{x}{r^2}\delta z.
$$

脚の出力側の力 $F_r$ とトルク $\tau_r$ を用いると、仮想仕事の表記は $\delta W_{\mathrm{act}}=F_r\delta r+\tau_r\delta\theta$ であり、PDFでは一般化力を

$$
\boldsymbol Q_{\mathrm{act}}=
\begin{bmatrix}
F_r\,x/r+\tau_r\,z/r^2\\
F_r\,z/r-\tau_r\,x/r^2\\
F_r\,\delta r/\delta ds+\tau_r\,\delta\theta/\delta ds
\end{bmatrix}
$$

と記している。これは $\boldsymbol Y$ に共役な力である。胴体の世界座標を運動方程式の一般化座標に使う場合は、前節の $\delta\boldsymbol Y=-\delta[x_b,z_b]^T$ により最初の2成分へ負号を付ける。

第3成分を数値化するには、EFLMのリンク幾何からバネ変位に対する脚長・脚角度の変化を求める必要がある。PDFではこの関係を未指定として、次節で $f(l),g(l)$ による近似を導入する。

## 3. SLIPモデル：脚長依存の線形近似

PDFの式(11)〜(14)を用いる。$(r,\theta)$ は脚の出力側座標、$(l,q)$ はバネ自然長側の仮想アクチュエータ座標である。図の厳密順運動学を $r=R(l,q,s)$、$\theta=\Theta(l,q,s)$ とすると、式(11)は $s=0$ 周りの一次近似に相当する。

$$
l=r+f(l)\,ds,\qquad q=\theta+g(l)\,ds.\tag{11}
$$

すなわち $r\simeq l-f(l)s$、$\theta\simeq q-g(l)s$ なので、図に示された厳密感度との符号関係は

$$
f(l)=-\left.\frac{\partial R}{\partial s}\right|_{s=0},\qquad
g(l)=-\left.\frac{\partial\Theta}{\partial s}\right|_{s=0}.
$$

`eflm_derivatives.png` で $\partial r/\partial s$ と $\partial\theta/\partial s$ が負になっている範囲では、式(11)の $f(l),g(l)$ は正になる。したがって、厳密感度をそのまま $f,g$ として代入すると符号が逆になる。

$l$ が右辺にも現れるため、第一式は $l$ に関する陰的な式である。$f'(l)=df(l)/dl$、$g'(l)=dg(l)/dl$、$D=1-f'(l)ds$ と置く。$D\ne0$ の範囲で式(11)を変分すると、

$$
\delta l=\frac{\delta r+f(l)\delta(ds)}{D},\qquad
\delta q=\delta\theta+g'(l)ds\,\delta l+g(l)\delta(ds).\tag{12}
$$

仮想アクチュエータ側の力・トルクを $(F,\tau)$ とする。その仕事 $\delta W_{\mathrm{act}}=F\delta l+\tau\delta q$ に式(12)を代入し、$\delta r,\delta\theta,\delta(ds)$ の係数を比較すると、

$$
\delta W_{\mathrm{act}}=F_r\delta r+\tau_r\delta\theta+Q_{ds}\delta(ds),
$$

$$
F_r=\frac{F+\tau g'(l)ds}{1-f'(l)ds},\qquad
\tau_r=\tau,\qquad
Q_{ds}=F_r f(l)+\tau_r g(l).\tag{13}
$$

すなわち、$(F,\tau)$ は仮想アクチュエータ側の入力、$(F_r,\tau_r)$ は脚の出力側の力である。$r,\theta$ をさらに $x,z$ に変換すると、式(14)の一般化力を得る。

$$
\boldsymbol Q_{\mathrm{act}}=
\begin{bmatrix}
F_r\,x/r+\tau_r\,z/r^2\\
F_r\,z/r-\tau_r\,x/r^2\\
F_r f(l)+\tau_r g(l)
\end{bmatrix}.\tag{14}
$$

PDFの弾性エネルギー $U_s=\tfrac12k_s(ds)^2$ を用い、バネ座標の慣性を持たせないと、$ds$ 方向の式は $k_s\,ds=Q_{ds}=F_r f(l)+\tau_r g(l)$ という代数的な釣り合いになる。

## 4. 指定床反力から入力への逆算

### 4.1 床反力から出力側の脚力へ

$x,z$ と床反力 $(F_x,F_z)$ が与えられたとする。式(14)の最初の2成分を逆に解くと、PDFの式(15)になる。

$$
F_r=\frac{xF_x+zF_z}{r},\qquad
\tau_r=zF_x-xF_z,\qquad r=\sqrt{x^2+z^2}.\tag{15}
$$

ここでは $r\ne0$ とする。$F_r$ は脚の長さ方向、$\tau_r$ は脚角度方向に対応する一般化力である。

この式の $(F_x,F_z)$ は $(x,z)=(Y_x,Y_z)$ に共役な力として解釈する必要がある。入力が「胴体へ作用する床反力」 $(F_x^{\mathrm{GRF}},F_z^{\mathrm{GRF}})$ で、足先固定の $\boldsymbol Y=\boldsymbol p_{\mathrm{toe}}-\boldsymbol p_{\mathrm{body}}$ を使う場合は

$$
\begin{bmatrix}F_x\\F_z\end{bmatrix}
=-
\begin{bmatrix}F_x^{\mathrm{GRF}}\\F_z^{\mathrm{GRF}}\end{bmatrix}
$$

を式(15)へ代入する。力をロボットが地面へ及ぼす向きで定義するか、地面がロボットへ及ぼす向きで定義するかによって、式(15)の $F_r,\tau_r$ は同時に反転する。

### 4.2 自然長側の脚長とバネ変位を求める

この時点で $r,F_r,\tau_r$ は既知だが、式(13)の $f(l),g(l),f'(l),g'(l)$ を評価するには $l$ と $ds$ が必要になる。式(11)の脚長の関係とバネ座標の釣り合いを組み合わせ、PDFの式(16)を同時に解く。

$$
\begin{cases}
l-r-f(l)\,ds=0 & \text{（幾何の関係）},\\
k_s\,ds-F_r f(l)-\tau_r g(l)=0 & \text{（バネ座標の釣り合い）}.
\end{cases}\tag{16}
$$

$k_s\ne0$ なら第二式から $ds=[F_r f(l)+\tau_r g(l)]/k_s$ と表せるが、右辺も $l$ に依存する。したがって、第一式と合わせて $l$ を決める。近似の有効範囲内で連続な解を選び、得た $l,ds$ を後続の式へ代入する。角度側の仮想座標も必要なら $q=\theta+g(l)\,ds$ で計算する。

#### 4.2.1 本プロジェクトで使用する予圧立ち上がりモデル

式(16)はPDFに示された線形ばねの場合である。本プロジェクトでは、通常の弾性力 $k_s\,ds$ を全伸び領域で維持し、予圧成分だけを微小変位区間で滑らかに立ち上げる。ばね反力を

$$
F_s(ds)=k_s\,ds+F_{\mathrm{pre}}\rho(ds)
$$

と定義する。ここで

$$
u=\frac{ds}{\delta_e},\qquad
\rho(ds)=
\begin{cases}
0, & ds\le0,\\
3u^2-2u^3, & 0<ds<\delta_e,\\
1, & ds\ge\delta_e
\end{cases}
$$

である。$3u^2-2u^3$ は、区間の両端で値と傾きを連続に接続するsmoothstepである。したがって、$ds=0$ ではばね反力がゼロとなり、$ds\ge\delta_e$ では

$$
F_s(ds)=k_s\,ds+F_{\mathrm{pre}}
$$

となる。`sim_env_EFLM_old` に従い、

$$
k_s=17.43\times10^3\ \mathrm{N/m},\qquad
F_{\mathrm{pre}}=3.4\ \mathrm{N}
$$

を用いる。予圧立ち上がり幅の初期値は $\delta_e=0.05\ \mathrm{mm}$ とし、最終的には数値安定性を確認して確定する。ばね伸びの許容範囲は

$$
0\le ds\le15\ \mathrm{mm}
$$

とする。$ds<0$ または $ds>15\ \mathrm{mm}$ は、ストッパトルクを発生させず、許容誤差を考慮したうえで実現不可能な候補として扱う。

この構成則を使う場合、式(16)の第2式は

$$
F_s(ds)-F_r f(l)-\tau_r g(l)=0\tag{16a}
$$

へ置き換える。$F_s$ と整合するばねポテンシャルは

$$
U_s(ds)=
\begin{cases}
0, & ds=0,\\[2mm]
\dfrac12k_s\,ds^2+F_{\mathrm{pre}}
\left(\dfrac{ds^3}{\delta_e^2}-\dfrac{ds^4}{2\delta_e^3}\right),
&0<ds<\delta_e,\\[4mm]
\dfrac12k_s\,ds^2+F_{\mathrm{pre}}\,ds
-\dfrac12F_{\mathrm{pre}}\delta_e,
&ds\ge\delta_e
\end{cases}
$$

であり、$\partial U_s/\partial ds=F_s(ds)$ を満たす。このため、ばね構成則を置き換えても仮想仕事およびラグランジュ方程式との整合性は維持される。

物理的な定義域は $ds\ge0$ である。非線形ソルバの中間反復では、$ds<0$ に対して $F_s=k_sds$、$U_s=\tfrac12k_sds^2$ と数値的に延長するが、負の伸びを物理的に許容する意味ではない。最終解は伸び下限制約で判定する。この延長では $ds=0$ の両側で剛性が $k_s$ となる。

### 4.3 仮想アクチュエータ入力を計算する

式(13)の $F_r$ の式を $F$ について解き直すと、PDFの式(17)になる。

$$
F=[1-f'(l)ds]F_r-g'(l)ds\,\tau_r,\qquad
\tau=\tau_r.\tag{17}
$$

これが $F$ の由来である。連立方程式(16)は $F$ を直接与える式ではなく、式(17)に代入する $l,ds$ を決める。$F$ に $F_r$ 以外の項が入るのは、仮想直動座標 $l$ の変化が脚長 $r$ と脚角度 $\theta$ の両方の仕事に結び付くためである。

### 4.4 実関節トルクへの変換

受動関節と機構拘束を消去し、$(l,q)$ を能動関節角 $(q_h,q_k)$ の関数として定義できる場合、仮想仕事からPDFの式(18)を得る。

$$
\begin{bmatrix}\tau_h\\\tau_k\end{bmatrix}
=J_{lq}^{T}\begin{bmatrix}F\\\tau\end{bmatrix},\qquad
J_{lq}=\frac{\partial(l,q)}{\partial(q_h,q_k)}.\tag{18}
$$

実際にこの式を評価するには、受動足首を含むEFLMの機構拘束から $J_{lq}$ を計算する。PDFにはその具体的な関数形は示されていない。

計算順は **$(F_x,F_z,x,z)\to(F_r,\tau_r)\to(l,ds)\to(F,\tau)\to(\tau_h,\tau_k)$** である。

## 5. `elastic_exact`：厳密順運動学を使う場合

### 5.1 厳密な幾何写像

`elastic_exact` では式(11)の $f(l),g(l)$ による一次近似を使わず、4節リンクの厳密順運動学を

$$
r=R(l,q,s),\qquad \theta=\Theta(l,q,s)
$$

として用いる。ここで $(l,q)$ は $s=0$ の剛体基準モデルを表す仮想アクチュエータ座標で、$s=L_{\mathrm{spring}}-L_3$ である。実装上は $(l,q)$ に対応する能動関節角 $(q_h,q_k)$ を求め、バネ長を $L_3+s$ としてEFLMの閉リンク順運動学を計算すれば、足先ベクトル $\boldsymbol Y(l,q,s)$ が得られる。

$$
R=\sqrt{Y_x^2+Y_z^2},\qquad
\Theta=\operatorname{atan2}(-Y_z,Y_x).
$$

図の感度式は、$(l,q)$、すなわち対応する $(q_h,q_k)$ を固定した偏微分である。

$$
R_s=\frac{\partial R}{\partial s}
=\frac{\boldsymbol Y^T\boldsymbol Y_s}{R},
$$

$$
\Theta_s=\frac{\partial\Theta}{\partial s}
=\frac{Y_zY_{x,s}-Y_xY_{z,s}}{R^2}.
$$

この式の $\Theta_s$ の符号は、$\Theta=\operatorname{atan2}(-Y_z,Y_x)$ を微分した結果と一致する。

### 5.2 厳密モデルでの $l,q,s$ の決定

出力側の状態 $(r,\theta)$ と力 $(F_r,\tau_r)$ が与えられた場合、未知数 $(l,q,s)$ は次の3式から求める。

$$
\begin{cases}
R(l,q,s)-r=0,\\
\Theta(l,q,s)-\theta=0,\\
k_s s+F_r R_s(l,q,s)+\tau_r\Theta_s(l,q,s)=0.
\end{cases}\tag{19}
$$

本プロジェクトの予圧立ち上がりモデルでは、第3式を

$$
F_s(s)+F_r R_s(l,q,s)+\tau_r\Theta_s(l,q,s)=0\tag{19a}
$$

へ置き換える。ここで $F_s$ は4.2.1節と同じ構成則であり、$k_s s$ は伸びの全段階で含まれる。

第3式の符号は、$s$ を伸び正、復元力を $-k_s s$ とした結果である。厳密写像の $s$ 方向の仮想仕事から

$$
Q_s=-F_r R_s-\tau_r\Theta_s
$$

となり、質量を持たないバネ座標の釣り合い $k_s s=Q_s$ と組み合わせると式(19)を得る。一次近似 $R_s=-f$、$\Theta_s=-g$ を代入すれば、式(16)の

$$
k_s s-F_r f(l)-\tau_r g(l)=0
$$

へ戻る。

したがって厳密モデルの $l$ は、単独の陽な式で求めるのではなく、厳密な位置・角度の一致とバネ釣り合いを同時に満たす根として求める。時間積分中は前時刻の $(l,q,s)$ を初期値にして同じ連続解枝を追跡する。

### 5.3 厳密な力変換

固定した $s$ における厳密ヤコビアンを

$$
A(l,q,s)=
\frac{\partial(R,\Theta)}{\partial(l,q)}
=
\begin{bmatrix}
R_l&R_q\\
\Theta_l&\Theta_q
\end{bmatrix}
$$

とする。$\det A\ne0$ の範囲では、仮想仕事の一致から仮想アクチュエータ入力は

$$
\begin{bmatrix}F\\\tau\end{bmatrix}
=A^T
\begin{bmatrix}F_r\\\tau_r\end{bmatrix}.\tag{20}
$$

一次近似では

$$
A=
\begin{bmatrix}
1-f'(l)s&0\\
-g'(l)s&1
\end{bmatrix}
$$

となり、式(20)は式(17)の

$$
F=[1-f'(l)s]F_r-g'(l)s\tau_r,\qquad \tau=\tau_r
$$

に一致する。最後に式(18)の $J_{lq}^T$ で $(F,\tau)$ を股・膝トルクへ変換する。

### 5.4 3モデルの対応

- `rigid`：$s=0$。$r=l$、$\theta=q$ とし、弾性座標の釣り合いを解かない。
- `elastic_approx`：$R\simeq l-f(l)s$、$\Theta\simeq q-g(l)s$ の一次近似と式(16)〜(18)を使う。
- `elastic_exact`：厳密な $R(l,q,s),\Theta(l,q,s)$、式(19)、式(20)を使う。

## 6. 図の間で確認された齟齬・注意点

1. `eflm_derivatives.png` は $+x$ を右向きに描くが、`model_definition_*.png`、`EFLM_stand.png`、`SLIP_stand.png` は左向きに描く。計算時は座標を一方へ統一する必要がある。本書は既存順運動学と一致する右向きを採用した。
2. `model_definition_angle_length_SLIP.png` は脚角を $q_{\mathrm{SLIP}}$、`SLIP1.png` は $\theta$、`SLIP2.png` は $q$ と記す。ここでは弾性変形後の出力角を $\theta$、$s=0$ 側の仮想アクチュエータ角を $q$ と区別する。
3. `model_definition_angle_length_SLIP.png` の力は $F_p$、`SLIP.png` は入力側を $F$、出力側を $F_r$、`SLIP_stand.png` は $F_{\mathrm{SLIP}}$ と記す。式(11)〜(20)では入力側を $F$、出力側を $F_r$ に統一する。
4. `model_definition_length_EFLM.png` の注記は $L_3$ を初期バネ長、$l_s$ を現在長としている。`eflm_derivatives.png` の $s=L_{\mathrm{spring}}-L_3$ と整合する。
5. `eflm_derivatives.png` の幾何パラメータは $L_1/L_2/L_4=130/140/110$ mm、$L_3/L_B/L_D=135/30/23$ mmであり、`sim_env_EFLM_old` と一致する。一方、`urls` は $100/90/88$ mm、$75/25/30$ mmを使う。したがって `urls` の歩行最適化ロジックを3モデルへ移植する際も、運動学・感度関数・初期姿勢をそのまま流用してはならない。
6. `eflm_derivatives.png` は $k_E=17.43$ N/mm、予圧0の条件を記載し、`urls` は下腿側ばね定数 $1.2$ N/mm、初張力 $3.4$ Nを使う。一方、`sim_env_EFLM_old` には $k_s=17.43$ N/mm と初張力 $3.4$ Nが記載されている。本プロジェクトでは `sim_env_EFLM_old` を優先し、4.2.1節の構成則を3モデルの共通条件とする。
7. `SLIP.png`、`SLIP_stand.png` の矢印だけでは、$F,F_r$ がロボットへ作用する力か、ロボットが地面へ及ぼす力かが一意に読めない。床反力は作用反作用で反転するため、実装では式(15)へ渡す力の向きを明示する。本文では $(x,z)=\boldsymbol Y$ に共役な力を式(15)の入力とした。

## 7. 実装時の関節角・ヤコビアン計算

### 7.1 `elastic_approx`

足先ベクトル $(x,z)$ とバネ変位 $s=ds$ が既知の場合、まず

$$
r=\sqrt{x^2+z^2},\qquad \theta=\operatorname{atan2}(-z,x)
$$

を計算する。近似関係

$$
l-r-f(l)s=0,\qquad q=\theta+g(l)s
$$

から $(l,q)$ を求め、$s=0$ のEFLM逆運動学

$$
(l,q)\longrightarrow(q_h,q_k,q_a)
$$

を解く。これは近似モデルで定義された仮想アクチュエータ座標を実関節角へ写す処理である。

ヤコビアンは

$$
J_{lq}=\frac{\partial(l,q)}{\partial(q_h,q_k)}
$$

とし、厳密な閉リンク順運動学を $s=0$ で関節角について微分して求める。初期実装では中心差分を使用できる。関節速度は、$J_{lq}$ が正則な範囲で

$$
\begin{bmatrix}\dot q_h\\\dot q_k\end{bmatrix}
=J_{lq}^{-1}
\begin{bmatrix}\dot l\\\dot q\end{bmatrix}
$$

から計算する。関節トルクは式(18)を使う。

近似関係を含む逆運動学が不安定または計算負荷になる場合、`elastic_approx` の関節角リミット評価に限り、$(r,\theta)$ を $ds=0$ の逆運動学へ直接渡してよい。トルク計算では式(16)〜(18)を維持する。

### 7.2 `elastic_exact`

式(19)から $(l,q,s)$ を求め、厳密な閉リンク運動学から関節角とヤコビアンを計算する。$s=0$ への置換や一次近似は使用しない。
