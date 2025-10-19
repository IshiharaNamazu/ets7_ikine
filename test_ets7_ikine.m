global d_time

addpath(genpath("./uchidasan"))
addpath(genpath("./SpaceDyn/src/matlab/spacedyn_v2r1"))

%ifdef
global Gravity
global Ez
global d_time
Ez =[0 0 1]';
Gravity =[0 0 0]'; % 重力（地球重力ならば Gravity = [0 0 -9.8]）
d_time =0.01; % シミュレーションの１ステップあたりの時間
    
%%%%%%%%%%%% リンクパラメータ定義と変数の初期化 %%%%%%%%%%%%%%%%%
LP = ets7_linkparam();%LP１とかにしてもよいi.         Sample_LP()を呼び出してLPに格納
SV = make_sv( LP );%SVも名前は自分で定義できる       Sample_SV(LP)を呼び出してSVに格納   同時にサイズを決めている
SV.q = zeros(6,1);
SV.tau = zeros(6,1);
% 各リンクの座標変換行列(方向余弦行列)の計算（リンクi → 慣性座標系）
SV =calc_aa( LP, SV );
% 各リンク重心の位置ベクトル(6×1)を計算
SV =calc_pos( LP, SV );
 
SV = f_dyn_rk2( LP, SV );
%endif

[H_asuta, C_asuta] = calc_asuta(LP, SV);

H_asuta, C_asuta