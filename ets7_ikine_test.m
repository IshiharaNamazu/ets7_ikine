%%%%%%%%%%%%%%%%% global 変数の定義 %%%%%%%%%%%%%%%%%%%%
global d_time
global Gravity
global Ez

addpath('./SpaceDyn/src/matlab/spacedyn_v2r1'); % SpaceDyn のパスを追加

%%%%%% 関節の単位回転軸ベクトル %%%%%
Ez =[0 0 1]';
Gravity =[0 0 0]'; % 重力（地球重力ならば Gravity = [0 0 -9.8]）

d_time =0.003; % シミュレーションの１ステップあたりの時間

%%%%%%%%%%%%%%% 変数初期化 %%%%%%%%%%%%%%%%%
t_all = 10;
r = 0.1;
omega = 2 * pi / 5;

%%%%%%%%%%%% リンクパラメータ定義と変数の初期化 %%%%%%%%%%%%%%%%%
LP = ets7_linkparam();%LP１とかにしてもよいi.         Sample_LP()を呼び出してLPに格納
SV = make_sv( LP );%SVも名前は自分で定義できる       Sample_SV(LP)を呼び出してSVに格納   同時にサイズを決めている

%%%%% ベースから num_e で指定された手先までを結ぶ関節(リンク)を求める %%%%%
num_e = 1;% num_e番目の末端リンクの位置見る
joints = j_num(LP, num_e);%アームの手先までリンクを求める

%%%%% ロボットの初期関節角度を設定 %%%%%
SV.q = zeros(6,1);

% fidw = fopen( 'sample.dat','w' );

%%%%%%%% history %%%%%%%%
time_array = 0:d_time:t_all+1;
pos_e_history = zeros(3, size(time_array, 1));

%%%%%%%% 動画保存の初期化 %%%%%%%%
video_speed_factor = 1;
video_filename = 'ets7_simulation.avi';
video = VideoWriter(video_filename,'Motion JPEG AVI');
video.FrameRate = round(video_speed_factor/d_time); % フレームレートをシミュレーションステップに合わせる
open(video);

%%%%%%%% 描画関連 %%%%%%%%
figure(6);
clf; % figureをクリア
FIG3D = ets7_fig3d();
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3); grid on;
xlim([-3 3]);
ylim([-3 3]);
zlim([-3 3]);
title('Figure');
hold on; % holdをonにして複数のオブジェクトを保持

%%%%%%%%%%%%%%%%% ここからシミュレーションループスタート %%%%%%%%%%%%%%%%%%%%%%%%%%
tic; % 計測開始
itr = 0;
for time = time_array
        itr = itr + 1;
        if mod(itr, 100) == 0
                fprintf('\rprogress: %f/%f', time, t_all);
        end


        %%%%%%%%%%%%%%%%%%%% 順動力学の計算 %%%%%%%%%%%%%%%%%%%%%
        SV = f_dyn_rk2( LP, SV );

        %%%%%%%%%%%%%%%%%%%%%%% 順運動学 %%%%%%%%%%%%%%%%%%%%%%%%%

        %%%%% 手先位置姿勢の計算 (順運動学) %%%%%%
        % 各リンクの座標変換行列(方向余弦行列)の計算（リンクi → 慣性座標系）
        SV =calc_aa( LP, SV );
        % 各リンク重心の位置ベクトル(6×1)を計算
        SV =calc_pos( LP, SV );

        v_ee = [0 0 0.1 0 0 0]'; % 目標手先速度ベクトル
        v_ee = r * omega * [-sin(omega * time) 0 sin(omega * time) 0 0 0]';

        %%%%%%%%%%%%%%%% qdの計算 %%%%%%%%%%%%%%%%%%%
        SV.qd = calc_qd(LP, SV, zeros(6,1), num_e, v_ee);

        %%%%% 手先位置姿勢の計算 %%%%%
        [ POS_e, ORI_e ] =f_kin_e(LP, SV, joints);

        pos_e_history(:, round(time/d_time) + 1) = POS_e;
        %%%%%%%%%%%%%%%%%%%%%%% 描画 %%%%%%%%%%%%%%%%%%%%%%%%%
        set(FIG3D.base.base, 'Vertices', (SV.A0 * FIG3D.base.vertices_local')' + SV.R0');
        for i = 1:6
                set(FIG3D.link(i).link, 'Vertices', (SV.AA(:,i*3-2:i*3) * FIG3D.link(i).vertices_local')' + SV.RR(:,i)');
        end
        drawnow;

        % 動画フレームの保存
        frame = getframe(gcf);
        writeVideo(video, frame);
end
elapsedTime = toc;
disp(['処理時間: ', num2str(elapsedTime), '/', num2str(t_all), ' 秒']);

% 動画ファイルを閉じる
close(video);

%%%%%%%%%%%%%%%%%%%%%%%%%%% シミュレーションループここまで %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%% EOF
