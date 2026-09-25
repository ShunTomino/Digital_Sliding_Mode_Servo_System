% 離散時間スライディングモードサーボ系を設計して、操舵による二輪車ロボットのロール角制御を行う。
clear all; clc;

%% 二輪車ロボットのパラメータの設定
ToRad = pi/180; % deg -> rad 変換用
ToDeg = 180/pi; % rad -> deg 変換用
g = 9.8; % 重力加速度[m/s^2]

lf = 0.160; % 前輪から重心位置までの距離[m]
lr = 0.140; % 後輪から重心位置までの距離[m]
l = lf + lr; % 前輪から後輪までの距離[m]
m = 0.904; % 車両の質量[kg]
h = 0.155;  % 地面から質点までの高さ[m]
Ix = 0.0100; % 重心位置におけるロール方向の慣性モーメント[Kg・m^2]


%% 制御対象の状態空間モデル
% 状態変数:x = [phi; d_phi], 出力:y = phi (車体ロール角)
A = zeros(2,2);
B = zeros(2,1);
E = zeros(2,1);

A(1,2) = 1;
A(2,1) = m*g*h/(Ix+m*h^2);

B(2) = m*h/l/(Ix+m*h^2);

E(2) = -h/(Ix+m*h^2); 

C = [1,0];


%% モデルの離散化
Ts = 0.005; % サンプル時間(制御周期)[s]

[Ad,Bd_and_Ed] = c2d(A,[B,E],Ts);
Bd = Bd_and_Ed(:,1);
Ed = Bd_and_Ed(:,2);
Cd = C;


%% スライディングモードサーボ系のエラーシステム
% x_[k+1] = A_*x_[k] + B_*delta_u[k]
% x_ = [e;delta_x]

A_ = [eye(1),-Cd*Ad;
    zeros(2,1),Ad];

B_ = [-C*Bd;Bd];

C_ = [Cd,0];


%% 切換関数σ=S*x_の設計
Q = diag([1,0,1]); % 調整用パラメータ
R = 10; % 調整用パラメータ

S = dlqr(A_,B_,Q,R),

SB_ = S*B_, % >0となること。
invSB_ = inv(SB_);

I = eye(3);


%% 切換入力の重み
etta = 0.9; % 0<etta<2とすること。0<etta<1とすればチャタリングせずに到達する。


%% システムの安定性
system = A_-B_*invSB_*S*(A_-I);
 
% 入力の個数r,xの次数n、出力の個数m個としたとき、
% systemの固有値eigは、r個の1とm+n-r個の実部1未満の極となれば安定。
eig = eig(system)


%% Simulinkでシミュレーション実行
file = 'digitalSMC_bicycle_sim';

open_system(file);

set_param(file,'WideLines','on');
%set_param(file,'ShowLineDimensions','off');

z = sim(file);

%save("result.mat","z");