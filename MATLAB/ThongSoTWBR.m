%% THONG SO TWBR - FULL SCRIPT (giu nguyen co che)
clear; clc;

%% 1) Tham so he
km = 0.022;
ke = 0.4;
R  = 1.7;
r  = 0.0325;

Mp = 1;
Mw = 0.03;

Ip = 0.0012;
Iw = 0.000016;

l  = 0.05;
g  = 9.81;

teta_init     = 0;
teta_dot_init = 0;
x_init        = 0;
x_dot_init    = 0;

%% 2) He so trung gian
beta  = 2*Mw + 2*Iw/(r^2) + Mp;
alpha = Ip*beta + 2*Mp*(l^2)*(Mw + Iw/(r^2));

%% 3) Ma tran trang thai A, B, C, D
A = [ 0, 1, 0, 0;
      0, (2*km*ke*(Mp*l*r - Ip - Mp*l^2))/(R*r^2*alpha), (Mp^2*g*l^2)/alpha, 0;
      0, 0, 0, 1;
      0, (2*km*ke*(r*beta - Mp*l))/(R*r^2*alpha),      (Mp*g*l*beta)/alpha, 0 ];

% QUAN TRONG: dat ngoac mau so (R*r*alpha) de dung thu tu toan
B = [ 0;
      (2*km*(Ip + Mp*l^2 - Mp*l*r))/(R*r*alpha);
      0;
      (2*km*(Mp*l - r*beta))/(R*r*alpha) ];

C = [ 1 0 0 0;
      0 0 1 0 ];

D = [ 0;
      0 ];

%% 4) Hien thi ma tran (giu nhu ban dang lam)
A
B
C
D

%% 5) Khao sat tinh dieu khien duoc
P = [B, A*B, (A^2)*B, (A^3)*B];
rankP = rank(P)

%% 6) Khao sat tinh quan sat duoc
L = [C; C*A; C*(A^2); C*(A^3)];
rankL = rank(L)

%% 7) Tinh ham truyen + ve do thi (giu nguyen co che)
% Cac ham nay can Control System Toolbox.
hasCST = license('test','Control_Toolbox') && ~isempty(which('tf'));

if hasCST
    [num, den] = ss2tf(A, B, C, D);

    HTXE1 = tf(num(1,:), den)
    HTXE2 = tf(num(2,:), den)

    % Neu ban muon rlocus tren HTXE1/HTXE2 thi dung:
    % rlocus(HTXE1); grid on;
    % rlocus(HTXE2); grid on;

    % Ban dang goi pzmap (A,B,C,D) trong code cu.
    % Cac phien ban toolbox cho phep pzmap(sys) ro rang hon:
    pzmap(HTXE1); grid on;
    figure; pzmap(HTXE2); grid on;

    nghiem = roots(den)

else
    % Giu co che nhung bao ro ly do khong chay duoc
    warning([ ...
        'Khong the chay ss2tf/tf/pzmap/rlocus vi thieu Control System Toolbox. ', ...
        'Ban van co the xem cuc he bang eig(A) va da thuc dac trung bang poly(A).']);

    % Thay the toi thieu de ban van xem duoc nghiem (khong doi co che dieu khien)
    den = poly(A);       % he so da thuc det(sI-A)
    poles = eig(A)

    nghiem = roots(den)
end
