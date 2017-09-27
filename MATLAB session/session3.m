load('Cessna_310.mat')

%aileron dynamics
A_da = -10;
B_da = 10;

%rudder dynamics
A_dr = -10;
B_dr = 10;

%To find the gain value for each control surface
%% k_a; aileron
A_a = [A_lat, B_lat(:,1); zeros(1,5), A_da];
B_a = [zeros(5,1); B_da];
C_a = [1,0,0,0,0,0];
D_a = zeros(1,1);

ss_a = ss(A_a,B_a,C_a,D_a);
sys_a = tf(ss_a);

% sisotool(sys_a)
k_phi = 0.1;

%% k_r; rudder
A_r = [A_lat, B_lat(:,1); zeros(1,5), A_dr];
B_r = [zeros(5,1); B_dr];
C_r = [0,1,0,0,0,0];
D_r = zeros(1,1);

ss_r = ss(A_r,B_r,C_r,D_r);
sys_r = tf(ss_r);

% sisotool(sys_r)
k_r = 0.002;
%% k_psi
%roll command
phicmd_phi=feedback(k_phi*ss_a,1);
% tf(phicmd_phi)

%heading hold
s=tf('s'); %Laplace variable definition
g=32.17;             %[ft/s2]
U1=312.5;              %[ft/s]
phi_psi=g/U1/s;
phicmd_psi=phicmd_phi*phi_psi;
sys_psi = tf(phicmd_psi);

% sisotool(sys_psi)

%Kpsi
k_psi=2;

A = [A_lat, B_lat; zeros(2,5), [A_da 0; 0 A_dr]];
B = [zeros(5,2); B_da 0; 0 B_dr];
C = eye(7);
D = zeros(7,2);

ss_final = ss(A,B,C,D);
sys_final = tf(ss_final);
