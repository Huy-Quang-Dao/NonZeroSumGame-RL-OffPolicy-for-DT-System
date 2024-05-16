clear;clc;
%% Model
% x(k+1)=Ax(k)+Bu(k)+Dw(k)
A=[0.9065  0.0816 -0.0005; ...
   0.0743 0.9012   -0.0007; ...
   0         0           0.1327 ];
B=[-0.0027;...
   -0.0068;...
    1];
D=[1;...
   0.0062;...
   0];

n=size(A,2);
m1=size(B,2);
m2=size(D,2);
f=n^2+m1^2+m2^2+m1*m2+n*(m1+m2)+20;
Q1 = diag([1,1,1]);
Q2 = diag([1,7,1]);
R11 = 1;
R12 = 6;
R21 = 6;
R22 = 1;
K10=[-1 0 0];
K20=[-1 0 0];
% Initial control matrix
K1 = {}; K1{1} = K10;
K2 = {}; K2{1} = K20;
for i = 1:30
    H1  = kron(eye(3)',eye(3)')-kron((A+B*K1{i}+D*K2{i})',(A+B*K1{i}+D*K2{i})');
    H2 = (Q1+(K1{i}')*R11*K1{i}+(K2{i}')*R12*K2{i});
    H2 = H2(:);
    H3 = (Q2+(K1{i}')*R21*K1{i}+(K2{i}')*R22*K2{i});
    H3 = H3(:);
    P1 = pinv(H1'*H1)*H1'*H2;
    P2 = pinv(H1'*H1)*H1'*H3;
    P1 = reshape(P1,3,3);
    P2 = reshape(P2,3,3);
    K1{i+1}= -pinv(R11+B'*P1*B-B'*P1*D*pinv(R22+D'*P2*D)*D'*P2*B)*(B'*P1*A-B'*P1*D*pinv(R22+D'*P2*D)*D'*P2*A);
    K2{i+1}= -pinv(R22+D'*P2*D-D'*P2*B*pinv(R11+B'*P1*B)*B'*P1*D)*(D'*P2*A-D'*P2*B*pinv(R11+B'*P1*B)*B'*P1*A);
end