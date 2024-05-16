% Code for paper: Data-Driven Nonzero-Sum Game for
%    Discrete-Time Systems Using Off-Policy Reinforcement Learning. 
% Programing Language : Matlab 
% Purpose : Practice and Research

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
x0=[10;-10;10]; % Initial state
%% Model based
K1_s = [0.0086 0.0272 -0.0667];
K2_s = [-0.6444 -0.8736 0.0005];

%% 
K10=[-1 0 0];
K20=[-1 0 0];
% Initial control matrix
K1 = {}; K1{1} = K10;
K2 = {}; K2{1} = K20;
i=1;
phi1 ={};phi2 ={};phi3 ={};phi4 ={};phi5 ={};phi6 ={};phi7 ={}; % Store data to serve LS solution
phi= {}; psi1={};psi2={};% Store data to serve LS solution

%% Collect data to use Off-policy RL algorithm
for k=1:f+2
    if k==1
        x(:,k)=x0;
    else
        e1(:,k-1)=cos(0.5*k)^2 + sin(k) + cos(10*k);
        e2(:,k-1)=sin(k) + 0.2*sin(2*k)+0.3*sin(3*k)+0.4*sin(4*k);
        % e1(:,k-1)=2*rand;
        % e2(:,k-1)=2*rand;
        % probing noise
        u(:,k-1)=K10*x(:,k-1)+e1(:,k-1);
        w(:,k-1)=K20*x(:,k-1)+e2(:,k-1);
        x(:,k)=A*x(:,k-1)+B*u(:,k-1)+D*w(:,k-1);
    end
end
% Data is used to find solutions
%%
while(1)
    Phi=[];Psi1=[];Psi2=[];
    for k=1:f
        phi1{k}=kron(x(:,k)',x(:,k)')-kron(x(:,k+1)',x(:,k+1)');
        % phi2{k}=2*(kron(u(:,k)',x(:,k)')-kron((K1{i}*x(:,k))',x(:,k)'));
        phi2{k}=2*(kron((u(:,k)-K1{i}*x(:,k))',x(:,k)'));
        % phi3{k}=kron(u(:,k)',u(:,k)')-kron((K1{i}*x(:,k))',(K1{i}*x(:,k))');
        phi3{k}=kron((u(:,k)+K1{i}*x(:,k))',(u(:,k)-K1{i}*x(:,k))');
        phi4{k}=kron((w(:,k)+K2{i}*x(:,k))',(u(:,k)-K1{i}*x(:,k))');
        % phi5{k}=2*(kron(w(:,k)',x(:,k)')-kron((K2{i}*x(:,k))',x(:,k)'));
        phi5{k}=2*(kron((w(:,k)-K2{i}*x(:,k))',x(:,k)'));
        phi6{k}=kron((u(:,k)+K1{i}*x(:,k))',(w(:,k)-K2{i}*x(:,k))');
        % phi7{k}=kron(w(:,k)',w(:,k)')-kron((K2{i}*x(:,k))',(K2{i}*x(:,k))');
        phi7{k}=kron((w(:,k)+K2{i}*x(:,k))',(w(:,k)-K2{i}*x(:,k))');
        phi{k}=[phi1(k),phi2(k),phi3(k),phi4(k),phi5(k),phi6(k),phi7(k)];
        psi1{k}=(x(:,k)')*(Q1+(K1{i}')*R11*K1{i}+(K2{i}')*R12*K2{i})*x(:,k);
        psi2{k}=(x(:,k)')*(Q2+(K1{i}')*R21*K1{i}+(K2{i}')*R22*K2{i})*x(:,k);
    end
    % Calculate the matrices needed for the equation Phi*X = Psi1 and Phi*Y
    % = Psi2
    for k=1:f
        Psi1=[Psi1;psi1{k}];
        Psi2=[Psi2;psi2{k}];
    end
    Phic=[];
    for k=1:f
        Phic=[Phic;phi{k}];
    end
    Phi=cell2mat(Phic);
    X=pinv(Phi'*Phi)*Phi'*Psi1;
    vX1=X(1:n^2);
    vX2=X(n^2+1:n^2+n*m1);
    vX3=X(n^2+n*m1+1:n^2+n*m1+m1^2);
    vX4=X(n^2+n*m1+m1^2+1:n^2+n*m1+m1^2+m1*m2);
    vX5=X(n^2+n*m1+m1^2+m1*m2+1:n^2+n*m1+m1^2+m1*m2+m2*n);
    vX6=X(n^2+n*m1+m1^2+m1*m2+m2*n+1:n^2+n*m1+m1^2+m1*m2+m2*n+m1*m2);
    vX7=X(n^2+n*m1+m1^2+m1*m2+m2*n+m1*m2+1:end);
    X1=reshape(vX1,n,n);
    X2=reshape(vX2,m1,n);
    X3=reshape(vX3,m1,m1);
    X4=reshape(vX4,m1,m2);
    X5=reshape(vX5,m2,n);
    X6=reshape(vX6,m2,m1);
    X7=reshape(vX7,m2,m2);

    Y=pinv(Phi'*Phi)*Phi'*Psi2;
    vY1=Y(1:n^2);
    vY2=Y(n^2+1:n^2+n*m1);
    vY3=Y(n^2+n*m1+1:n^2+n*m1+m1^2);
    vY4=Y(n^2+n*m1+m1^2+1:n^2+n*m1+m1^2+m1*m2);
    vY5=Y(n^2+n*m1+m1^2+m1*m2+1:n^2+n*m1+m1^2+m1*m2+m2*n);
    vY6=Y(n^2+n*m1+m1^2+m1*m2+m2*n+1:n^2+n*m1+m1^2+m1*m2+m2*n+m1*m2);
    vY7=Y(n^2+n*m1+m1^2+m1*m2+m2*n+m1*m2+1:end);
    Y1=reshape(vY1,n,n);
    Y2=reshape(vY2,m1,n);
    Y3=reshape(vY3,m1,m1);
    Y4=reshape(vY4,m1,m2);
    Y5=reshape(vY5,m2,n);
    Y6=reshape(vY6,m2,m1);
    Y7=reshape(vY7,m2,m2);
    % Use LS to solve to find the X,Y matrix
  
    i=i+1;
    K1{i}=-(pinv(X3+R11-X4*pinv(Y7+R22)*Y6))*(X2-X4*pinv(Y7+R22)*Y5);
    K2{i}=-(pinv(Y7+R22-Y6*pinv(X3+R11)*X4))*(Y5-Y6*pinv(X3+R11)*X2);
    % Find the control matrix
    % dK1(i-1)=norm(K1{i-1}-K1{i});
    % dK2(i-1)=norm(K2{i-1}-K2{i});
    dK1(i-1)=norm(K1{i-1}-K1_s);
    dK2(i-1)=norm(K2{i-1}-K2_s);
    % Evaluate convergence
    if i>20
        break;
    end
end

%% Plot
figure(1)
subplot(2,1,1)
j=0:1:i-2;
plot(j,dK1,'-oc','LineWidth',3,'MarkerEdgeColor','m','MarkerFaceColor','c','MarkerSize',3)
xlim([0 inf])
grid on
ylabel('||K1^i-K1^*||')
xlabel('Iteration step')
legend('||K1^i-K1^*||')
subplot(2,1,2)
plot(j,dK2,'-oc','LineWidth',3,'MarkerEdgeColor','m','MarkerFaceColor','c','MarkerSize',3)
xlim([0 inf])
grid on
ylabel('||K2^i-K2^*||')
xlabel('Iteration step')
legend('||K2^i-K2^*||')


