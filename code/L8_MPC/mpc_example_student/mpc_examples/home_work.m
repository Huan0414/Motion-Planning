clear all;
close all;
clc;

p_0 = [0 8 20];
v_0 = [0 0 0];
a_0 = [0 0 0];
K=20;
dt=0.2;

P=[];
V=[];
A=[];
w1 = 100;
w2 = 1;
w3 = 1;
w4 = 1;
for t=0.2:0.2:80
    %% Construct the conical spiral reference signal
    for i = 1:20
        tref = t + i*0.2;
        r=0.25*tref;
        pt(i,1) = r*sin(0.2*tref);
        vt(i,1) = r*cos(0.2*tref);
        at(i,1) = -r*sin(0.2*tref);
        
        pt(i,2) = r*cos(0.2*tref);
        vt(i,2) = -r*sin(0.2*tref);
        at(i,2) = -r*cos(0.2*tref);
        
        pt(i,3) = 20 - 0.5*tref;
        vt(i,3) = -0.5;
        at(i,3) = 0;
    end
    %% Do the MPC
    %% Please follow the example in linear mpc part to fill in the code here to do the tracking
    for ax = 1:3
        [Tp, Tv, Ta, Bp, Bv, Ba] = getPredictionMatrix(K,dt,p_0(ax),v_0(ax),a_0(ax));
        
        dBp = Bp - pt(:,ax);
        dBv = Bv - vt(:,ax);
        dBa = Ba - at(:,ax);
        
        % construct the optimization problem
        H = w4*eye(K)+w1*(Tp'*Tp)+w2*(Tv'*Tv)+w3*(Ta'*Ta);
        F = w1*dBp'*Tp+w2*dBv'*Tv+w3*dBa'*Ta;
        
        Aieq = [Tv;-Tv;Ta;-Ta;eye(K);-eye(K)];
        if ax ~= 3
            bieq = [6*ones(20,1)-Bv;6*ones(20,1)+Bv;3*ones(20,1)-Ba;3*ones(20,1)+Ba;3*ones(20,1);3*ones(20,1)];
        else
            bieq = [6*ones(20,1)-Bv;ones(20,1)+Bv;3*ones(20,1)-Ba;ones(20,1)+Ba;2*ones(20,1);2*ones(20,1)];
        end
        % solve the optimization problem
        J = quadprog(H, F, Aieq, bieq);
        
        %% Apply the first control variable
        j = J(1);
        [p_0(ax),v_0(ax),a_0(ax)] = forward(p_0(ax),v_0(ax),a_0(ax),j,dt);
    end

    %% Log the states
    P = [P;p_0 pt(1,:)];
    V = [V;v_0 vt(1,:)];
    A = [A;a_0 at(1,:)];
end

%% Plot the result

plot(P(:,4:6),'k','LineWidth',1);
hold on
plot(P(:,1:3));
grid on;
legend('Pxref','Pyref','Pzref','Px','Py','Pz');
figure;
plot3(P(:,4),P(:,5),P(:,6),'k','LineWidth',1);
hold on
plot3(P(:,1),P(:,2),P(:,3));
legend('ref','track');
axis equal;
grid on;