%% ME4322 Final Project 
% Renée Gruner-Mitchell, Ander Carbajo, Shawn Salvatto, Erik Wegge
%% Solve Diff Eqs
%Define Initital Conditions as array
x0=[0 0 0 0 0 0 0 0 0 0 0]; %[lambda hR t1 h1 t2 h2 t3 h4 t4 hCR,v(t)]
tspan=[0,10]; %time to run variables over
[t,x]=ode45(@DiffEqSolve, tspan, x0);
%% Plot
figure(1)
plot(t,x(:,1))
title('lambda');
xlabel('Time'); ylabel('Lambda');

%% Function
%detailed calculations shown in report
% _P denotes prime
function xprime=DiffEqSolve(~,x)
lambda = x(1);
hR = x(2);
t1 = x(3);
h1 = x(4);
t2 = x(5);
h2 = x(6);
t3 = x(7);
h4 = x(8);
t4 = x(9);
hCR = x(10);
vT = x(11); % need equation for this guy

%Define differential Eq's
lambda_P = vT - ((R/L)*lambda);
hR_P = ((n/L)*lambda) - ((Brotor/Jrotor)*hR) - (K1*t1) - (K2*t2);
t1_P = ((1/Jrotor)*hR) - ((1/J1)*h1);
h1_P = (K1*t1) - ((B1/J1)*h1);
t2_P = ((1/Jrotor)*hR) - ((1/J2)*h2);
h2_P = (((J2*K2*N1^2)/((J2*N1^2)+(J3*N2^2)))*t2) - (((B2*N1^2)+(B3*N2^2))/((J1*N1^2)+(J3*N2^2))*h2) - (((J2*K3*N1*N2)/((J2*N1^2)+(J3*N2^2)))*t3);
t3_P = ((N1/(N2*J2))*h2) - ((1/J4)*h4);
h4_P = (((J4*K3*N4^2)/((J4*N3^2)+(J5*N4^2)))*t3) - (((B4*N3^2)+(B5*N4^2))/((J4*N3^2)+(J5*N4^2))*h4) - (((J4*K4*N3*N4)/((J4*N3^2)+(J5*N4^2)))*t4);
t4_P = ((N4/(N3*J4))*h4) - ((1/JCR)*hCR);
hCR_P = (((K4*JCR)/(JCR + (MCO*r1^2) + (MCO*r2^2) + (JCO*r3^2) + (JCO*r34^2)))*t4) - ((B_CR_GR + (MCO*r1*r1_P) + (MCO*r2*r2_P) + (B_CR_CO*(r3-1)^2) + (JCO*r3*r3_P) + (BC_CO_RO*(r4-r3)^2) + (JRO*r4*r4_P) + (B_RO_GR*r4^2))/(JCR + (MCO*r1^2) + (MCO*r2^2) + (JCO*r3^2) + (JRO*r4^2)));


xprime = [lambda_P, hR_P, t1_P, h1_P, t2_P, h2_P, t3_P, h4_P, t4_P, hCR_P];
end