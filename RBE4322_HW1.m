clear;
clc;
pi = 3.14;
%% Static Analysis
% Assume assembly is made out of aluminium ​
%coordinates of joints
A=[1.4 .485 0];
B=[1.67 0.99 0];
C=[.255  1.035 0];
D=[.285 .055 0];
E=[.195 2.54 0];
F=[-.98 2.57 0];
G=[.05 .2 0];

L=[-1.715 4.260 0]; %location of load

%coordinates of link's COM
Hab = [((A(1,1) + B(1,1)) / 2) ((A(1,2) + B(1,2)) / 2) 0];
Hbc = [((B(1,1) + C(1,1)) / 2) ((B(1,2) + C(1,2)) / 2) 0];
Hde = [((D(1,1) + E(1,1)) / 2) ((D(1,2) + E(1,2)) / 2) 0];
Hef = [((E(1,1) + F(1,1)) / 2) ((E(1,2) + F(1,2)) / 2) 0];
Hlg = [((L(1,1) + G(1,1)) / 2) ((L(1,2) + G(1,2)) / 2) 0];

%position vectors of COM & relative points
pvHab = Hab-A;
pvHbc = Hbc-B;
pvHde = Hde-D;
pvHef = Hef-E;
pvHlg = Hlg-G;

%length of each link/ distance between joints
AB=norm(B-A);
BC=norm(C-B);
CD=norm(D-C);
DE=norm(E-D);
BE=norm(E-B);
EF=norm(F-E);
FG=norm(G-F);
LG=norm(G-L); % distance between load and grounded joint G

%position vectors 
pvAB=B-A;
pvBC=C-B;
pvCD=D-C;
pvDE=E-D;
pvEF=F-E;
pvFG=G-F;
pvLG=G-L; %load

%without weight of each link considered
syms Ax Ay Bx By Cx Cy Dx Dy Ex Ey Fx Fy Gx Gy inTorque
fA=[Ax Ay 0];
fB=[Bx By 0];
fC=[Cx Cy 0];
fD=[Dx Dy 0];
fE=[Ex Ey 0];
fF=[Fx Fy 0];
fG=[Gx Gy 0];
Ta=[0 0 inTorque];

%weight of links in Newtons (assuming the link material is Al 6061 T6)
linkDensity = [0 2.70 0]; %g/cm^3
linkWidth = [0 10 0]; % cm
linkThickness = [0 5 0]; %cm
jointDiameter = [0 6 0]; %cm
Wab = (linkDensity .* linkWidth .* linkThickness .* AB .* -9.8) ./ 1000 ; 
Wbc = (linkDensity .* linkWidth .* linkThickness .* BC .* -9.8) ./ 1000 ; 
Wcd = (linkDensity .* linkWidth .* linkThickness .* CD .* -9.8) ./ 1000 ; 
Wde = (linkDensity .* linkWidth .* linkThickness .* DE .* -9.8) ./ 1000 ; 
Wef = (linkDensity .* linkWidth .* linkThickness .* EF .* -9.8) ./ 1000 ; 
Wfg = (linkDensity .* linkWidth .* linkThickness .* FG .* -9.8) ./ 1000 ; 
Wl = [0 -200 0] ; %given weight of load in NEWTONS

%Link AB/1
%First equation represents sum of forces
%Second Equation represents sum of moments
eqn1=fA+fB+Wab==0;
eqn2=Ta+cross(pvHab,Wab)+cross(pvAB,fB)==0; %DOUBLE CHECK ORDER OF CROSS
%Link BC
eqn3=fB+fC+Wbc==0;
eqn4=cross(pvBC,fC)+cross(pvHbc,Wbc)==0;
%Link DEC
eqn5=-fC+fD+fE+Wde==0;
eqn6=cross(pvDE,fE)+cross(pvHde,Wde)+cross(pvCD,fC)==0;
%Link EF
eqn7=-fE+fF+Wef==0;
eqn8=cross(pvEF,fF)+cross(pvHef,Wef)==0;
%Link FG with load L
eqn9=fF+fG+Wfg+Wl==0;
eqn10=cross(pvFG,fF)+cross(pvLG,Wl)+cross(pvHlg,Wfg)==0;

solution = (solve([eqn1,eqn2,eqn3,eqn4,eqn5,eqn6,eqn7,eqn8,eqn9, eqn10],[Ax,Ay,Bx,By,Cx,Cy,Dx,Dy,Ex,Ey,Fx,Fy,Gx,Gy,inTorque]));

noWeightforce_Ax=double(solution.Ax)
noWeightforce_Ay=double(solution.Ay)
noWeightforce_Bx=double(solution.Bx)
noWeightforce_By=double(solution.By)
noWeightforce_Cx=double(solution.Cx)
noWeightforce_Cy=double(solution.Cy)
noWeightforce_Dx=double(solution.Dx)
noWeightforce_Dy=double(solution.Dy)
noWeightforce_Ex=double(solution.Ex)
noWeightforce_Ey=double(solution.Ey)
noWeightforce_Fx=double(solution.Fx)
noWeightforce_Fy=double(solution.Fy)
noWeightforce_Gx=double(solution.Gx)
noWeightforce_Gy=double(solution.Gy)
noWeighttorque_T=double(solution.inTorque)