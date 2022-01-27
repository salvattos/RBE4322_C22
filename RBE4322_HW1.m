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
pvDA=A-D;
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

staticsolution = (solve([eqn1,eqn2,eqn3,eqn4,eqn5,eqn6,eqn7,eqn8,eqn9, eqn10],[Ax,Ay,Bx,By,Cx,Cy,Dx,Dy,Ex,Ey,Fx,Fy,Gx,Gy,inTorque]));

noWeightforce_Ax=double(staticsolution.Ax)
noWeightforce_Ay=double(staticsolution.Ay)
noWeightforce_Bx=double(staticsolution.Bx)
noWeightforce_By=double(staticsolution.By)
noWeightforce_Cx=double(staticsolution.Cx)
noWeightforce_Cy=double(staticsolution.Cy)
noWeightforce_Dx=double(staticsolution.Dx)
noWeightforce_Dy=double(staticsolution.Dy)
noWeightforce_Ex=double(staticsolution.Ex)
noWeightforce_Ey=double(staticsolution.Ey)
noWeightforce_Fx=double(staticsolution.Fx)
noWeightforce_Fy=double(staticsolution.Fy)
noWeightforce_Gx=double(staticsolution.Gx)
noWeightforce_Gy=double(staticsolution.Gy)
noWeighttorque_T=double(staticsolution.inTorque)

%% position analysis

omegaAB=[0 0 (7450/7)/3600*2*pi]; % 7450 parts per 7 hours assuming 1 revolution is 1 part
alphaAB=[0 0 0]; % input link rotating at a constant velocity

syms omegaBCz omegaDEz omegaEFz omegaFGz alphaBCz alphaDEz alphaEFz alphaFGz
omegaBC=[0 0 omegaBCz];
omegaDE=[0 0 omegaDEz];
omegaEF=[0 0 omegaEFz];
omegaFG=[0 0 omegaFGz];
alphaBC=[0 0 alphaBCz];
alphaDE=[0 0 alphaDEz];
alphaEF=[0 0 alphaEFz];
alphaFG=[0 0 alphaFGz];

eqn11=cross(omegaAB,pvAB)+cross(omegaBC,pvBC)+cross(omegaDE,pvCD)==0;
eqn12=cross(alphaAB,pvAB)+cross(omegaAB,cross(omegaAB,pvAB))+cross(alphaBC,pvBC)+cross(omegaBC,cross(omegaBC,pvBC))+cross(alphaDE,pvCD)+cross(omegaDE,cross(omegaDE,pvCD))==0;
eqn13=cross(omegaDE,pvDE)+cross(omegaEF,pvEF)+cross(omegaFG,pvFG)==0;
eqn14=cross(alphaDE,pvDE)+cross(omegaDE,cross(omegaDE,pvDE))+cross(alphaEF,pvEF)+cross(omegaEF,cross(omegaEF,pvEF))+cross(alphaFG,pvFG)+cross(omegaFG,cross(omegaFG,pvFG))==0;

positionsolution= (solve([eqn11,eqn12,eqn13,eqn14],[omegaBCz,omegaDEz,omegaEFz,omegaFGz,alphaBCz,alphaDEz,alphaEFz,alphaFGz]));

angvel_BC=double(positionsolution.omegaBCz)
angvel_DE=double(positionsolution.omegaDEz)
angvel_EF=double(positionsolution.omegaEFz)
angvel_FG=double(positionsolution.omegaFGz)
angacc_BC=double(positionsolution.alphaBCz)
angacc_DE=double(positionsolution.alphaDEz)
angacc_EF=double(positionsolution.alphaEFz)
angacc_FG=double(positionsolution.alphaFGz)

%% dynamic analysis
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

