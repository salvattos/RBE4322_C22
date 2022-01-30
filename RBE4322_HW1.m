clear;
clc;
%% Initial Position calculations
A=[1.4 .485 0];
B=[1.67 0.99 0];
C=[.255  1.035 0];
D=[.285 .055 0];
E=[.195 2.54 0];
F=[-.98 2.57 0];
G=[.05 .2 0];

values = linkageAnalysis(B,C,E,F);



%% Static Analysis
%Make this pretty
% Assume assembly is made out of aluminium ​
%coordinates of joints
A=[1.4 .485 0];
B=[1.67 0.99 0];
C=[.255  1.035 0];
D=[.285 .055 0];
E=[.195 2.54 0];
F=[-.98 2.57 0];
G=[.05 .2 0];


%coordinates of link's COM
Hab = [((A(1,1) + B(1,1)) / 2) ((A(1,2) + B(1,2)) / 2) 0];
Hbc = [((B(1,1) + C(1,1)) / 2) ((B(1,2) + C(1,2)) / 2) 0];
Hde = [((D(1,1) + E(1,1)) / 2) ((D(1,2) + E(1,2)) / 2) 0];
Hef = [((E(1,1) + F(1,1)) / 2) ((E(1,2) + F(1,2)) / 2) 0];

%position vectors of COM & relative points
pvHab = Hab-A;
pvHbc = Hbc-B;
pvHde = Hde-D;
pvHef = Hef-E;

%length of each link/ distance between joints
AB=norm(B-A);
BC=norm(C-B);
CD=norm(D-C);
DE=norm(E-D);
BE=norm(E-B);
EF=norm(F-E);
FG=norm(G-F);
LF=1.843; % distance between load and joint F

%position vectors 
pvAB=B-A;
pvBC=C-B;
pvCD=D-C;
pvDA=A-D;
pvDE=E-D;
pvEF=F-E;
pvFG=G-F;

unit_GF=-pvFG/FG;
pvFL=unit_GF*LF; 
pvGL=pvFL-pvFG; %load from G to L
L=pvGL+G; %location of load
Hlg = [((L(1,1) + G(1,1)) / 2) ((L(1,2) + G(1,2)) / 2) 0]; % location of COM
pvHlg = Hlg-G; % position of COM from G to Hlg

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
linkDensity = [0 2710 0]; % kg/m^3
linkWidth = [0 0.10 0]; % m
linkThickness = [0 0.05 0]; % m
jointDiameter = [0 0.06 0]; % m
Wab = (linkDensity .* linkWidth .* linkThickness .* AB .* -9.8); % N
Wbc = (linkDensity .* linkWidth .* linkThickness .* BC .* -9.8); % N
Wcd = (linkDensity .* linkWidth .* linkThickness .* CD .* -9.8); % N
Wde = (linkDensity .* linkWidth .* linkThickness .* DE .* -9.8); % N
Wef = (linkDensity .* linkWidth .* linkThickness .* EF .* -9.8); % N
Wfg = (linkDensity .* linkWidth .* linkThickness .* (LF + FG) .* -9.8); % N
Wl = [0 -200 0] ; %given weight of load in NEWTONS

%Link AB/1
%First equation represents sum of forces
%Second Equation represents sum of moments
eqn1=fA-fB+Wab==0;
eqn2=Ta+cross(pvHab,Wab)+cross(pvAB,-fB)==0;
%Link BC
eqn3=fB-fC+Wbc==0;
eqn4=cross(pvBC,-fC)+cross(pvHbc,Wbc)==0;
%Link DEC
eqn5=fC-fD+fE+Wde==0;
eqn6=cross(pvDE,fE)+cross(pvHde,Wde)+cross(-pvCD,fC)==0;
%Link EF
eqn7=-fE+fF+Wef==0;
eqn8=cross(pvEF,fF)+cross(pvHef,Wef)==0;
%Link FG with load L
eqn9=-fF+fG+Wfg+Wl==0;
eqn10=cross(-pvFG,-fF)+cross(pvGL,Wl)+cross(pvHlg,Wfg)==0;

staticsolution = (solve([eqn1,eqn2,eqn3,eqn4,eqn5,eqn6,eqn7,eqn8,eqn9,eqn10],[Ax,Ay,Bx,By,Cx,Cy,Dx,Dy,Ex,Ey,Fx,Fy,Gx,Gy,inTorque]));

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

angvel_BCz=double(positionsolution.omegaBCz)
angvel_DEz=double(positionsolution.omegaDEz)
angvel_EFz=double(positionsolution.omegaEFz)
angvel_FGz=double(positionsolution.omegaFGz)
angacc_BCz=double(positionsolution.alphaBCz)
angacc_DEz=double(positionsolution.alphaDEz)
angacc_EFz=double(positionsolution.alphaEFz)
angacc_FGz=double(positionsolution.alphaFGz)

%% extra acceleration values

angvel_BC=[0 0 angvel_BCz];
angvel_DE=[0 0 angvel_DEz];
angvel_EF=[0 0 angvel_EFz];
angvel_FG=[0 0 angvel_FGz];
angacc_BC=[0 0 angacc_BCz];
angacc_DE=[0 0 angacc_DEz];
angacc_EF=[0 0 angacc_EFz];
angacc_FG=[0 0 angacc_FGz];

accH_AB=cross(alphaAB,pvHab)+cross(omegaAB,cross(omegaAB,pvHab));
accH_BC=cross(angacc_BC,pvHbc)+cross(angvel_BC,cross(angvel_BC,pvHbc));
accH_DE=cross(angacc_DE,pvHde)+cross(angvel_DE,cross(angvel_DE,pvHde));
accH_EF=cross(angacc_EF,pvHef)+cross(angvel_EF,cross(angvel_EF,pvHef));
accH_GL=cross(angacc_FG,pvHlg)+cross(angvel_FG,cross(angvel_FG,pvHlg));

%% dynamic analysis

JAB_A=1/12*(Wab(2)/-9.8)*(linkWidth(2)^2+AB^2)+(Wab(2)/-9.8)*norm(pvHab)^2;
JBC_B=1/12*(Wbc(2)/-9.8)*(linkWidth(2)^2+BC^2)+(Wbc(2)/-9.8)*norm(pvHbc)^2;
JDE_D=1/12*(Wde(2)/-9.8)*(linkWidth(2)^2+DE^2)+(Wde(2)/-9.8)*norm(pvHde)^2;
JEF_E=1/12*(Wef(2)/-9.8)*(linkWidth(2)^2+EF^2)+(Wef(2)/-9.8)*norm(pvHef)^2;
JLG_G=1/12*(Wfg(2)/-9.8)*(linkWidth(2)^2+(LF+FG)^2)+(Wfg(2)/-9.8)*norm(pvHlg)^2;

eqn15=fA-fB+Wab==(Wab(2)/-9.8)*accH_AB;
eqn16=Ta+cross(pvHab,Wab)+cross(pvAB,-fB)==JAB_A*alphaAB;
%Link BC
eqn17=fB-fC+Wbc==(Wbc(2)/-9.8)*accH_BC;
eqn18=cross(pvBC,-fC)+cross(pvHbc,Wbc)==JBC_B*angacc_BC;
%Link DEC
eqn19=fC-fD+fE+Wde==(Wde(2)/-9.8)*accH_DE;
eqn20=cross(pvDE,fE)+cross(pvHde,Wde)+cross(-pvCD,fC)==JDE_D*angacc_DE;
%Link EF
eqn21=-fE+fF+Wef==(Wef(2)/-9.8)*accH_EF;
eqn22=cross(pvEF,fF)+cross(pvHef,Wef)==JEF_E*angacc_EF;
%Link FG with load L
eqn23=-fF+fG+Wfg+Wl==(Wfg(2)/-9.8)*accH_GL;
eqn24=cross(-pvFG,-fF)+cross(pvGL,Wl)+cross(pvHlg,Wfg)==JLG_G*angacc_FG;

dynamicsolution = (solve([eqn15,eqn16,eqn17,eqn18,eqn19,eqn20,eqn21,eqn22,eqn23,eqn24],[Ax,Ay,Bx,By,Cx,Cy,Dx,Dy,Ex,Ey,Fx,Fy,Gx,Gy,inTorque]));

dynamicforce_Ax=double(dynamicsolution.Ax)
dynamicforce_Ay=double(dynamicsolution.Ay)
dynamicforce_Bx=double(dynamicsolution.Bx)
dynamicforce_By=double(dynamicsolution.By)
dynamicforce_Cx=double(dynamicsolution.Cx)
dynamicforce_Cy=double(dynamicsolution.Cy)
dynamicforce_Dx=double(dynamicsolution.Dx)
dynamicforce_Dy=double(dynamicsolution.Dy)
dynamicforce_Ex=double(dynamicsolution.Ex)
dynamicforce_Ey=double(dynamicsolution.Ey)
dynamicforce_Fx=double(dynamicsolution.Fx)
dynamicforce_Fy=double(dynamicsolution.Fy)
dynamicforce_Gx=double(dynamicsolution.Gx)
dynamicforce_Gy=double(dynamicsolution.Gy)
dynamictorque_T=double(dynamicsolution.inTorque)
