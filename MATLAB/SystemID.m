%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%         u = 0         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
clc;

data = load("v0_phi.mat");
data = data.v0_phi;
phi = data(:,1)/180*pi;
range=100:150;

%smoothing phi
for i=2:length(phi)-1
    phi(i) = (phi(i+1)+phi(i-1))/2;
end

%get phid
dt = 0.01; 
for i=2:length(phi)
    phid(i-1) = (phi(i)-phi(i-1))/dt;
end
phid(length(phi)) = phid(end);
phid = phid';

%get phidd
for i=2:length(phid)
    phidd(i-1) = (phid(i)-phid(i-1))/dt;
end
phidd(length(phid)) = phidd(end);
phidd = phidd';

g2 = -2*cos(phi(range)).*phidd(range)+sin(2*phi(range)).*sec(phi(range)).*phid(range).*phid(range);
g4 = phid(range);
g8 = -phid(range);
g9 = sin(phi(range));
E = phidd(range);
G1 = [g2 g4];
G2 = [g8 g9];
A1 = pinv(G1'*G1)*G1'*E;
B1 = pinv(G2'*G2)*G2'*E;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%         u = 102         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear g2 g4 g8 g9 phi phidd phid range

%%%load theta
data_theta = load("v2_theta.mat");
data_theta = data_theta.v2_theta;

%%%load phi
data_phi = load("v2_phi.mat");
data_phi = data_phi.v2_phi;

range=15:35;
dt = 0.01;

%get thetad
thetad = data_theta(:,1)/180*pi;

%get thetadd
for i=2:length(thetad)
    thetadd(i-1) = (thetad(i)-thetad(i-1))/dt;
end
thetadd(length(thetad)) = thetadd(end);
thetadd = thetadd';

%get phi
phi = data_phi(:,1)/180*pi;

%get phid
dt = 0.01; 
for i=2:length(phi)
    phid(i-1) = (phi(i)-phi(i-1))/dt;
end
phid(length(phi)) = phid(end);
phid = phid';

%get phidd
for i=2:length(phid)
    phidd(i-1) = (phid(i)-phid(i-1))/dt;
end
phidd(length(phid)) = phidd(end);
phidd = phidd';

u = zeros(length(phi),1);
for i=1:length(phi)
   u(i)=102; 
end


g1 = -thetadd(range);
g2 = -2*cos(phi(range)).*phidd(range)+sin(2.*phi(range)).*sec(phi(range)).*phid(range).*phid(range);
g3 = -thetad(range);
g4 = phid(range);
g5 = u(range);
g6 = -thetadd(range);
g7 = -cos(phi(range)).*thetadd(range);
g8 = thetad(range)-phid(range);
g9 = sin(phi(range));
g10 = -u(range);
E1 = phidd(range) - A1(1).*g2 - A1(2).*g4;
E2 = phidd(range) - B1(1).*g8 - B1(2).*g9;
G3 = [g1 g3 g5];
G4 = [g6 g7 g10];

A2 = pinv(G3'*G3)*G3'*E1;
B2 = pinv(G4'*G4)*G4'*E2;

a = [A2(1) A1(1) A2(2) A1(2) A2(3)]
b = [B2(1) B2(2) B1(1) B1(2) B2(3)]