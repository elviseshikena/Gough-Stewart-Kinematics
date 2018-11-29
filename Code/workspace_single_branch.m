%workspace.m
%This Matlab code plots the workspace of a manipulator.
% Use the example below a R//P'R, (where ' denotes being perpendicular),
% to generate your own code.
%
% (1) There are three loops that you would have to establish for the range
% of your joint displacements.
% (2) Write your forward kinemtatic equations for x, y, and z
% (3) The code saves a png picture of the workspace. If you want to change
% the view, click on Rotate 3D, Zoom, or Pan icon of the main bar and
% change the aspect of the figure. Once you find the right aspect save
% the figure by typing at the matlab prompt
% >> print -dpng 'workspace_3D.png'
% or use pritscreen. If you try to copy the figure from Matlab, your
% computer may crash. There may be hundereds of thousands of points.
% Code written by Flavio Firmani
function workspace_single_branch
close all; clear all
ms=16; %markersize - the bigger the markersize the more covered the volume
%will be, however the workspace loses its shape.
%Define the joint limits for joint 1, 2 and 3, and the stepsize (MODIFY)
q1=-90:1:90;
q2=0:1:180;
q3=1:0.1:2;
%Prelocating (Do not modify)
density=length(q1)*length(q2)*length(q3);
x=zeros(density,1); y=zeros(density,1); z=zeros(density,1);
count=0;
for i=q1
for j=q2
for k=q3
count=count+1;
%Creating the variables (MODIFY)
theta1=i;
theta2=j;
l4=k;
% constants:
alpha = 225;
w = 270 - alpha;

%Forward Kinematics (Write your forward kinematics for x, y and z)(MODIFY)

x(count) = cosd(w)*l4*sind(theta2)*sind(theta1) - sind(w)*l4*cosd(theta2) + 1.5*cosd(alpha);
y(count) = sind(w)*l4*sind(theta2)*sind(theta1) + cosd(w)*l4*cosd(theta2) + 1.5*sind(alpha);
z(count) = l4*sind(theta2)*cosd(theta1);

end
end
end
% Figure
scrsz = get(0,'ScreenSize');
figure('Position',[1 1 scrsz(3) scrsz(4)]);
axis off;
% Plotting points
angmax=max(z); angmin=min(z);
res=512; graymap=gray(res);
colordot=round((z-angmin)*.9*res/(angmax-angmin));
fscatter3(x,y,z,colordot,graymap,ms);
%Arrows
arrow_length=max([x,y,z])*2;
radius=min(arrow_length)/1000;
tip_arrow=min(arrow_length)/100;
arrow3([0, 0, arrow_length(3)], [0, 0, 0],radius,tip_arrow,1,36, [1, 1, 1]);
text(0,0,arrow_length(3)*1.1,'z_0','FontSize',14)
arrow3([0, arrow_length(2), 0], [0, 0, 0],radius,tip_arrow,1,36, [1, 1, 1]);
text(0,arrow_length(2)*1.1, 0,'y_0','FontSize',14)
arrow3([arrow_length(1), 0, 0], [0, 0, 0],radius,tip_arrow,1,36, [1, 1, 1]);
text(arrow_length(1)*1.1, 0, 0,'x_0','FontSize',14)
view(3)
set(gcf,'color', [1,1,1]) %white background
axis square; axis equal
print -dpng 'workspace_3D.png'
return
function [h] = fscatter3(X,Y,Z,C,cmap,ms)
% [h] = fscatter3(X,Y,Z,C,cmap);
% Plots point cloud data in cmap color classes and 3 Dimensions,
% much faster and very little memory usage compared to scatter3 !
% Original code written by Felix Morsdorf, Jan 2003, Remote Sensing Laboratory Zuerich
% Last Modification Flavio Firmani June 1, 2010
numclass = max(C);
mins = min(C); maxs = max(C); col = cmap;
ii = round(interp1([floor(mins) ceil(maxs)],[1 numclass],C));
hold on; %colormap(cmap);
k = 0;
for j = 1:numclass
jj = find(ii == j);
if ~isempty(jj)
k = k + 1;
h(k) = plot3(X(jj),Y(jj),Z(jj),'.','color',col(j,:), ...
'markersize',ms);
end
end
function arrow3(v,x0,radius,l,scale,ntet,c)
V=norm(v);
salpha=v(3)/V;calpha=sqrt(v(1)*v(1)+v(2)*v(2))/V;
sbeta=0;cbeta=1;
if calpha~=0,sbeta=v(2)/V/calpha;cbeta=v(1)/V/calpha;end
tet=(0:pi/ntet:2*pi)';ct=radius*V*cos(tet);st=radius*V*sin(tet);
x(:,1)=+ct*salpha*cbeta+st*sbeta;
x(:,2)=+ct*salpha*sbeta-st*cbeta;
x(:,3)=-ct*calpha;
ntet2=2*ntet;
v=v*scale;x=x*scale;
p=x0+v;
for i=1:3,x(:,i)=x0(i)+(1-l)*v(i)+c(i)*x(:,i);end
if l<1
plot3(x(:,1),x(:,2),x(:,3),'k-',[p(1)*ones(ntet,1) x(1:2:ntet2,1)]',...
[p(2)*ones(ntet,1) x(1:2:ntet2,2)]',[p(3)*ones(ntet,1) x(1:2:ntet2,3)]',...
'k-',[x0(1) p(1)],[x0(2) p(2)],[x0(3) p(3)],'k-');
else
plot3(x(:,1),x(:,2),x(:,3),'k-',[p(1)*ones(ntet,1) x(1:2:ntet2,1)]',...
[p(2)*ones(ntet,1) x(1:2:ntet2,2)]',[p(3)*ones(ntet,1) x(1:2:ntet2,3)]',...
'k-');
end