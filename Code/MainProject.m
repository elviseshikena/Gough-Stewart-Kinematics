function MainProject
% The following code imports our GS platform design into matlab
clc; clear; close all;
%% 1. SETUP
%% 1.1. Define size of figure and create figure handle
set(0,'Units','pixels');
dim = get(0,'ScreenSize');
fig_handle = figure('doublebuffer','on','Position',[0,35,dim(3),dim(4)-100],...
    'Name','3D Object','NumberTitle','off');
set(gcf,'color', [1 1 1]) %Background Colour

%% 1.2. Define the light in the figure
set(fig_handle,'Renderer','zbuffer','doublebuffer','off')
light('color',[.5,.5,.5],'position',[0,1,3],'Style','infinite')
lighting gouraud
daspect([1 1 1]);
axis off
view(60,30)

%%% Arrows
% You need to have the file arrow3 in the same directory
% arrow_length=400; hold on
% line([0,0],[0,0], [0,arrow_length]); text(0,0,arrow_length*1.1,'z_0','FontSize',14);
% line([0,0],[0,arrow_length],[0,0]); text(0,arrow_length*1.1, 0,'y_0','FontSize',14);
% line([0,arrow_length],[0,0],[0,0]); text(arrow_length*1.1, 0, 0,'x_0','FontSize',14);

%% 1.3 Fix view of Plot
Xmin=-3000; Xmax=3000;
Ymin=-3000; Ymax=3000;
Zmin=-9000; Zmax=5000;
axis([Xmin Xmax Ymin Ymax Zmin Zmax]);

clear arrow_length dim;

%% 2. SolidParts
%% 2.1. Convert figures into Object
% Load parts into code
% From top to bottom

% Create File variables
partFiles = [
    {'STL/Platform.mat'};...
    repelem({'STL/JointTop.mat'}, 6,1);...
    repelem({'STL/TubeTop.mat'}, 6,1);...
    repelem({'STL/TubeBottom.mat'}, 6,1);...
    repelem({'STL/JointBottom.mat'}, 6,1);...
    {'STL/Base.mat'}
    ];
NUM_PARTS = length(partFiles);
TOP_JOINT_1 = 2;
TOP_TUBE_1 = 8;
BOTTOM_TUBE_1 = 14;
BOTTOM_JOINT_1 = 20;

% Loop through to load files
obj{NUM_PARTS} = [];
for i=1:NUM_PARTS
    load(string(partFiles(i)), 'object');
    setappdata(0,'object_data',object);
    object = getappdata(0,'object_data');
    obj{i} = object;
end

q_(NUM_PARTS) = patch('faces', obj{end}.F, 'vertices', obj{end}.V);
set(q_(end),'EdgeColor','none');
for i=1:NUM_PARTS
    q_(i) = patch('faces', obj{i}.F, 'vertices', obj{i}.V);
    set(q_(i),'EdgeColor','none');
end

clear object partFiles;

%% 2.2. Coloring Parts
% Set colour to the componenets
% Color Base and Platform
set(q_(1),'FaceColor', [0.25,.6,1]);
set(q_(end),'FaceColor', [1,.5,0]);

% Color Top Joints
for i=2:7
    set(q_(i),'FaceColor', [1,1,.5]);
end

% Color Top Tube
for i=8:13
    set(q_(i),'FaceColor', [1,.242,.293]);
end

% Color Bottom Tube
for i=14:19
    set(q_(i),'FaceColor', [0,1,0]);
end

% Color Bottom Joints
for i=20:25
    set(q_(i),'FaceColor', [1,0,1]);
end

%% 2.3. ANIMATION
% Animation (DO NOT CHANGE)
% Setting up the animation of the plaform and creating an exportable
% gif of the animation.
RGB = 256; % Resolution
fm = getframe;
[img,map] = rgb2ind(fm.cdata,RGB,'nodither');

%% 3. KINEMATICS
% Kinematics Setup
% Plafrom parameters
NUM_JOINTS = 6; % number if joints
pRadius = 1000; % mm Plafrom radius
bRadius = 1500; % mm Base radius

% Position and Orientation of Mobile Platform
% Path Generation
P_ee=[repelem(0,10);
    0,300,650,900,1000,1000,900,650,300,0; % mm
    repelem(1800,10); %mm
    0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0;
    0,-5,-10,-15,-20,-20,-15,-10,-5,0]; % deg

tf=repelem(1, length(P_ee)-1)'; % s
tf(5)=0.5;

if length(P_ee(1,:))-1 ~= length(tf)
    disp('You must specify the duration to each segment');
    return
end

% Setup
dt=0.1; % stepsize
[pos,vel,acc,tim]=via_points_match_VA(P_ee,tf,dt,'prescribed',[0,0]);

%% 3.1 Inverse Kinematics
% Inverse kinematics for all the points
joints = zeros(18, length(pos(1,:)));
for j=1:length(pos(1,:))
    % Solve the INVERSE KINEMATICS for the jth posture and store the
    % joint displacements in a (18x1)vector, say joints_j, which
    % includes the joint displacements for branches 1 through 6
    
    x0=pos(1,j); y0=pos(2,j); z0=pos(3,j); % mm
    ALPHA=pos(4,j); BETA=pos(5,j); GAMMA=pos(6,j); % deg gamma, beta, alpha
    
    % Find angles given the branch end positions/vertexes
    % Vectors
    P_ = [x0; y0; z0]; % position of platform
    A_ = zeros(6,3); % base center-to-edge vector
    C_ = zeros(6,3); % platfrom center-to-edge vector
    B_ = zeros(6,3); % prismatic vector
    D_ = zeros(6,3); % Platfrom vertex wrt to the origin
    Blen = zeros(6,1); % Joint lenghts
    theta1 = zeros(6,1); % angular displacement around y
    theta2 = zeros(6,1); % angular displacement around x
    
    % Rotation matrix of platfrom wrt base
    Rot_b0_p0 = rotmat(ALPHA, BETA, GAMMA);
    
    % Angles from each vertices on the base plate w/ respect to ba_ngle platform
    bAngle = [225; 315; 345; 75; 105; 195];
    % Angles from each vertices on the platform w/ respect to the platform
    pAngle = [255; 285; 15; 45; 135; 165];
    
    % Vector on ba_ngle plate to each corner, each row is different corner
    for i=1:NUM_JOINTS
        A_(i,:) = [bRadius*cosd(bAngle(i)), bRadius*sind(bAngle(i)), 0];
    end
    
    % Vector on platform to each corner, each row is different corner
    for i=1:NUM_JOINTS
        C_(i,:) = [pRadius*cosd(pAngle(i)), pRadius*sind(pAngle(i)), 0];
    end
    
    %% Calculate prismatic joint vector w/ respect to base platform origin
    % This will be the connecting vector from bi to pi
    for i=1:NUM_JOINTS
        Pos_b0_p0 = P_;
        Pos_b0_bi = transpose( A_(i,:) );
        Pos_p0_pi = transpose( C_(i,:) );
        Bi = Pos_b0_p0 + (Rot_b0_p0 * Pos_p0_pi) - Pos_b0_bi;
        B_(i,:) = [Bi(1), Bi(2), Bi(3)];
        pi = Pos_b0_p0 + (Rot_b0_p0 * Pos_p0_pi);
        D_(i,:) = [pi(1), pi(2), pi(3)];
        Blen(i,:) = norm(B_(i,:));
    end
    
    %% Determine angluar displacement for universal joints
    for i=1:NUM_JOINTS
        r_ = Blen(i);
        b_x = B_(i,1);
        b_y = B_(i,2);
        b_z = B_(i,3);
        
        c2 = b_y/r_;
        s2 = sqrt(1 - c2^2);
        c1 = b_z/(r_*s2);
        s1 = b_x/(r_*s2);
        
        theta1(i,:) = atan2d(s1, c1);
        theta2(i,:) = atan2d(s2, c2);
    end
    
    %% Create joint vector
    % joints_i =[theta1_1 theta2_1, L_1, theta1_2 theta2_2, L_2,... ,
    % theta1_6 theta2_6, L_6];
    % Store vectors in matrix form for each end-effector�s position
    joints_j = zeros(18,1);
    for i=1:3:length(joints_j)
        joints_j(i:i+2,1)=[theta1(ceil(i/3)), theta2(ceil(i/3)), Blen(ceil(i/3))];
    end
    
    %% Add joint vector the joints matrix
    joints(:,j) = joints_j;
    
end

clear pi Bi Pos_b0_p0 Pos_b0_bi Pos_p0_pi Rot_b0_p0;
clear c1 s1 c2 s2 r_ b_x b_y b_z al bt gm;
clear joints_j theta1 theta2 Blen;
clear x0 y0 z0 ALPHA BETA GAMMA A_ B_ C_ D_ Blen;

%% 3. Tragjectory Generation

% Prepare vairables for animation file
% ANIMATION (DO NOT CHANGE)
n=length(pos(1,:));
mov(1:length(n)) = struct('cdata', [],'colormap', []);
[a,b]=size(img); gifim=zeros(a,b,1,n-1,'uint8');

%% Animation of Platform
% Initialize variables
V_{NUM_PARTS} = [];
newV{NUM_PARTS} = [];

P0loc = zeros(3,length(joints(1,:)));
Blen = zeros(6, length(joints(1,:)));
for k=1:length(joints(1,:))
    % Extract the joint displacement values for each position from
    % the large matrix position.
    theta1i = joints(1:3:end,k);
    theta2i = joints(2:3:end,k);
    di = joints(3:3:end,k);
    
    %% Forward Kinematics
    Bi(:,1)=di.*sind(theta2i).*sind(theta1i);
    Bi(:,2)=di.*cosd(theta2i);
    Bi(:,3)=di.*sind(theta2i).*cosd(theta1i);
    for i=1:length(Bi(:,1))
        Blen(i,k) = norm(Bi(i,:));
    end
    % Base Joints Position
    Ai(:,1)=bRadius.*cosd(bAngle);
    Ai(:,2)=bRadius.*sind(bAngle);
    Ai(:,3)=repelem(0,6);
    % Vertexes wrt the base cemter
    Di(:,1)=bRadius.*cosd(bAngle) + Bi(:,1);
    Di(:,2)=bRadius.*sind(bAngle) + Bi(:,2);
    Di(:,3)=Bi(:,3);
    
    %% Position of Platform
    % Generate end effector positions
    P0 = [pos(1,k);pos(2,k);pos(3,k)];
    P0loc(:,k) = P0; 
    
    % Generate rotation matrix of P wrt b
    al=pos(4,k);
    bt=pos(5,k);
    gm=pos(6,k);
    
    Rbp=rotmat(al,bt,gm);
    
    %% Position platfrom
    % Moving Parts to where they belong
    % Rename of all the vertices. This step is redundant obj{}.V
    % will not longer be used.
    for i=1:NUM_PARTS
        V_{i} = obj{i}.V';
    end
    
    % Rotations
    for i=1:NUM_PARTS
        % Platform
        if i==1
            MAT = Rbp;
            % Platform Joints
        elseif sum(ismember(TOP_JOINT_1:TOP_JOINT_1+5,i))
            vec = [0;0;-1];
            MAT = findrot(vec,Bi(i-TOP_JOINT_1+1,:)');
            % Platform Links
        elseif sum(ismember(TOP_TUBE_1:TOP_TUBE_1+5,i))
            vec = [0;0;-1];
            MAT = findrot(vec,Bi(i-TOP_TUBE_1+1,:)');
            % Base Links
        elseif sum(ismember(BOTTOM_TUBE_1:BOTTOM_TUBE_1+5,i))
            vec = [0;0;1];
            MAT = findrot(vec,Bi(i-BOTTOM_TUBE_1+1,:)');
            % Base Joints
        elseif sum(ismember(BOTTOM_JOINT_1:BOTTOM_JOINT_1+5,i))
            vec = [0;0;1];
            MAT = findrot(vec,Bi(i-BOTTOM_JOINT_1+1,:)');
            % Base
        else
            al=0; bt=0; gm=0;
            MAT = rotmat(al, bt, gm);
        end
        newV{i} = MAT*V_{i};
    end
    
    % Translations
    for i=1:NUM_PARTS
        % Platform
        if i==1
            MAT = P0';
            % Platform Joints
        elseif sum(ismember(TOP_JOINT_1:TOP_JOINT_1+5,i))
            MAT = Di(i-TOP_JOINT_1+1,:);
            % Platform Links
        elseif sum(ismember(TOP_TUBE_1:TOP_TUBE_1+5,i))
            MAT = Di(i-TOP_TUBE_1+1,:);
            % Base Links
        elseif sum(ismember(BOTTOM_TUBE_1:BOTTOM_TUBE_1+5,i))
            MAT = Ai((i-BOTTOM_TUBE_1+1),:);
            % Base Joints
        elseif sum(ismember(BOTTOM_JOINT_1:BOTTOM_JOINT_1+5,i))
            MAT = Ai((i-BOTTOM_JOINT_1+1),:);
            % Base
        else
            MAT = [0,0,0];
        end
        newV{i} = newV{i} + repmat(MAT',[1 length(V_{i}(1,:))]);
    end
    
    % Set the new position in the handle (graphical link)
    for i=1:NUM_PARTS
        set(q_(i),'Vertices',newV{i}(1:3,:)');
    end
    
    drawnow
    
    %% Animation Frame
    im = frame2im(getframe);
    gifim(:,:,:,k)=rgb2ind(im, map);
    mov(k)=getframe(gcf);
end % close loop k

clear v1 v2 P0 Rbp temp Ai Bi Di;
clear al bt gm MAT;

%% ANIMATION
% Create animated gif (DO NOT MODIFY)
imwrite(gifim,map,'OUT/MSE429Project.gif','DelayTime',0); % 'LoopCount',inf)

%% Draw Plots
plot_end_effector(P0loc)
kinematic_plots(pos,vel,acc,tim,joints);
Jacobian(pos,Blen,vel,tim);

end

function plot_end_effector(P0loc)
    eePlot = figure;
    set(eePlot, 'Position', [0 0 2736 1604]);
    comet3(P0loc(1,:), P0loc(2,:), P0loc(3,:));
    filepath = fullfile('OUT','images','ee-plot.png');
    saveas(eePlot, filepath, 'png');
end

function kinematic_plots(pos,vel,acc,tim,joints)
%% Plot end effector graphs
eeGraphs = figure;
set(eeGraphs, 'Position', [0 0 2736 1604])
displacement = zeros(1, length(pos(1,:)));
velocity = zeros(1, length(pos(1,:)));
acceleration = zeros(1, length(pos(1,:)));
time = zeros(1, length(pos(1,:)));
for i=1:length(pos(1,:))
    displacement(i) = norm([pos(1,i);pos(2,i);pos(3,i)]);
    velocity(i) = norm([vel(1,i);vel(2,i);vel(3,i)]);
    acceleration(i) = norm([acc(1,i);acc(2,i);acc(3,i)]);
    time(i) = norm([tim(1,i);tim(2,i);tim(3,i)]);
end
% Displacement
subplot(3,1,1);
plot(time, displacement);
% Velocity
subplot(3,1,2);
plot(time, velocity);
% Acceleration
subplot(3,1,3);
plot(time, acceleration);
%Plot Labels
subplot(3,1,1);
xlabel('Time');
ylabel('Displacement (mm)');
title('End Effector Displacement');
subplot(3,1,2);
xlabel('Time');
ylabel('Velocity (m/s)');
title('End Effector Velocity');
subplot(3,1,3);
xlabel('Time');
ylabel('Acceleration (m/s^2)');
title('End Effector Acceleration');
filepath = fullfile('OUT','images','ee-graphs.png');
saveas(eeGraphs, filepath,'png');

%% Plot Joint Displacements
JointDisplacements = figure;
set(JointDisplacements, 'Position', [0 0 2736 1604])
for i=1:3:length(joints(:,1))
    link = ceil(i/3);
    time = tim(link,:);
    theta1s = joints(i,:);
    theta2s = joints(i+1,:);
    ds = joints(i+2,:);
    % Angular displacement plot
    subplot(1,2,1);
    hold on
    plot(time, theta1s);
    hold on
    plot(time, theta2s);
    % Linear displacement plot
    subplot(1,2,2);
    hold on
    plot(time, ds);
end
% Labels
subplot(1,2,1);
xlabel('Time');
ylabel('Displacement (deg)');
legend({'Joint \theta1_1',...
    'Joint \theta2_1',...
    'Joint \theta1_2',...
    'Joint \theta2_2',...
    'Joint \theta1_3',...
    'Joint \theta2_3',...
    'Joint \theta1_4',...
    'Joint \theta2_4',...
    'Joint \theta1_5',...
    'Joint \theta2_5',...
    'Joint \theta1_6',...
    'Joint \theta2_6'},...
    'Location','southeast');
title('Angular Displacements');

subplot(1,2,2);
xlabel('Time');
ylabel('Displacement (mm)');
legend({
    'Joint d_1',...
    'Joint d_2',...
    'Joint d_3',...
    'Joint d_4',...
    'Joint d_5',...
    'Joint d_6',...
    },...
    'Location', 'northeast');
title('Linear Displacements');
filepath = fullfile('OUT','images','joint-displacements.png');
saveas(JointDisplacements, filepath,'png');

end

function Jacobian(pos,B_len,vel,tim)
%% Build Jacobian
%pos saves all positions of end effector
%vel saves all velocities of end effector
syms Px_jon Py_jon Pz_jon alpha_jon beta_jon gamma_jon L1 L2 L3 L4 L5 L6 

%---------information redundant, don't have enough time to fix-------------
b = 2;
p = 1;
btheta = [225, 315, 345, 75, 105, 195];
ptheta = [255, 285, 15, 45, 135, 165];
for i = 1:6
    P_b0_b_jon(:,i) = [b*cosd(btheta(1,i)) b*sind(btheta(1,i)) 0]; %A vectors
end   
for i = 1:6
    P_p0_p_jon(:,i) = [p*cosd(ptheta(1,i)) p*sind(ptheta(1,i)) 0]; %C vectors
end

R0_p0=[    cosd(beta_jon)*cosd(gamma_jon)                                 ,       -sind(beta_jon)      , cosd(beta_jon)*sind(gamma_jon);                               
       cosd(alpha_jon)*sind(beta_jon)*cosd(gamma_jon)+sind(alpha_jon)*sind(gamma_jon) , cosd(alpha_jon)*cosd(beta_jon) , cosd(alpha_jon)*sind(beta_jon)*sind(gamma_jon)-sind(alpha_jon)*cosd(gamma_jon);
       sind(alpha_jon)*sind(beta_jon)*cosd(gamma_jon)-cosd(alpha_jon)*sind(gamma_jon) , sind(alpha_jon)*cosd(beta_jon) , sind(alpha_jon)*sind(beta_jon)*sind(gamma_jon)+cosd(alpha_jon)*cosd(gamma_jon)];
%--------------------------------------------------------------------------
P_b0_p0=[Px_jon; Py_jon; Pz_jon];  %position vector 

for i = 1:6
    P_p0_pi = P_b0_b_jon(:,i); %A vector
    P_b0_bi = P_p0_p_jon(:,i); %C vector
    P_bi_pi=simplify(sum(expand((P_b0_p0+R0_p0*P_p0_pi-P_b0_bi).^2))); %g(x)
    Jx(i,1) = diff(P_bi_pi,Px_jon);
    Jx(i,2) = diff(P_bi_pi,Py_jon);
    Jx(i,3) = diff(P_bi_pi,Pz_jon);
    Jx(i,4) = diff(P_bi_pi,alpha_jon);
    Jx(i,5) = diff(P_bi_pi,beta_jon);
    Jx(i,6) = diff(P_bi_pi,gamma_jon);
end

Jq = 2*[L1, 0, 0, 0, 0, 0;
        0, L2, 0, 0, 0, 0;
        0, 0, L3, 0, 0, 0;
        0, 0, 0, L4, 0, 0;
        0, 0, 0, 0, L5, 0;
        0, 0, 0, 0, 0, L6];
    
 %creating Jacobians
    jacobian_x = zeros(6,6,length(pos(1,:))); %length(pos(1,:)) is the size of the columns therefore 6x6x99
    jacobian_q = zeros(6,6,length(pos(1,:)));
    jacobian = zeros(6,6,length(pos(1,:)));
    %determinates
    d_jx = zeros(1,length(pos(1,:)));
    d_jq = zeros(1,length(pos(1,:)));
    d_j = zeros(1,length(pos(1,:)));
    qdot = zeros(6,length(pos(1,:)));
    torque = zeros(6,length(pos(1,:)));
    transj = zeros(6,6,length(pos(1,:)));

    mass = 2717.39; %kg
    weight = mass*9.81;
    force = [0; 0; -weight; 0; 0; 0];
%% calculate velocities and torques of the prismatic joints
    for jj = 1:length(pos(1,:))
        %fills in Jx6 for jacobian_x
        Jx1 = subs(Jx,  Px_jon,     pos(1,jj));
        Jx2 = subs(Jx1, Py_jon,     pos(2,jj));
        Jx3 = subs(Jx2, Pz_jon,     pos(3,jj));
        Jx4 = subs(Jx3, alpha_jon,  pos(4,jj));
        Jx5 = subs(Jx4, beta_jon,   pos(5,jj));
        Jx6 = subs(Jx5, gamma_jon,  pos(6,jj));

        jacobian_x(:,:,jj) = double(Jx6);
        d_jx(1,jj) = det(jacobian_x(:,:,jj));

        %generating all Jq
        Jq1 = subs(Jq,  L1, B_len(1,jj));
        Jq2 = subs(Jq1, L2, B_len(2,jj));
        Jq3 = subs(Jq2, L3, B_len(3,jj));
        Jq4 = subs(Jq3, L4, B_len(4,jj));
        Jq5 = subs(Jq4, L5, B_len(5,jj));
        Jq6 = subs(Jq5, L6, B_len(6,jj));

        jacobian_q(:,:,jj) = double(Jq6);
        d_jq(1,jj) = det(jacobian_q(:,:,jj));

        jacobian(:,:,jj) = inv(jacobian_q(:,:,jj))*jacobian_x(:,:,jj);
        %velocity and force calculations

        qdot(:,jj) = jacobian(:,:,jj)*vel(:,jj);

        %force calculation
        transj(:,:,jj) = transpose(jacobian(:,:,jj));
        torque(:,jj) = transj(:,:,jj)*force;
        d_j(1,jj) = det(jacobian(:,:,jj));
    end

    %% Plot Torques
    torPlot = figure;
    set(torPlot, 'Position', [0 0 2736 1604]);
    for i=1:length(torque(:,1))
        subplot(1,2,1)
        hold on
        plot(tim(i,:), torque(i,:));
        subplot(1,2,2)
        hold on
        plot(tim(i,:), qdot(i,:));
    end
    subplot(1,2,1);
    xlabel('Time');
    ylabel('Displacement (mm)');
    legend({
        'Joint L_1',...
        'Joint L_2',...
        'Joint L_3',...
        'Joint L_4',...
        'Joint L_5',...
        'Joint L_6',...
        },...
        'Location', 'northeast');
    title('Torques');
    subplot(1,2,2);
    xlabel('Time');
    ylabel('Displacement (mm)');
    legend({
        'Joint L_1',...
        'Joint L_2',...
        'Joint L_3',...
        'Joint L_4',...
        'Joint L_5',...
        'Joint L_6',...
        },...
        'Location', 'northeast');
    title('Joint Velocities');

    filepath = fullfile('OUT','images','tor-plot.png');
    saveas(torPlot, filepath, 'png');
end