%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%ECE 5553 - Autonomy in Vehicles
%%HW 4 - Path Following Linear Model
%%Spring 2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Initialize the variables
Xp = [];
Yp = [];

%%%%%%%
%DOUBLE LANE CHANGE MANOEUVER - ISO 3888-1
%%%%%%%
Veh_width = 1.57; %For a Mid-size Sedan Vehicle, width in m
L1 = Veh_width*1.1+0.15;
L2 = Veh_width*1.2+0.15;
L3 = Veh_width*1.3+0.15;

p = []; %[x1 y1;x2 y2;x3 y3;...;xn yn]

%Change the points here to modify the shape of the curve.
%If you are adding new points, make sure you modify the p & p_ub_pts array
%points accordingly
p_lb_pts = [0,0; 15,0; 15,0; 45,3.5; 53,3.5; 63,3.5; 70,3.5; 95,0; 95,0; 125,0;]; %Points on the curve considering 0,0 starting point and no offset L1,L2,L3



p_ub_pts = p_lb_pts;
p = p_lb_pts;

%Upper Bound
p_ub_pts(1:3,2) = p_lb_pts(1:3,2)+(L1); %Offset of the path. upper bound
p_ub_pts(4:7,2) = p_lb_pts(4:7,2)+(L2); %Offset of the path. upper bound
p_ub_pts(8:10,2) = p_lb_pts(8:10,2)+(L3); %Offset of the path. upper bound

%Mid-path
p(1:3,2) = p_lb_pts(1:3,2)+(L1/2); %Offset of the path to the centre of the vehicle
p(4:7,2) = p_lb_pts(4:7,2)+(L2/2); %Offset of the path to the centre of the vehicle
p(8:10,2) = p_lb_pts(8:10,2)+(L3/2); %Offset of the path to the centre of the vehicle

%Split the points into three bezier curves%

%Beizer Curves passes through first and last points and approximates
%through the middle points. This is the characteristic of the Beizer curve
p1 = p(1:3,:); 
p2 = p(3:8,:);
p3 = p(8:10,:);


[Xp_1,Yp_1] = bezier_curve(p1); %For first curve
[Xp_2,Yp_2] = bezier_curve(p2); %For second curve
[Xp_3,Yp_3] = bezier_curve(p3); %For third curve

Xp = [Xp_1(1:end-1); Xp_2(1:end-1); Xp_3(1:end-1);]; 
Yp = [Yp_1(1:end-1); Yp_2(1:end-1); Yp_3(1:end-1);];

figure(1001);
subplot(1,1,1)
plot(Xp_1(1:end-1),Yp_1(1:end-1),'Linewidth',2); hold on;
plot(Xp_2(1:end-1),Yp_2(1:end-1),'Linewidth',2); hold on;
plot(Xp_3(1:end-1),Yp_3(1:end-1),'Linewidth',2); hold on;

plot(p(:,1),p(:,2),'k--','Linewidth',2); hold on;  %Middle path
plot(p_ub_pts(:,1),p_ub_pts(:,2),'x'); hold on;  %Upper points
plot(p_lb_pts(:,1),p_lb_pts(:,2),'o'); hold on; %Lower points

xlabel('X Position [m]')
ylabel('Y Position [m]')
axis([0 125 0 6])
title('Double Lane Change Path')

%Road Curvature Calculations

no_of_pts = length(Xp);
rho_ref = zeros(length(Xp)-1,1); %Create a Zero Vector for initialization

%Compute the Differentiation terms
del_1_Xp = diff(Xp);
del_1_Yp = diff(Yp);
del_2_Xp = diff(Xp,2); %Second order difference
del_2_Yp = diff(Yp,2); %Second order difference
del_pts = 1; %Difference in the index;

%Curvature Formula given in Lecture 21, Slide 27
rho_new = (((del_1_Xp(2:end)/del_pts).*(del_2_Yp/del_pts.^2))-((del_1_Yp(2:end)/del_pts).*((del_2_Xp/del_pts.^2))))./((del_1_Xp(2:end)./del_pts).^2+((del_1_Yp(2:end)./del_pts).^2)).^1.5;

%Calculation of total distance travelled for the lookuo table - xaxis input
Tot_d = 0;
for j = 1:no_of_pts-1
    d(j) = sqrt((Xp(j+1)-Xp(j))^2+(Yp(j+1)-Yp(j))^2);
    Tot_d = Tot_d+d(j);
    Cum_dis(j) = Tot_d; %Cumulative distance
end

dis_vec = Cum_dis(1:end-1); %Distance vector for the lookup table in the model
t_end = Tot_d/V; %for simulation time
