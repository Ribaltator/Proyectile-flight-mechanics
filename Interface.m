clear all
%% Launch parameterrs
pos0=[0 0 500];%[m]
theta=0;%[rad] Direction angle
phi=pi/4;%[rad] Elevation angle
arg_v0=300;%[m/s]


%% Initial conditions
g0=9.81;%[m/s^2]
rho0=1.225;%[kg/m^3]
S_ref=0.5;%[m^2]
Cd_0=0.2;%[-]
mass=8;%[kg]


%Simulator data
v0x=arg_v0*cos(phi)*cos(theta);
v0y=arg_v0*cos(phi)*sin(theta);
v0z=arg_v0*sin(phi);
v0=[v0x v0y v0z];

x_dir=[1 0 0];
y_dir=[0 1 0];
z_dir=[0 0 1];
z0=pos0(3);

%% Results

out=sim(Simulator);

out.x=zeros(length(out.pos),1);
out.y=zeros(length(out.pos),1);
out.z=zeros(length(out.pos),1);
for i=1:length(out.pos)
    out.x(i)=dot(out.pos(i,:),x_dir);
    out.y(i)=dot(out.pos(i,:),y_dir);
    out.z(i)=dot(out.pos(i,:),z_dir);
end


figure(1)
subplot(1,3,1)
plot(out.t, out.x), hold on
xlabel('t (s)')
ylabel('X (m)')
grid
hold on
subplot(1,3,2)
plot(out.t, out.y), hold on
xlabel('t (s)')
ylabel('Y (m)')
grid
hold on
subplot(1,3,3)
plot(out.t, out.z), hold on
xlabel('t (s)')
ylabel('Z (m)')
grid
hold on

figure(2)
subplot(1,3,1)
plot(out.x, out.z), hold on
xlabel('X (m)')
ylabel('Z (m)')
axis equal
grid
hold on
subplot(1,3,2)
plot(out.y, out.z), hold on
xlabel('Y (m)')
ylabel('Z (m)')
axis equal
grid
hold on
subplot(1,3,3)
plot(out.x, out.y), hold on
xlabel('X (m)')
ylabel('Y (m)')
axis equal
grid
hold on

figure(3)
plot3(out.x,out.y,out.z), hold on
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
axis equal
hold on

figure(4)
comet3(out.x,out.y,out.z), hold on
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
axis equal
