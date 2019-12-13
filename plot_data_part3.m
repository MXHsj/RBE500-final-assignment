% # read joint values and plot the data
clc
clear
close all
rosshutdown

ipaddress = 'localhost';
rosinit(ipaddress);

% joint_states = rossubscriber('/custom_scara/joint_states');
link_states = rossubscriber('/gazebo/link_states');
vx_rec = [];
vy_rec = [];
vz_rec = [];
rate = robotics.Rate(30);
record_time = 10;
reset(rate);

link_init = receive(link_states);
x_old = link_init.Pose(5,1).Position.X;
y_old = link_init.Pose(5,1).Position.Y;
z_old = link_init.Pose(5,1).Position.Z;
t_old = 0;

while rate.TotalElapsedTime < record_time
    t = rate.TotalElapsedTime;
    dt = t - t_old;
    link_update = receive(link_states);
    x_curr = link_update.Pose(5,1).Position.X;
    y_curr = link_update.Pose(5,1).Position.Y;
    z_curr = link_update.Pose(5,1).Position.Z;
    
    vx_rec = [vx_rec, (x_curr - x_old)/dt];
    vy_rec = [vy_rec, (y_curr - y_old)/dt];
    vz_rec = [vz_rec, (z_curr - z_old)/dt];
    disp('recording ...');
    x_old = x_curr;
    y_old = y_curr;
    z_old = z_curr;
    t_old = t;
    waitfor(rate);
end

disp('complete');

%% plotting

t = linspace(0,record_time,length(vx_rec));
ref1 = 0.0*ones(1,length(vx_rec));
ref2 = 0.1*ones(1,length(vy_rec));
ref3 = 0.0*ones(1,length(vz_rec));

subplot(3,1,1)
plot(t,vx_rec,'b','LineWidth',1);
hold on
plot(t,ref1,'-.m','LineWidth',1);
grid on
xlabel('time [sec]');
ylabel('vx [m/s]');
legend('actual x direction velocity','desired x direction velocity');

subplot(3,1,2)
plot(t,vy_rec,'b','LineWidth',1);
hold on
plot(t,ref2,'-.m','LineWidth',1);
grid on
xlabel('time [sec]');
ylabel('vy [m/s]');
legend('actual y direction velocity','desired y direction velocity');

subplot(3,1,3)
plot(t,vz_rec,'b','LineWidth',1);
hold on
plot(t,ref3,'-.m','LineWidth',1);
grid on
xlabel('time [sec]');
ylabel('vz [m/s]');
legend('actual z direction velocity','desired z direction velocity');



