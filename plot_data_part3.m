% # read joint values and plot the data
clc
clear
close all
rosshutdown

ipaddress = 'localhost';
rosinit(ipaddress);

joint_states = rossubscriber('/custom_scara/joint_states');

joint1_rec = [];
joint2_rec = [];
joint3_rec = [];
rate = robotics.Rate(30);
record_time = 10;
reset(rate);

joint_init = receive(joint_states);
joint1_old = joint_init.Position(1);
joint2_old = joint_init.Position(2);
joint3_old = joint_init.Position(3);
t_old = 0;

while rate.TotalElapsedTime < record_time
    t = rate.TotalElapsedTime;
    dt = t - t_old;
    joint_update = receive(joint_states);
    joint1_curr = joint_update.Position(1);
    joint2_curr = joint_update.Position(2);
    joint3_curr = joint_update.Position(3);
    joint1_rec = [joint1_rec, (joint1_curr - joint1_old)/dt];
    joint2_rec = [joint2_rec, (joint2_curr - joint2_old)/dt];
    joint3_rec = [joint3_rec, (joint3_curr - joint3_old)/dt];
    disp('recording ...');
    joint1_old = joint1_curr;
    joint2_old = joint2_curr;
    joint3_old = joint3_curr;
    t_old = t;
    waitfor(rate);
end

disp('complete');

%% plotting

t = linspace(0,record_time,length(joint3_rec));
ref1 = 0.05*ones(1,length(joint1_rec));
ref2 = 0.05*ones(1,length(joint2_rec));
ref3 = 0.05*ones(1,length(joint3_rec));

subplot(3,1,1)
plot(t,joint1_rec,'b','LineWidth',1);
hold on
plot(t,ref1,'-.m','LineWidth',1);

subplot(3,1,2)
plot(t,joint2_rec,'b','LineWidth',1);
hold on
plot(t,ref2,'-.m','LineWidth',1);

subplot(3,1,3)
plot(t,joint3_rec,'b','LineWidth',1);
hold on
plot(t,ref,'-.m','LineWidth',1);

grid on
xlabel('time [sec]');
ylabel('joint velocities [rad/s] & [m/s]');
legend('actual joint velocity','desired joint velocity');
