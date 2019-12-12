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
rate = robotics.Rate(20);
record_time = 10;
reset(rate);

joint_init = receive(joint_states);
joint1_init = joint_init.Position(1);
joint2_init = joint_init.Position(2);
joint3_init = joint_init.Position(3);

while rate.TotalElapsedTime < record_time
    t = rate.TotalElapsedTime;
    joint_update = receive(joint_states);
    joint1_rec = [joint3_rec, (joint_update.Position(3) - joint3_init)/t];
    joint2_rec = [joint3_rec, (joint_update.Position(3) - joint3_init)/t];
    joint3_rec = [joint3_rec, (joint_update.Position(3) - joint3_init)/t];
    disp('recording ...');
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
