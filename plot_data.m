% # read joint values and plot the data

clc
clear
close all
rosshutdown

ipaddress = 'localhost';
rosinit(ipaddress);

joint_states = rossubscriber('/custom_scara/joint_states');

joint3 = [];
rate = robotics.Rate(20);
record_time = 8;
reset(rate);

while rate.TotalElapsedTime < record_time
    joint_update = receive(joint_states);
    joint3 = [joint3, joint_update.Position(3)];
    disp('recording ...');
    waitfor(rate);
end

disp('complete');

%%
t = linspace(0,record_time,length(joint3));
ref = 0.05*ones(1,length(joint3));
plot(t,joint3,'b','LineWidth',1);
hold on
plot(t,ref,'-.m','LineWidth',1);
grid on
xlabel('time [sec]');
ylabel('joint3 position [rad]');
legend('actual joint pose','desired joint pose');




