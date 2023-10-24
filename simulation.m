clc; clear; close all;

try
    setenv('ROS_MASTER_URI', 'http://10.70.192.13:11311')
    setenv('ROS_IP', '10.1.8.167')
    rosinit
catch
end
    
%% Defining Server/Client
serviceName = '/start_simulation';
client = rossvcclient('/start_simulation');
requestMessage = rosmessage('cart_pole/LaunchParametersRequest');

%% Define the parameter
mc = 1.5; mp = 0.5;
g = 9.81; L = 1.0;
d1 = 0; d2 = 0;
Iw = 0.00125; r = 0.2;

A = [0 0 1 0;
     0 0 0 1;
     0 -L*g*mp*r^2/(-2*Iw*L-L*mc*r^2) -d1*r/(-2*Iw-mc*r^2) -d2*r^2/(-2*Iw*L-L*mc*r^2);
     0 L*g*mp*(-2*Iw-mc*r^2-mp*r^2)/(-2*Iw*L^2*mp-L^2*mc*mp*r^2) -d1*r/(-2*Iw*L-L*mc*r^2) d2*(-2*Iw-mc*r^2-mp*r^2)/(-2*Iw*L^2*mp-L^2*mc*mp*r^2)];


B = [0;
     0;
    -r/(-2*Iw-mc*r^2)
    -r/(-2*Iw*L-L*mc*r^2)];

C = eye(4);
D = [0; 0; 0; 0];

Ts = 0.02;
Q = eye(4);
R = 1;

K = lqrd(A, B, Q, R, Ts);

%% Defining global variable state and subscriber 
global state
state = [];
sub = rossubscriber('/state', 'std_msgs/Float64MultiArray', @state_callback);
pause(2);

%% Calling the service
requestMessage.Mc = mc;
requestMessage.Mp = mp;
requestMessage.L = L;
requestMessage.K1 = K(1);
requestMessage.K2 = K(2);
requestMessage.K3 = K(3);
requestMessage.K4 = K(4);

asyncServiceCall = @(serviceName, requestMessage) callServiceAsync(serviceName, requestMessage);
asyncServiceCall(serviceName, requestMessage);

%% callback function for subscriber
function state_callback(~, msg)
    global state
                %    t           x           phi          xdot        phidot
    new_state = [msg.Data(1)/1000, msg.Data(2), msg.Data(3), msg.Data(4), msg.Data(5), msg.Data(6)];
    state = [state; new_state];
end

%% function to call the service Async
function callServiceAsync(serviceName, requestMessage)
    client = rossvcclient(serviceName);
    call(client, requestMessage);
    clear client;
end