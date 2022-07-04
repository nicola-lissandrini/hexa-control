%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Initialize GenoM stuff and connect the ports %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
Initialization
disp('Initializing system');

%% Generalrinit.log_time_step
init.time_step = 0.005;
init.log_time_step = 0.01;

%% Genomix client
openrobots_dir = getenv('ROBO*/**TPKG_BASE');
client = genomix.client();
client.rpath([openrobots_dir,strcat(openrobots_dir,'/lib/genom/pocolibs/plugins/')]);

clear ans openrobots_dir

%% Mikrocopter
usb_port_mikro = '/dev/ttyUSB0';
baud_rate_serial = 500000;
rate_imu = 1/init.time_step;
rate_esc = 0;
rate_battery = 1;

disp('* Initialize Mikrocopter');
mikro = client.load('rotorcraft'); % mikro = client.load('mikrokopter');
pause(2);
result = mikro.connect(usb_port_mikro, baud_rate_serial); %mikro.connect({usb_port_mikro, ''},baud_rate_serial);
string = ['Connecting to mikrokopter: ',result.status];
disp(string);
mikro.set_sensor_rate(rate_imu, 0, rate_esc, rate_battery);
%%
disp('* Initialize Joystick')
joystick = client.load('joystick');


%% Vicon
disp('* Initialize Vicon');
vicon = client.load('vicon');
pause(2);
result = vicon.connect('192.168.1.18');% optitrack.connect('marey.laas.fr','1510','239.192.168.30','1511');
string = ['Connecting to MoCap: ',result.status];
pause(2);
disp(string);


% %% State estimation
pom = client.load('pom');
pause(1);
result = pom.connect_port('measure/imu', 'rotorcraft/imu'); 
string = ['Initializing 1st connection of POM: ',result.status];
pom.add_measurement('imu',0,0,0,0,0,0);
disp(string);
%
result = pom.connect_port('measure/mocap', 'vicon/bodies/Hexa');
 pause(1);
 string = ['Initializing 2nd connection of POM: ',result.status];
 pom.add_measurement('vicon',0,0,0,0,0,0);
 disp(string);

mikro.set_timeout(250);

clear result string


