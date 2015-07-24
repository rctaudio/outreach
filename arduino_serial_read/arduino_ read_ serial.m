% Basic Arduino and Matlab
% Communication with an external hardware device
% ———————————————-
%
% Erin Kennedy
% Jan. 18, 2010
%
 
clear all; clc; close all;
delete(instrfindall);
    
% Initialize serial port

s = serial('COM24');

%set(s, ‘ Terminator’, ‘LF’); % Default terminator is \n
set(s,'BaudRate', 115200);
set(s,'DataBits', 8);
set(s,'StopBits', 1);
fopen(s);
s.ReadAsyncMode = 'continuous';

% Various variables
numberOfDatas = 1000;
data = zeros(1,3, numberOfDatas);
i = 1;
 
% Start asynchronous reading
readasync(s);
 
 
while(i<=numberOfDatas)  
   
    % Get the data from the serial object
    data(:,:,i) = fscanf(s, '%f');
    data(:,:,i)
    i=i+1;
   
end
    
stopasync(s);
fclose(s); % bad
delete(instrfindall);