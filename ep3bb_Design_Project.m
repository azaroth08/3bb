function varargout = ep3bb_Design_Project(varargin)


% Begin initialization code - DO NOT EDIT
clc; 
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ep3bb_Design_Project_OpeningFcn, ...
                   'gui_OutputFcn',  @ep3bb_Design_Project_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT



% --- Executes just before ep3bb_Design_Project is made visible.
function ep3bb_Design_Project_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ep3bb_Design_Project (see VARARGIN)

% Choose default command line output for ep3bb_Design_Project
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes ep3bb_Design_Project wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = ep3bb_Design_Project_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in start.
function start_Callback(hObject, eventdata, handles)
global run
global set
setpoint = 25;
run=1;
i=2;
global proportional_term
global integral_term
global derivative_term
proportional_term = 0.1; %initialize the PID values
integral_term = 0;
derivative_term = 0;
h = 0.001; %This is to calculate the derivative (error - previous error)/h is the derivative 
text = fopen('new.txt','a');
fmt = '%5d \n'; %test text file for diagnostic purposes
%Determine the optimal values for the proportional, integral and derivative 
%terms via experimentation 
old_pos = 280; %original position of the motor
comport = serial('COM3', 'baud', 155300, 'FlowControl','none');
fopen(comport);
fwrite(comport, old_pos/3);
while (run==1)
   clock_input = fread(comport,1,'uint16');

   x(i) = ((clock_input)/2000000*34300/2); %current position of the ball 
   err(i) = x(i)-setpoint; %calculates error from position of ball
   if (x==2) %initializes x so derivative term stays within bounds
       x(1) = x(2);
       err(1) = err(2);
   end
   last_err = err(end-100+1:end); %last_err is the vector array that stores the last 100 error values 
   P_out = proportional_term*err(i);
   I_out = integral_term*trapz(last_err); 
   D_out = derivative_term*((err(i)-err(i-1))/h); 
   
   plot(2:i,err(2:i));
   ylim([-25 25]);
   new_pos = old_pos +P_out+I_out +D_out; %new position of ball 
   fprintf(text,fmt,new_pos); %diagnostics text file write
   %compares the new_pos to the limits and changes it to match the absolute
   %limits. limits values may need to be updated with number of steps. 
   i=i+1;   
   if (new_pos > 300)
       new_pos = 300;
   elseif (new_pos <230)
       new_pos = 230;
   end
   old_pos = new_pos; 
   fwrite(comport, new_pos/3);
   drawnow
end
%plot(i,x)
fclose(comport);
fclose(text);

% --- Executes on button press in stop.
function stop_Callback(hObject, eventdata, handles)
global run;
run=0;


% --- Executes on button press in quit.
function quit_Callback(hObject, eventdata, handles)
global run
run=0;
fclose(instrfind);
close all;



function middle_Callback(hObject, eventdata, handles)
% hObject    handle to middle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of middle as text
%        str2double(get(hObject,'String')) returns contents of middle as a
%        double
global set
set=str2double(get(hObject,'String'));

% --- Executes during object creation, after setting all properties.
function middle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to middle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function proportional_Callback(hObject, eventdata, handles)
% hObject    handle to middle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of middle as text
%        str2double(get(hObject,'String')) returns contents of middle as a
%        double
global proportional_term
proportional_term=str2double(get(hObject,'String'));

% --- Executes during object creation, after setting all properties.
function proportional_CreateFcn(hObject, eventdata, handles)
% hObject    handle to proportional (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function integral_Callback(hObject, eventdata, handles)
% hObject    handle to integral (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of integral as text
%        str2double(get(hObject,'String')) returns contents of integral as a double
global integral_term
integral_term=str2double(get(hObject,'String'));

% --- Executes during object creation, after setting all properties.
function integral_CreateFcn(hObject, eventdata, handles)
% hObject    handle to integral (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function derivative_Callback(hObject, eventdata, handles)
% hObject    handle to derivative (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of derivative as text
%        str2double(get(hObject,'String')) returns contents of derivative as a double
global derivative_term
derivative_term=str2double(get(hObject,'String'));

% --- Executes during object creation, after setting all properties.
function derivative_CreateFcn(hObject, eventdata, handles)
% hObject    handle to derivative (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
