function varargout = GUI(varargin)
% GUI MATLAB code for GUI.fig
%      GUI, by itself, creates a new GUI or raises the existing
%      singleton*.
%
%      H = GUI returns the handle to a new GUI or the handle to
%      the existing singleton*.
%
%      GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI.M with the given input arguments.
%
%      GUI('Property','Value',...) creates a new GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI

% Last Modified by GUIDE v2.5 26-Nov-2020 14:17:09

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_OutputFcn, ...
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

% x variable to store image after editing
function setGlobalx(val)
global x
x = val;

function r = getGlobalx
global x
r = x;

% y variable to store the value of amount text box  
function setGlobaly(val)
global y
y = val;

function r = getGlobaly
global y
r = y;

% display the image with the histogram side by side
function display_image(handles, a)
axes(handles.axes1);
imshow(a);
axes(handles.axes2);
imhist(a);
setGlobalx(a)

% --- Executes just before GUI is made visible.
function GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI (see VARARGIN)

% Choose default command line output for GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in filters_menu.
function filters_menu_Callback(hObject, eventdata, handles)
% hObject    handle to filters_menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

allItems = handles.filters_menu.String;
selectedIndex = handles.filters_menu.Value;
selectedItem = allItems{selectedIndex};
switch selectedItem
    case char('Median')
        a = getGlobalx()
        a = medfilt2(a, [3 3]);
        display_image(handles, a)
    case char('Average')
        a = getGlobalx()
        x = fspecial('average',[3 3])
        a = imfilter(a,x,'replicate')
        display_image(handles, a)
    case char('Gaussian')
        a = getGlobalx()
        a = imgaussfilt(a,2)
        display_image(handles, a)
    case char('Laplacian')
        a = getGlobalx()
        x = fspecial('laplacian',0.2)
        a = imfilter(a,x,'replicate')
        display_image(handles, a)
                 
    case char('Equalization')
        a = getGlobalx()
        a = histeq(a)
        display_image(handles, a)
    
    case char('Max')
        a = getGlobalx()
        maxf = @(x)max(x(:));
        a = nlfilter(a,[3 3], maxf);
        display_image(handles, a)
        
    case char('Min')
        a = getGlobalx()
        minf = @(x)min(x(:));
        a = nlfilter(a,[3 3], minf);
        display_image(handles, a)
    
    case char('sharp')
        a = getGlobalx()
        a = imsharpen(a,'Radius',2,'Amount',1);
        display_image(handles, a)
        
    case char('Unsharp')
        a = getGlobalx()
        sharp = imsharpen(a,'Radius',2,'Amount',1);
        dif = sharp - a
        a = a + dif
        display_image(handles, a)
    
        
    case char('Midpoint')
        a = getGlobalx()
        minf = @(x)min(x(:));
        minf = nlfilter(a,[3 3], minf);
        maxf = @(x)max(x(:));
        maxf = nlfilter(a,[3 3], maxf);
        midf = 0.5*(maxf+minf)
        display_image(handles, midf)
    
    case char('Low_pass')
        a = getGlobalx()
        A = fft2(double(a)); % compute FFT of the grey image
        Y=fftshift(A); % frequency scaling
        [M N]=size(A); % image size
        D=30;
        Lo(1:M,1:N)=0;
        Lo(0.5*M-D:0.5*M+D,0.5*N-D:0.5*N+D)=1;
        J=Y.*Lo;
        J1=ifftshift(J);
        B1=ifft2(J1);
        axes(handles.axes1)
        imshow(abs(B1),[12 290])
        setGlobalx(B2)
    case char('High_pass')
        a = getGlobalx()
        A = fft2(double(a)); % compute FFT of the grey image
        Y=fftshift(A); % frequency scaling
        [M N]=size(A); % image size
        D=30; % filter size parameter
        Hi(1:M,1:N)=1;
        Hi(0.5*M-D:0.5*M+D,0.5*N-D:0.5*N+D)=0;
        K=Y.*Hi;
        K1=ifftshift(K);
        B2=ifft2(K1);
        axes(handles.axes1)
        imshow(abs(B2),[12 290])
        setGlobalx(B2)
       
   
end
% Hints: contents = cellstr(get(hObject,'String')) returns filters_menu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from filters_menu


% --- Executes during object creation, after setting all properties.
function filters_menu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to filters_menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
    

end


% --- Executes on button press in upload_img.
function upload_img_Callback(hObject, eventdata, handles)
% hObject    handle to upload_img (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
a=uigetfile('.jpg')
a=imread(a);
axes(handles.axes1);
imshow(a);
axes(handles.axes2);
imhist(a);
setGlobalx(a)
setappdata(0,'a',a)



% --- Executes on selection change in Noise_menu.
function Noise_menu_Callback(hObject, eventdata, handles)
% hObject    handle to Noise_menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
allItems = handles.Noise_menu.String;
selectedIndex = handles.Noise_menu.Value;
selectedItem = allItems{selectedIndex};
switch selectedItem
    case char('salt_and_pepper')
        a = getGlobalx()
        a = imnoise(a,'salt & pepper',0.05)
        display_image(handles, a)
        
     case char('Gaussian')
        a = getGlobalx()
        a = imnoise(a,'gaussian')
        display_image(handles, a)
        
    case char('Speckle')
        a = getGlobalx()
        a = imnoise(a,'speckle')
        display_image(handles, a)
        
    case char('Poisson')
        a = getGlobalx()
        a = imnoise(a,'poisson')
        display_image(handles, a)
        
    case char('Uniform')
        a = getGlobalx()
        a = im2double(a);
        p = 0.2; % p between 0 and 1
        a = (a + p*rand(size(a)))/(1+p);
        display_image(handles, a)
    
    
end 
% Hints: contents = cellstr(get(hObject,'String')) returns Noise_menu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Noise_menu


% --- Executes during object creation, after setting all properties.

function Noise_menu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Noise_menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in popupmenu4.
function popupmenu4_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu4 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu4


% --- Executes during object creation, after setting all properties.
function popupmenu4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in plot_histogram.
function plot_histogram_Callback(hObject, eventdata, handles)
% hObject    handle to plot_histogram (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
a = getGlobalx()
axes(handles.axes2);
imhist(a);


% --- Executes on button press in convert_to_grey.
function convert_to_grey_Callback(hObject, eventdata, handles)
% hObject    handle to convert_to_grey (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
a = getGlobalx()
a = rgb2gray(a)
setappdata(0,'a',a)
display_image(handles, a)


% --- Executes on selection change in Transform_menu.
function Transform_menu_Callback(hObject, eventdata, handles)
% hObject    handle to Transform_menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
allItems = handles.Transform_menu.String;
selectedIndex = handles.Transform_menu.Value;
selectedItem = allItems{selectedIndex};
amount = getGlobaly()

switch selectedItem
    case char('Power')
        a = getGlobalx()
        [m, n] = size(a)
        doublea = im2double(a)
        c = 1;
        g = getGlobaly() % Gamma Correction Array
        I = a
        for p = 1:m
            for q = 1:n
                I(p, q) = c* doublea(p, q)^ g;
            end
        end
        display_image(handles, I)
        
    case char('Root')
        a = getGlobalx()
        [m, n] = size(a)
        doublea = im2double(a)
        c = 1;
        g = getGlobaly() % Gamma Correction Array
        I = a
        for p = 1:m
            for q = 1:n
                I(p, q) = c* doublea(p, q)^ (1/g);
            end
        end
        display_image(handles, I)
        
    case char('Logarithmic')
        a = getGlobalx()
        a = im2double(a)
        y = getGlobaly()
        a = log(1+y)
        display_image(handles, a)
        
    case char('Inverse_log')
        a = getGlobalx()
        y = getGlobaly()
        [M N] = size(a)
        for x = 1:M
            for s = 1:N
                m = double(a(x,s));
                I(x,s) = 1/(y*log10(1+m));
            end
        end
        display_image(handles, I)
        
    case char('Negative')
        a = getGlobalx()
        a = im2double(a)
        a = imcomplement(a)
        display_image(handles, a)
       
  
    
end
% Hints: contents = cellstr(get(hObject,'String')) returns Transform_menu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Transform_menu


% --- Executes during object creation, after setting all properties.
function Transform_menu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Transform_menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Reset.
function Reset_Callback(hObject, eventdata, handles)
% hObject    handle to Reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
a = getappdata(0,'a');
setGlobalx(a)
axes(handles.axes1);
imshow(a);
axes(handles.axes2);
imhist(a);



function amount_Callback(hObject, eventdata, handles)
% hObject    handle to amount (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
amount = get(handles.amount,'String')
if isempty(amount)
    amount = 0.1
else
    amount = str2double(char(amount))
    
setGlobaly(amount)
print(amount)
end
% Hints: get(hObject,'String') returns contents of amount as text
%        str2double(get(hObject,'String')) returns contents of amount as a double


% --- Executes during object creation, after setting all properties.
function amount_CreateFcn(hObject, eventdata, handles)
% hObject    handle to amount (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in seg_menue.
function seg_menue_Callback(hObject, eventdata, handles)
% hObject    handle to seg_menue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
allItems = handles.seg_menue.String;
selectedIndex = handles.seg_menue.Value;
selectedItem = allItems{selectedIndex};
switch selectedItem
    case char('Global_thresholding')
        a = getGlobalx()
        a = imbinarize(a, 'global');
        display_image(handles, a)
    
    case char('Adaptive_thresholding')
        a = getGlobalx()
        a = imbinarize(a, 'adaptive');
        display_image(handles, a)
        
    case char('Erosion')
        a = getGlobalx()
        se = strel('disk',33);        
        erodedBW = imerode(a,se);
        display_image(handles, erodedBW)
        
    case char('Dilation')
        a = getGlobalx()
        se = strel('disk',33);        
        erodedBW = imdilate(a,se);
        display_image(handles, erodedBW)
        
    case char('Opening')
        a = getGlobalx()
        se = strel('disk',33);              
        erodedBW = imerode(a,se);           % erosion
        erodedBW = imdilate(erodedBW,se);   % dilation
        display_image(handles, erodedBW)
        
    case char('Closing')
        a = getGlobalx()
        se = strel('disk',33); 
        erodedBW = imdilate(a,se);          % dilation
        erodedBW = imerode(erodedBW,se);    % erosion
        display_image(handles, erodedBW)
            
end       

% Hints: contents = cellstr(get(hObject,'String')) returns seg_menue contents as cell array
%        contents{get(hObject,'Value')} returns selected item from seg_menue


% --- Executes during object creation, after setting all properties.
function seg_menue_CreateFcn(hObject, eventdata, handles)
% hObject    handle to seg_menue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu8.
function popupmenu8_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu8 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu8


% --- Executes during object creation, after setting all properties.
function popupmenu8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in frequency_domain.
function frequency_domain_Callback(hObject, eventdata, handles)
% hObject    handle to frequency_domain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
a = getGlobalx();
a = im2double(a);
f = fft2(a);
s = fftshift(log(1+abs(f)));
axes(handles.axes2);
imshow(s,[]);
