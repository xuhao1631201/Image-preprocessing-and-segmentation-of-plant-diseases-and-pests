function varargout = gui2(varargin)
% GUI2 MATLAB code for gui2.fig
%      GUI2, by itself, creates a new GUI2 or raises the existing
%      singleton*.
%
%      H = GUI2 returns the handle to a new GUI2 or the handle to
%      the existing singleton*.
%
%      GUI2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI2.M with the given input arguments.
%
%      GUI2('Property','Value',...) creates a new GUI2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui2_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui2

% Last Modified by GUIDE v2.5 15-May-2021 15:00:30

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui2_OpeningFcn, ...
                   'gui_OutputFcn',  @gui2_OutputFcn, ...
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


% --- Executes just before gui2 is made visible.
function gui2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui2 (see VARARGIN)

% Choose default command line output for gui2
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes gui2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = gui2_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
global f;
I1=rgb2gray(f); %把 RGB 图像转化成灰度图像
thresh = graythresh(I1);     %自动确定二值化阈值
I2=im2bw(I1,thresh); %对图像二值化
%imerode()腐蚀
%strel函数的功能是运用各种形状和大小构造结构元素
SE1=strel('disk',3);%这里是创建一个半径为3的平坦型圆盘结构元素
ER=imerode(I2,SE1);
axes(handles.axes3);
imshow(ER);
% figure();
% imshow(ER);
% title('腐蚀图像');
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
global f;
I1=rgb2gray(f); %把 RGB 图像转化成灰度图像
thresh = graythresh(I1);     %自动确定二值化阈值
I2=im2bw(I1,thresh); %对图像二值化
SE1=strel('disk',3);
%imerode()腐蚀
%strel函数的功能是运用各种形状和大小构造结构元素
DI=imdilate(I2,SE1);%使用结构元素SE对图像I2进行一次膨胀
axes(handles.axes3);
imshow(DI);
% figure();
% imshow(DI);
% title('膨胀图像');
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
global f;
I1=rgb2gray(f); %把 RGB 图像转化成灰度图像
thresh = graythresh(I1);     %自动确定二值化阈值
I2=im2bw(I1,thresh); %对图像二值化
SE=[0 1 0
    1 1 1
    0 1 0];
OP=imopen(I2,SE);
axes(handles.axes3);
imshow(OP);
% figure();
% imshow(OP);
% title('开运算图像');
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
global f;
I1=rgb2gray(f); %把 RGB 图像转化成灰度图像
thresh = graythresh(I1);     %自动确定二值化阈值
I2=im2bw(I1,thresh); %对图像二值化
SE=[0 1 0
    1 1 1
    0 1 0];
%imclose()闭运算
CL=imclose(I2,SE);%直接闭运算
axes(handles.axes3);
imshow(CL);
% figure();
% imshow(CL);
% title('闭运算图像');
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
global f;
T=graythresh(f);    %采用Otsu方法计算最优阈值T对图像二值化；  
I=im2bw(f,T);
% subplot(2,2,1),imshow(I);title('Otsu法二值图');
axes(handles.axes3);
imshow(I);
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
[filename, pathname, filterindex] = uigetfile('C:\Users\Hasee\Desktop\毕业设计\测试图库\.jpg', '选择图片');
file = fullfile(pathname, filename); 
global f;
f= imread(file); 
f=im2double(f);
axes(handles.axes2);
imshow(f);
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
global f;
I=f;
BW = im2bw(I(:,:,3),0.3);%二值化,注意应保证集水盆地的值较低（为0），否则就要对b取反
I(repmat(BW,[1,1,3]))=1;
%  gc=~BW
gc =BW;
D = bwdist(gc); % 距离变换
L = watershed(-D); % 计算距离变换的负分水岭变换
w=L== 0.5;
im3=gc&~w;  % 黑色叠加在原图上后的分水岭脊线
axes(handles.axes5);
imshow(im3);
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
global f;
I2=f;
I2=imadjust(I2,[0.01,0.4],[0.01,0.5],0.8);
I_rgb=I2;
%进行kmeans聚类
k=2;%设置聚类中心个数
[m,n,p]=size(I_rgb);%提取原图片尺寸信息
I=reshape(double(I_rgb), m*n, p);%reshape矩阵，预处理
[Idx,C,sumD,D]=kmeans(I,k);
I_mid=uint8(C(Idx,:));
for i=1:m
for j=1:n
if Idx((i-1)*n+j)==1
I_mid((i-1)*n+j,:)=[255 0 0];
elseif Idx((i-1)*n+j)==2
I_mid((i-1)*n+j,:)=[0 255 0];
elseif Idx((i-1)*n+j)==3
I_mid((i-1)*n+j,:)=[0 0 255];
elseif Idx((i-1)*n+j)==4
I_mid((i-1)*n+j,:)=[255 255 0];
end
end
end
I_seg=reshape(I_mid,m,n,p);
im3=I_seg;
im4=im2bw(im3);
im5=~im4;
axes(handles.axes5);
imshow(im5);
figure();
subplot(1, 3, 1);
imshow(im3);
title('1')
subplot(1, 3, 2);
imshow(im4);
title('2');
subplot(1, 3, 3);
imshow(im5);
title('3');
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in popupmenu3.
function popupmenu3_Callback(hObject, eventdata, handles)
global f;
I=f;
I2=rgb2gray(I);
b=get(handles.popupmenu3,'value');
switch b
    case 1
        I3=edge(I2, 'roberts');%应用roberts算子对图像进行边缘检测
        axes(handles.axes5);
        imshow(I3);
    case 2
        I4=edge(I2, 'sobel');%应用sobel算子对图像进行边缘检测
        axes(handles.axes5);
        imshow(I4);
    case 3
        I5=edge(I2, 'prewitt');%应用prewitt算子对图像进行边缘检测
        axes(handles.axes5);
        imshow(I5);
    case 4
        I6=edge(I2, 'canny');%应用canny算子对图像进行边缘检测axes(handles.axes3);
        axes(handles.axes5);
        imshow(I6);
    case 5
        I7=edge(I2, 'log');%应用log算子对图像进行边缘检测
        axes(handles.axes5);
        imshow(I7);
end
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu3 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu3


% --- Executes during object creation, after setting all properties.
function popupmenu3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
global f;
I=im2double(f);%转换数据类型为double
 I=rgb2gray(I);

% 增加对比度
% Fa=2;Fb=-55;
% P=Fa.*I+Fb/255;
k=2;b=10;
P=k.*I+b/255;
axes(handles.axes4);
imshow(P);
[M,N]=size(I);
figure();
subplot(2,2,1);
[H,x]=imhist(I,64);%计算64个区间的灰度直方图
stem(x,H/M/N,'.');%显示原图像的直方图
title('原图像','fontsize',8);

subplot(2,2,2);
[H,x]=imhist(P,64);
stem(x,H/M/N,'.');
title('Fa=2 Fb=-55 增加对比度','fontsize',8);

% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
global f;
I=im2double(f);%转换数据类型为double
I=rgb2gray(I);
m = 255;
H = histeq(I,m);
axes(handles.axes4);
imshow(H,[]);

% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
global f;
I=im2double(f);%转换数据类型为double
I=rgb2gray(I);

[M,N]=size(I);
rL=0.5;
rH=4.7;%可根据需要效果调整参数
c=2;
d0=10;
I1=log(I+1);%取对数
FI=fft2(I1);%傅里叶变换
n1=floor(M/2);
n2=floor(N/2);
H = ones(M, N);
for i=1:M
    for j=1:N
        D(i,j)=((i-n1).^2+(j-n2).^2);
        H(i,j)=(rH-rL).*(exp(c*(-D(i,j)./(d0^2))))+rL;%高斯同态滤波
    end
end
I2=ifft2(H.*FI);%傅里叶逆变换
I3=real(exp(I2));
axes(handles.axes4);
imshow(I3,[]);
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
global f;
I=f;
[n m a]=size(I);%判断图像的大小
a=get(handles.popupmenu2,'value');
switch a
%     case 1
%         GrayImage= rgb2gray(f);%调用MATLAB库函数实现灰度化
%         axes(handles.axes3);
%         imshow(Img_Gray);
    case 1
        Img_Gray1=zeros(n,m);
        for x=1:n%通过双循环对图像进行灰度化处理
            for y=1:m
                Img_Gray1(x,y)=max(I(x,y,1),max(I(x,y,2),I(x,y,3)));  %第二种方法实现灰度化,最大值法灰度化
            end
        end
        axes(handles.axes3);
        imshow(Img_Gray1);
    case 2
        Img_Gray2=zeros(n,m);
        for x=1:n%通过双循环对图像进行灰度化处理
            for y=1:m
                Img_Gray2(x,y)=(I(x,y,1)+I(x,y,2)+I(x,y,3))/3;%第三种方法实现灰度化
                
            end
        end
        axes(handles.axes3);
        imshow(Img_Gray2);
    case 3
        Img_Gray3=zeros(n,m);
        for x=1:n%通过双循环对图像进行灰度化处理
            for y=1:m
                
                Img_Gray3(x,y)=0.30*I(x,y,1)+0.59*I(x,y,2)+0.11*I(x,y,3);%第四种方法实现灰度化
            end
        end
        axes(handles.axes3);
        imshow(Img_Gray3);
end

% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2


% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double


% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit10_Callback(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit10 as text
%        str2double(get(hObject,'String')) returns contents of edit10 as a double


% --- Executes during object creation, after setting all properties.
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_1_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
