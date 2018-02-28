function varargout = OLA(varargin)
% OLA MATLAB code for OLA.fig
%      OLA, by itself, creates a new OLA or raises the existing
%      singleton*.
%
%      H = OLA returns the handle to a new OLA or the handle to
%      the existing singleton*.
%
%      OLA('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in OLA.M with the given input arguments.
%
%      OLA('Property','Value',...) creates a new OLA or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before OLA_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to OLA_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help OLA

% Last Modified by GUIDE v2.5 21-Nov-2017 13:23:19

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @OLA_OpeningFcn, ...
                   'gui_OutputFcn',  @OLA_OutputFcn, ...
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


% --- Executes just before OLA is made visible.
function OLA_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to OLA (see VARARGIN)

% Choose default command line output for OLA
handles.output = hObject

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes OLA wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = OLA_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes during object creation, after setting all properties.
function axes2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes2
I= imread('HeaderFooter.png');
axes(hObject);
imshow(I);


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Load positive samples & data set.

[fname path]=uigetfile('*.*','Load training file');
%cd(path);
%rawdata1=load([path,fname]);
train=strcat(path,fname);
train2=fname;
%set(handles.trainFilename,'String',train2);
disp(train2);
load('data.mat')    
load(train2);
load('matureMango.mat');
global  positiveInstances;
switch train2
    case "trainingPositiveMango3.mat"
        %trainingPositiveMango3 = evalin('base','trainingPositiveMango3');
        positiveInstances = trainingPositiveMango3;
        
    case "Detection1.mat"
        disp('pasok');
        %Detection1 = evalin('base','Detection1');
        disp(Detection1);
        positiveInstances= Detection1;
    case "Detection2.mat"
        %Detection2 = evalin('base','Detection2');
        positiveInstances= Detection2;
        
    case "Detection3.mat"
        %Detection3 = evalin('base','Detection3');
        positiveInstances= Detection3;
    case "Detection4.mat"
        %Detection4 = evalin('base','Detection4');
        positiveInstances= Detection4;
        
    case "Detection5.mat"
        %Detection5 = evalin('base','Detection5');
        positiveInstances= Detection5;
    case "Detection6.mat"
        %Detection6 = evalin('base','Detection6');
        positiveInstances= Detection6;
        case "Detection7.mat"
       %Detection7 = evalin('base','Detection7');
        positiveInstances= Detection7;
    case "Detection8.mat"
        %Detection8 = evalin('base','Detection8');
        positiveInstances= Detection2;
    case "Detection9.mat"
        %Detection9 = evalin('base','Detection9');
        positiveInstances= Detection9;
    case "Detection10.mat"
        %Detection10 = evalin('base','Detection10');
        positiveInstances= Detection10;
    case "Detection11.mat"
       %Detection11 = evalin('base','Detection11');
        positiveInstances= Detection11;
    case "Detection12.mat"
       %Detection12 = evalin('base','Detection12');
        positiveInstances= Detection12;
        case "Detection13.mat"
        %Detection13 = evalin('base','Detection13');
        positiveInstances= Detection13;
    case "Detection14.mat"
        %Detection14 = evalin('base','Detection14');
        positiveInstances= Detection14;
    case "Detection15.mat"
       %Detection15 = evalin('base','Detection15');
        positiveInstances= Detection15;
    case "Detection16.mat"
        %Detection16 = evalin('base','Detection16');
        positiveInstances= Detection16;
    case "Detection17.mat"
       % Detection17 = evalin('base','Detection17');
        positiveInstances= Detection17;
    case "Detection18.mat"
        %Detection18 = evalin('base','Detection18');
        positiveInstances= Detection18;
    case "Detection19.mat"
        %Detection19 = evalin('base','Detection19');
        positiveInstances= Detection19;
    case "Detection20.mat"
      % Detection20 = evalin('base','Detection20');
        positiveInstances= Detection20;
    case "Detection21.mat"
      % Detection21 = evalin('base','Detection21');
        positiveInstances= Detection21;
    case "Detection22.mat"
      % Detection22 = evalin('base','Detection22');
        positiveInstances= Detection22;
    case "Detection23.mat"
       % Detection23 = evalin('base','Detection23');
        positiveInstances= Detection23;
    case "Detection24.mat"
       % Detection24 = evalin('base','Detection24');
        positiveInstances= Detection24;
    case "Detection25.mat"
       % Detection25 = evalin('base','Detection25');
        positiveInstances= Detection25;
    case "Detection26.mat"
       % Detection26 = evalin('base','Detection26');
        positiveInstances= Detection26;
end
%matureMango = evalin('base','matureMango');
matureInstances = matureMango;
%assignin('base','matureInstances',matureInstances);
% Add the image directory to the MATLAB path.
imDir = fullfile(matlabroot,'toolbox','vision','visiondata',...
    'mangoTraining');
addpath(imDir);
%%
% Specify the folder for images.
negativeFolder = fullfile(matlabroot,'toolbox','vision','visiondata',...
    'mangoNegative');
mangoFolder = fullfile(matlabroot,'toolbox','vision','visiondata',...
    'mangoSet');
%%
% Create an |imageDatastore| object containing images.
negativeImages = imageDatastore(negativeFolder);
mangoImages = imageDatastore(mangoFolder);

%% Calculate RGBs to build Average Linear Classifier
 %matureInstances = evalin('base','matureInstances');
 sizer = size(matureInstances,1);
 countme2 = (sizer+1);
 global RGBSet;
 RGBSet = nan(1,3);
 for j=0:countme2
     if j==countme2
     %elseif j == countme2
     elseif j == 0    
     else
         imgSet = readimage(mangoImages,j); %Read Image from Datastore
         C = matureInstances{j,2}{:,1:2}; %Get BBOX values
         cCounter = size(C,1);
         %% Allow variable number of BBOX
            amaxNum = cCounter;
            anum1 =(amaxNum - (amaxNum-1)); %first value X
            anum2 =(amaxNum+1); %First value Y
            anum3 =(anum2 + amaxNum); %First Value Width
            anum4 =(anum3 + amaxNum); %First Value Height
            countmefirst = (cCounter+1);
            for g=0:countmefirst
                if g==countmefirst-1

                elseif g == countmefirst

                else
                      ximgcent=((C(anum1+g)+C(anum3+g))+C(anum1+g))/2;
                      yimgcent=((C(anum2+g)+C(anum4+g))+C(anum2+g))/2;
                      ximg=[(ximgcent), (ximgcent), ximgcent, (ximgcent), (ximgcent)];%X coord of Points
                      yimg=[(yimgcent+8), (yimgcent+4), yimgcent, (yimgcent-4), (yimgcent-8)];%Y coord of Points
                      pixels = impixel(imgSet,ximg,yimg);
                      %Average
                      rpixave = (pixels(1)+pixels(2)+pixels(3)+pixels(4)+pixels(5))/5;
                      gpixave = (pixels(6)+pixels(7)+pixels(8)+pixels(9)+pixels(10))/5;
                      bpixave = (pixels(11)+pixels(12)+pixels(13)+pixels(14)+pixels(15))/5;
                      RGBAve = [round(rpixave),round(gpixave),round(bpixave)];

                      RGBSet = [RGBAve;RGBSet];
                end
            end
     end    
     
     
 end
 
 %% Average RGB Set for Linear Classifier
RGBSize= size(RGBSet,1);
RGBSet(RGBSize,:) = [];
RGBCounter= (RGBSize+1);
RGBSize2= size(RGBSet,1);
RSHold= 0;
GSHold= 0;
BSHold= 0;
 for h=1:RGBCounter 
     if h==RGBCounter-1
     elseif h == RGBCounter
     else
          RSet= RGBSet(h,1);
          GSet= RGBSet(h,2);
          BSet= RGBSet(h,3);
          RSHold = RSHold+RSet;
          GSHold = GSHold+GSet;
          BSHold = BSHold+BSet;
     end
 
 end
 
%% Declaring Line Boundaries
global RLineAve GLineAve BLineAve RGBLineAve RYMAX GYMAX BYMAX RYMIN GYMIN BYMIN RGMAX GGMAX BGMAX;
RLineAve= round((RSHold/RGBSize2));
GLineAve= round((GSHold/RGBSize2));
BLineAve= round((BSHold/RGBSize2));
RGBLineAve= [RLineAve,GLineAve ,BLineAve]; %Mid Line

RYMAX=255;%Yellow Line Max
GYMAX=255;
BYMAX=200;

RYMIN=100;%Yellow Line Min
GYMIN=100;
BYMIN=0;

RGMAX=70;%Green Line Max
GGMAX=190;
BGMAX=70;

% RGMIN=0;%Green Line Min
% GGMIN=128;
% BGMIN=0;
%%
% Train a cascade object detector
% using HOG features.
% NOTE: The command can take several minutes to run.
trainCascadeObjectDetector('MangoDetector.xml',positiveInstances, ...
    negativeFolder,'FalseAlarmRate',0.01,'NumCascadeStages',7,'TruePositiveRate',0.99);

h = msgbox('Training Complete','Training Status','warn');
%%
% Use the newly trained classifier to detect the mangoes in the image.
global detector;
detector = vision.CascadeObjectDetector('MangoDetector.xml');



% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% Load image file
global filename img;
[fname path]=uigetfile('*.*','Load an Image File');
fname=strcat(path,fname);
filename = fname;
img = imread(filename);
%assignin('base','img',img);
set(handles.text2,'String','Original Image','visible','on');
axes(handles.Image1);
imshow(img,'Parent',handles.Image1);

% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% Get RGB per BBOX
global img detectedImg numCounter bbox bbox2 bbox3 RLineAve GLineAve BLineAve RGBLineAve RYMAX GYMAX BYMAX RYMIN GYMIN BYMIN RGMAX GGMAX BGMAX filename; 
RLineAve1 = RLineAve;
%img = evalin('base','img');
pixels1 = nan(1,3);
mature1 = nan(1,3);
immature1 = nan(1,3);
%% Allow variable number of BBOX
maxNum = numCounter;
num1 =(maxNum - (maxNum-1)); %first value X
num2 =(maxNum+1); %First value Y
num3 =(num2 + maxNum); %First Value Width
num4 =(num3 + maxNum); %First Value Height
%% Get demn RGBs
countme = (numCounter+1);
bbox2 = [];%clear 
bbox3 = [];%clear
 for k=0:countme %Calculate demn RGBs
     if k==countme-1
         
     elseif k == countme
         
     else
          ximgcent=((bbox(num1+k)+bbox(num3+k))+bbox(num1+k))/2;
          yimgcent=((bbox(num2+k)+bbox(num4+k))+bbox(num2+k))/2;
          curbox=[bbox(num1+k), bbox(num2+k), bbox(num3+k), bbox(num4+k)];
          ximg=[(ximgcent), (ximgcent), ximgcent, (ximgcent), (ximgcent)];%X coord of Points
          yimg=[(yimgcent+8), (yimgcent+4), yimgcent, (yimgcent-4), (yimgcent-8)];%Y coord of Points
          pixels = impixel(img,ximg,yimg);
          %Average
          rpixave = (pixels(1)+pixels(2)+pixels(3)+pixels(4)+pixels(5))/5;
          gpixave = (pixels(6)+pixels(7)+pixels(8)+pixels(9)+pixels(10))/5;
          bpixave = (pixels(11)+pixels(12)+pixels(13)+pixels(14)+pixels(15))/5;
          RAve=round(rpixave);
          GAve=round(gpixave);
          BAve=round(bpixave);
          RGBAve = [RAve,GAve,BAve];
          pixels1 = [RGBAve;pixels1];
          
          if (RAve >= RLineAve1) && (GAve >= RLineAve1) && (BAve >= BLineAve)...
                  && (RAve <= RYMAX) && (GAve <= GYMAX) && (BAve <= BYMAX) %Best Case
              bbox2=[curbox;bbox2];
              mature1=[RGBAve;mature1];
          elseif (RAve < RLineAve1) && (GAve >= RLineAve1) && (BAve >= BLineAve)...
                  && (RAve <= RYMAX) && (GAve <= GYMAX) && (BAve <= BYMAX)...
                  && (RAve >= RYMIN)%R Lower
              bbox2=[curbox;bbox2];
              mature1=[RGBAve;mature1]; 
          elseif (RAve >= RLineAve1) && (GAve < RLineAve1) && (BAve >= BLineAve)...
                  && (RAve <= RYMAX) && (GAve <= GYMAX) && (BAve <= BYMAX)...
                  && (GAve >= GYMIN)%G Lower
              bbox2=[curbox;bbox2];
              mature1=[RGBAve;mature1];
          elseif (RAve >= RLineAve1) && (GAve >= RLineAve1) && (BAve < BLineAve)...
                  && (RAve <= RYMAX) && (GAve <= GYMAX) && (BAve <= BYMAX) %B Lower
              bbox2=[curbox;bbox2];
              mature1=[RGBAve;mature1];
          elseif (RAve < RLineAve1) && (GAve < RLineAve1) && (BAve >= BLineAve)...
                  && (RAve <= RYMAX) && (GAve <= GYMAX) && (BAve <= BYMAX)...
                  && (RAve >= RYMIN) && (GAve >= GYMIN)%R and G Lower
              bbox2=[curbox;bbox2];
              mature1=[RGBAve;mature1];
          elseif (RAve >= RLineAve1) && (GAve < RLineAve1) && (BAve < BLineAve)...
                  && (RAve <= RYMAX) && (GAve <= GYMAX) && (BAve <= BYMAX)...
                  && (GAve >= GYMIN) && (BAve >= BYMIN)%G and B Lower
              bbox2=[curbox;bbox2];
              mature1=[RGBAve;mature1];
          elseif (RAve < RLineAve1) && (GAve >= RLineAve1) && (BAve < BLineAve)...
                  && (RAve <= RYMAX) && (GAve <= GYMAX) && (BAve <= BYMAX)...
                  && (RAve >= RYMIN) && (BAve >= BYMIN)%R and B Lower
              bbox2=[curbox;bbox2];
              mature1=[RGBAve;mature1];
          elseif (RAve < RLineAve1) && (GAve < RLineAve1) && (BAve < BLineAve)...
                  && (RAve <= RYMAX) && (GAve <= GYMAX) && (BAve <= BYMAX)...
                  && (RAve >= RYMIN) && (GAve >= GYMIN) && (BAve >= BYMIN)%RGB All Lower, WORST CASE
              bbox2=[curbox;bbox2];
              mature1=[RGBAve;mature1];
          else %Immature goes here
              bbox3=[curbox;bbox3];
              immature1=[RGBAve;immature1];
          end
              
          
     end
 
 end
%% CLASSIFY THEM BY USING THE RGB SET
numFinalCounter = size(bbox2,1);
numFinalCounter2 = size(bbox3,1);
if size(bbox2,1) == 0
    h = msgbox('No mature mangoes detected');
    detectedImg2= imread(filename);
else
    detectedImg2= insertText(insertObjectAnnotation(img,'rectangle',bbox2,'Mature','color',...
    'yellow'),[5,5],numFinalCounter);
end

if size(bbox3,1) == 0
    h = msgbox('No immature mangoes detected');
    detectedImg3= imread(filename);
else
    detectedImg3= insertText(insertObjectAnnotation(img,'rectangle',bbox3,'Immature','color',...
    'green'),[5,5],numFinalCounter2);
end

%%
% Display the detected mangoes.
%detectedImg=evalin('base','detectedImg');
%figure; imshow(detectedImg);title('Detected Mangoes')
%figure; imshowpair(detectedImg2,detectedImg3,'blend');title('Classified Mangoes');%%
axes(handles.Image1);
set(handles.text2,'String','Detected Image','visible','on');
imshow(detectedImg,'Parent',handles.Image1);
axes(handles.Image2);
set(handles.text3,'String',' Classified Image','visible','on');
imshowpair(detectedImg2,detectedImg3,'blend','Parent',handles.Image2);
% Remove the image directory from the path.
%rmpath(imDir);


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global  detectedImg img detector bbox  bbox2 bbox3 numCounter;
%img= evalin('base','img');
bbox = step(detector,img); 
bbox2= step(detector,img);
bbox3= step(detector,img);%initialize
%%
% Insert bounding box rectangles and return the marked image.
 %detectedImg = insertObjectAnnotation(img,'rectangle',bbox,'Mango');
 numCounter = size(bbox,1);
detectedImg= insertText(insertObjectAnnotation(img,'rectangle',bbox,'Mango'),[5,5],numCounter);
%assignin('base','detectedImg',detectedImg);
set(handles.text3,'String','Detected Image','visible','on');
axes(handles.Image2);
imshow(detectedImg,'Parent',handles.Image2);
