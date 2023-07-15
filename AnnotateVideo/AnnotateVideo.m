%   Add a path for support functions.
    % addpath('..\SupportFunctions');
    
%   Set the image file path, prefix, and extension.
    PathString = '\\tri-gt2\Gttr4CPU\00001\Video\';
    CameraNum = input('Camera Number?');
    RunId = input('Run Number?');

    
%   What should be included in the output?

%   Include radar events?
    UseRadarEvents = 0;
%   Include radar calibrated detections?
    UseRadarDetections = 0;
%   Limit on range for radar detections.
    RadarRangeThresh = 60.;
    nRadars = 6;
%   Exclude coasted targets?
    RadarNoCoast = 0;
%   For radar detections, how close to the frame time (in Cs) for display?
    RadarTimeCsTol = 5;

%   Include video calibrated detections?    
    UseVideoDetections = 0;
%   Include video events?
    UseVideoEvents = 1;

%   Include lidar calibrated detections?
    UseLidarDetections = 0;
%   Include lidar events?
    UseLidarEvents = 1;
%   Limit on range for lidar detections.
    LidarRangeThresh = 60.;
%   For lidar detections, how close to the frame time (in Cs) for display?
    LidarTimeCsTol = 4;
%   Which lidars to use.
    LidarNums = [1];
%   Show bounding boxes?
    UseLidarBoundingBoxes = 0;

%   Show instumented POV GPS Antenna?
    UsePOV = 0;
    
%   Add road stripe.
    UseStripe = 0;
    StripeDx = 0.5;
    StripeNDots = 100.;
    StripeDy = 1.5;

%   Parameters for radar and lidar marker size.
%   MarkerDiameter = max(1, PixelsatMax*(1 - Dist/MarkerRangeMax))
    MarkerRangeMax = 60.;
    PixelsatMax = 20.;
%   Output file pre-dot suffix.
    Suffix = ('VL');
    
%   Range of frames to save in the output vid.
    FirstFrame = input('First Frame?');
    LastFrame = input('Last Frame?');
%   Range of frames to show live.
    FirstDisplayFrame = FirstFrame;
    LastDisplayFrame = LastFrame;
%   Establish the images used for viewing and saving. 
    MaxImages = LastFrame;



    
%   Get video calibration ID.
%   This is a bit of a hack, and it depends on the assumption that the same
%   calibration was used for the entire run.
    [IntCalNum, ExtCalNum] = GetVidCalNumsbyRunId(RunId, CameraNum);
    
%   Get intrinsic calibration.
    [CameraIntCal] = GetIntrinsicCalParameters(CameraNum, IntCalNum);
    xpcmin = CameraIntCal.xpcmin(1); xpcmax = CameraIntCal.xpcmax(1); 
    ypcmin = CameraIntCal.ypcmin(1); ypcmax = CameraIntCal.ypcmax(1); 
%   Get extrinsic calibration.
    [CameraExtCal] = GetExtrinsicCalParameters(CameraNum, ExtCalNum);

%   Video file name.
    FileName = strcat('Basler', num2str(CameraNum), 'Out_001_0', num2str(RunId, '%04u'), '.bin');

%   Initialize the image variables. 
    ImageRows = CameraIntCal.NumRows(1);
    ImageColumns = CameraIntCal.NumCols(1);
    imgraw = zeros(ImageColumns, ImageRows, 'uint8');
    img = zeros(ImageColumns, ImageRows, 'uint8');

%   Open the output file.
    outfile = VideoWriter(strcat(extractBefore(FileName,'.'), Suffix, '.avi'), 'Uncompressed AVI');
    outfile.FrameRate = 10;
    open(outfile);

%   Establish communication with the databases.
    conn = database('GTTR4CPU', ' ', ' ');
    conna = database('GTTRAnalysis', ' ', ' ');
    % conns = database('GTTRSteve', ' ', ' ');

%   Get the relationship between TimeCs and V51Frame.
    [query] = BuildFrameTimeCsJoin(RunId, CameraNum);
    FrameTime = fetch(conn, query);
    TimeCsMin = FrameTime.TimeCs(max(FirstFrame, 1)) - 10;
    TimeCsMax = FrameTime.TimeCs(min(LastFrame, height(FrameTime))) + 10;

    
    
    
    
    
    
    
    
    
%   POV
    if (UsePOV == 1)
        GetPOVDetections;
    end
       
%   VIDEO DETECTIONS
    if (UseVideoDetections == 1)
        GetVideoDetections;
    end
    
%   VIDEO EVENTS
    if (UseVideoEvents == 1)
        GetVideoEvents;
    end
    
%   RADAR DETECTIONS
    if (UseRadarDetections == 1)
        GetRadarDetections;
    end
    
%   RADAR EVENTS
    if (UseRadarEvents == 1)
        GetRadarEvents;
    end
    
%   LIDAR DETECTIONS
    if (UseLidarDetections == 1)
        GetLidarDetections;
    end
    
%   LIDAR EVENTS
    if (UseLidarEvents == 1)
        GetLidarTracks;
    end

%   ROAD STRIPE
    if (UseStripe == 1)
        GetRoadStripe;
    end
    
    
    

%   Open the file.
    if ~isequal(FileName,0)
        infile = fopen(fullfile(PathString,FileName));
%       Skip over frames that are not of interest.
        FrameCount = min(FirstFrame, FirstDisplayFrame) - 1;
        fseek(infile,FrameCount*(ImageColumns*ImageRows+32),'bof'); 


%       Loop through the images.
        fprintf(1,'Processing frame %06i of %06i\n',0,MaxImages);
        FrameCount = FrameCount + 1;    
        while ~feof(infile) && FrameCount <= MaxImages
            if (mod(FrameCount, 20) == 0)
                fprintf(1,'\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b %06i of %06i\n',FrameCount,MaxImages);
            end

        
%           READ THIS IMAGE.
%           Read the header stuff.
            framesize = fread(infile,1,'uint16');      %skip over framesize
            header = fread(infile,30,'uint8');    %skip over header
%           Read the image.
            imgraw = uint8(fread(infile,[ImageColumns,ImageRows],'uint8'));
%           img=mat2gray(imgraw');           %for converting grayscale images.  Note transposition.


%           STEPS IN THE IMAGE OUTPUT LOOP.
            if (FrameCount >= FirstFrame && FrameCount <= LastFrame)  

%               CONVERT THIS IMAGE.            
                BayerFilter = 'rggb';
                if (CameraNum == 6)
                    BayerFilter = 'bggr';
                end
                img = demosaic(imgraw', BayerFilter); %for converting Bayer color images 


                
%               ADD TIMECS TO THE IMAGE.  
                trow = (FrameTime.V51Frame == FrameCount);
                TimeCs = FrameTime.TimeCs(trow);
                note = strcat('TimeCs = ', num2str(TimeCs));
                img = insertText(img, [960 1140], note, 'TextColor', 'white', 'FontSize', 36, 'BoxOpacity', 0, 'AnchorPoint', 'Center');
           
            
            
%               PROJECT POV (IF ASKED).        
                if (UsePOV == 1)
                    ProjectPOVDetections;
                end


                
%               PROJECT VIDEO BOUNDING BOXES (IF ASKED).        
                if (UseVideoDetections == 1)
                    ProjectVideoDetections;
                end
        
        
        
%               PROJECT VIDEO BOUNDING BOXES FROM THE EVENTS TABLE (IF ASKED).        
                if (UseVideoEvents == 1)
                    ProjectVideoEvents;
                end
                
                
                
 %              PROJECT RADAR DETECTIONS FROM THE EVENTS TABLE (IF ASKED).
                if (UseRadarEvents == 1)
                    ProjectRadarEvents;
                end
                
                
                
 %              PROJECT RADAR DETECTIONS (IF ASKED).
                if (UseRadarDetections == 1)
                    ProjectRadarDetections;
                end
 
 
                
               
                
                
 %              STEP 8 PROJECT LIDAR DETECTIONS (IF ASKED).        
                if (UseLidarDetections == 1)
                    ProjectLidarDetections;
                end

                
                
                
 %              STEP 9 PROJECT LIDAR DETECTIONS FROM TRACKS (IF ASKED).        
                if (UseLidarEvents == 1)
                    ProjectLidarTracks;
                end
                
                
                if (UseStripe == 1)
                    ProjectRoadStripe;
                end
              



                    
                   
                   
                   
                   
                   
                   
                   
                   
                   
                   
                   
                   
                   
                   
                
                
                
                
                
                
                
         
%               STEP 10: DISPLAY THE IMAGE.        
                if (FrameCount >= FirstDisplayFrame && FrameCount <= LastDisplayFrame)
                    hold('off');
                    figure(1)      %uncomment this line and the imshow line below to watch conversion
                    imshow(img)
%                   Plot the found points.
            		axis('on');
                    hold('on');
                end % End of display loop.
                
                
                
%               STEP 11: SAVE THE IMAGE.
                writeVideo(outfile,img);
            end % End of output loop.

    


            FrameCount = FrameCount + 1;
        
        end % End of file read loop.

        fclose(infile);
        close(outfile);



    end % End of if file exists.
    


