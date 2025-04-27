//
//  ViewController.m
//  BodyFusion
//
//  Created by yanshi on 23/01/2019.
//  Copyright (c) 2019 yanshi. All rights reserved.
//

#import "ViewController.h"
#import "PointCloudViewController.h"
#import <MobileCoreServices/MobileCoreServices.h>

#include "Engine/ImageSourceEngine.h"
#include "Engine/IMUSourceEngine.h"

#include "ITMLib/ITMLib.h"
#include "ORUtils/MetalContext.h"

using namespace InfiniTAM::Engine;

static void* SessionRunningContext = &SessionRunningContext;

typedef NS_ENUM(NSInteger, SetupResult) {
    SetupResultSuccess,
    SetupResultCameraNotAuthorized,
    SetupResultDepthCameraNotFound,
    SetupResultSessionConfigurationFailed
};

@interface ViewController() <AVCaptureVideoDataOutputSampleBufferDelegate, AVCaptureDataOutputSynchronizerDelegate>

@property (nonatomic, strong) dispatch_queue_t sessionQueue;
@property (nonatomic, strong) dispatch_queue_t renderingQueue;
@property (nonatomic, strong) dispatch_queue_t dataOutputQueue;
@property (nonatomic, strong) MetalContext *context;
@property (nonatomic, strong) CMMotionManager *motionManager;
@property (nonatomic) SetupResult setupResult;
@property (nonatomic, strong) AVCaptureVideoDataOutput *videoOutput;
@property (nonatomic, strong) AVCaptureDepthDataOutput *depthOutput;
@property (nonatomic, strong) AVCaptureDataOutputSynchronizer *dataOutputSynchronizer;
@property (nonatomic, strong) AVCaptureSession* session;
@property (nonatomic, strong) AVCaptureDeviceDiscoverySession *videoDeviceDiscoverySession;
@property (nonatomic, strong) AVCaptureDeviceInput *videoDeviceInput;

@end

@implementation ViewController
{
    // MARK: - Properties
    
    CGColorSpaceRef depthSpace;
    Vector2i depthImageSize;
    Vector2i rgbLargeImageSize;
    Vector2f scaleFactor;
    // 定义一个vector 
    std::vector<float> intrinsics_vector;// fx, fy, cx, cy
    ITMUChar4Image *result;
    
    ImageSourceEngine *imageSource;
    ImageSourceEngine *rgbLargeSource;

    IMUSourceEngine *imuSource;
    ITMLibSettings *internalSettings;
    ITMMainEngine *mainEngine;
    
    ITMIMUMeasurement *imuMeasurement;

    // For iPhone front depth, ITMShortImage short type is half/float16 actually.
    ITMUChar4Image *inputFakeRGBImage;
    ITMShortImage *inputRawDepthImage;
    ITMUChar4Image *inputLargeRGBImage;

    
    bool calibrated;
    bool setupDone;
    bool fullProcess;
    bool isRecording;
    bool usingSensor;
    bool authWasSuccessful;
    int currentFrameNo;
    int saveFrameNo;
    int numFusionButtonClicked;
    
    float minDepth;
    float maxDepth;
    NSTimeInterval totalProcessingTime;
    int totalProcessedFrames;
    
    char documentsPath[1000], *docsPath;
    
    AVCaptureSession* session;  // 添加私有实例变量
};  // 添加分号结束类声明

// MARK: - View Controller Life Cycle

- (void) viewDidLoad
{
    [super viewDidLoad];
    
    // 初始化成员变量
    minDepth = 0.01;
    maxDepth = 0.4; // 2.0
    
    // Disable UI. The UI is enabled if and only if the session starts running.
    self.tbOut.enabled = NO;
    
    // Disable automatic focus management in SCNView
    if ([self.renderView isKindOfClass:[SCNView class]]) {
        SCNView *scnView = (SCNView *)self.renderView;
        scnView.allowsCameraControl = NO;
        scnView.rendersContinuously = NO;
    }
    
    // Create the motionManager.
    _motionManager = [[CMMotionManager alloc]init];
    _motionManager.deviceMotionUpdateInterval = 1.0f / 60.0f;
    
    // Create the AVCaptureSession.
    session = [[AVCaptureSession alloc] init];
    
    // Create queues
    self.sessionQueue = dispatch_queue_create("session", DISPATCH_QUEUE_SERIAL);
    self.renderingQueue = dispatch_queue_create("rendering", DISPATCH_QUEUE_SERIAL);
    self.dataOutputQueue = dispatch_queue_create("dataOutput", DISPATCH_QUEUE_SERIAL);
    
    self.setupResult = SetupResultSuccess;
    /*
     Check video authorization status. Video access is required and audio
     access is optional. If audio access is denied, audio is not recorded
     during movie recording.
     */
    switch ([AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo])
    {
        case AVAuthorizationStatusAuthorized:
        {
            // The user has previously granted access to the camera.
            break;
        }
        case AVAuthorizationStatusNotDetermined:
        {
            /*
             The user has not yet been presented with the option to grant
             video access. We suspend the session queue to delay session
             setup until the access request has completed.
             
             Note that audio access will be implicitly requested when we
             create an AVCaptureDeviceInput for audio during session setup.
             */
            dispatch_suspend(self.sessionQueue);
            [AVCaptureDevice requestAccessForMediaType:AVMediaTypeVideo completionHandler:^(BOOL granted) {
                if (!granted) {
                    self.setupResult = SetupResultCameraNotAuthorized;
                }
                dispatch_resume(self.sessionQueue);
            }];
            break;
        }
        default:
        {
            // The user has previously denied access.
            self.setupResult = SetupResultCameraNotAuthorized;
            break;
        }
    }
    
    /*
     Setup the capture session.
     In general, it is not safe to mutate an AVCaptureSession or any of its
     inputs, outputs, or connections from multiple threads at the same time.
     
     Don't perform these tasks on the main queue because
     AVCaptureSession.startRunning() is a blocking call, which can
     take a long time. We dispatch session setup to the sessionQueue, so
     that the main queue isn't blocked, which keeps the UI responsive.
     */
    dispatch_async(self.sessionQueue, ^{
        [self configureSession];
    });
    
    // Record time and frames.
    totalProcessingTime = 0;
    totalProcessedFrames = 0;
    
    // Setup main engine.
    [self setupEngine];
}

// viewWillAppear() may be called several times, but viewDidLoad() only once.
- (void) viewWillAppear:(BOOL)animated
{
    [self.navigationController setNavigationBarHidden:YES];
    [super viewWillAppear:animated];
    [self.tbOut setText:@"front depth"];
    
    // Reset/Clear previous reconstruction scene and tracking state.
    mainEngine->resetAll();
    setupDone = true;
    NSProcessInfoThermalState initialThermalState = [[NSProcessInfo processInfo] thermalState];
    if (initialThermalState == NSProcessInfoThermalStateSerious || initialThermalState == NSProcessInfoThermalStateCritical) {
        [self showThermalState:initialThermalState];
    }

    dispatch_async(self.sessionQueue, ^{
        switch (self.setupResult)
        {
            case SetupResultSuccess:
            {
                // Only start the session running if setup succeeded.
                [self configureSession];
                [session startRunning];
                break;
            }
            case SetupResultCameraNotAuthorized:
            {
                dispatch_async(dispatch_get_main_queue(), ^{
                    NSString* message = NSLocalizedString(@"App doesn't have permission to use the camera, please change privacy settings", @"Alert message when the user has denied access to the camera");
                    UIAlertController* alertController = [UIAlertController alertControllerWithTitle:@"Front Depth Camera" message:message preferredStyle:UIAlertControllerStyleAlert];
                    UIAlertAction* cancelAction = [UIAlertAction actionWithTitle:NSLocalizedString(@"OK", @"Alert OK button") style:UIAlertActionStyleCancel handler:nil];
                    [alertController addAction:cancelAction];
                    // Provide quick access to Settings.
                    UIAlertAction* settingsAction = [UIAlertAction actionWithTitle:NSLocalizedString(@"Settings", @"Alert button to open Settings") style:UIAlertActionStyleDefault handler:^(UIAlertAction* action) {
                        [[UIApplication sharedApplication] openURL:[NSURL URLWithString:UIApplicationOpenSettingsURLString] options:@{} completionHandler:nil];
                    }];
                    [alertController addAction:settingsAction];
                    [self presentViewController:alertController animated:YES completion:nil];
                });
                break;
            }
            case SetupResultDepthCameraNotFound:
            {
                dispatch_async(dispatch_get_main_queue(), ^{
                    NSString* message = NSLocalizedString(@"Cannot find front depth camera, requires iPhoneX, XS, XS Max or XR.", @"Alert message when TrueDepth camera not found");
                    UIAlertController* alertController = [UIAlertController alertControllerWithTitle:@"TrueDepth Camera" message:message preferredStyle:UIAlertControllerStyleAlert];
                    UIAlertAction* cancelAction = [UIAlertAction actionWithTitle:NSLocalizedString(@"OK", @"Alert OK button") style:UIAlertActionStyleCancel handler:nil];
                    [alertController addAction:cancelAction];
                    [self presentViewController:alertController animated:YES completion:nil];
                });
                break;
            }
            case SetupResultSessionConfigurationFailed:
            {
                dispatch_async(dispatch_get_main_queue(), ^{
                    NSString* message = NSLocalizedString(@"Unable to capture media", @"Alert message when something goes wrong during capture session configuration");
                    UIAlertController* alertController = [UIAlertController alertControllerWithTitle:@"Session Configuration" message:message preferredStyle:UIAlertControllerStyleAlert];
                    UIAlertAction* cancelAction = [UIAlertAction actionWithTitle:NSLocalizedString(@"OK", @"Alert OK button") style:UIAlertActionStyleCancel handler:nil];
                    [alertController addAction:cancelAction];
                    [self presentViewController:alertController animated:YES completion:nil];
                });
                break;
            }
        }
    });
}

- (void) viewDidDisappear:(BOOL)animated
{
    dispatch_async(self.sessionQueue, ^{
        if (self.setupResult == SetupResultSuccess) {
            [session stopRunning];
        }
    });
    
    [self.navigationController setNavigationBarHidden:NO];
    [super viewDidDisappear:animated];
}

- (void) didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
}

// You can use this opportunity to take corrective action to help cool the system down.
-(void) thermalStateChanged:(NSNotification*) notification
{
    NSProcessInfo* processInfo = notification.object;
    [self showThermalState: [processInfo thermalState]];
}

-(void) showThermalState:(NSProcessInfoThermalState) state
{
    dispatch_async(dispatch_get_main_queue(), ^{
        NSString* thermalStateString = @"UNKNOWN";
        if (state == NSProcessInfoThermalStateNominal) {
            thermalStateString = @"NOMINAL";
        } else if (state == NSProcessInfoThermalStateFair) {
            thermalStateString = @"FAIR";
        } else if (state == NSProcessInfoThermalStateSerious) {
            thermalStateString = @"SERIOUS";
        } else if (state == NSProcessInfoThermalStateCritical) {
            thermalStateString = @"CRITICAL";
        }
        NSString* theMessage = [NSString stringWithFormat:@"Thermal state: %@", thermalStateString];
        NSString* message = NSLocalizedString(theMessage, @"Alert message when thermal state has changed");
        UIAlertController* alertController = [UIAlertController alertControllerWithTitle:@"BodyFusion" message:message preferredStyle:UIAlertControllerStyleAlert];
        UIAlertAction* cancelAction = [UIAlertAction actionWithTitle:NSLocalizedString(@"OK", @"Alert OK button") style:UIAlertActionStyleCancel handler:nil];
        [alertController addAction:cancelAction];
        [self presentViewController:alertController animated:YES completion:nil];
    });
}

// MARK: -  KVO and Notifications

- (void) addObservers
{
    [session addObserver:self forKeyPath:@"running" options:NSKeyValueObservingOptionNew context:SessionRunningContext];

    [[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(thermalStateChanged:) name:NSProcessInfoThermalStateDidChangeNotification object:nil];
}

- (void) removeObservers
{
    [[NSNotificationCenter defaultCenter] removeObserver:self];
    
    [session removeObserver:self forKeyPath:@"running" context:SessionRunningContext];
}

- (void) observeValueForKeyPath:(NSString*)keyPath
                       ofObject:(id)object
                         change:(NSDictionary*)change
                        context:(void*)context
{
    if (context != SessionRunningContext) {
        [super observeValueForKeyPath:keyPath ofObject:object change:change context:context];
    }
}

// MARK: - Session Management

// Call this on the sessionQueue.
- (void) configureSession
{
    [session beginConfiguration];
    
    // Remove existing inputs and outputs
    for (AVCaptureInput *input in session.inputs) {
        [session removeInput:input];
    }
    for (AVCaptureOutput *output in session.outputs) {
        [session removeOutput:output];
    }
    
    _videoOutput = [AVCaptureVideoDataOutput new];
    _depthOutput = [AVCaptureDepthDataOutput new];
    
    authWasSuccessful = false;
    
    _videoDeviceDiscoverySession = [AVCaptureDeviceDiscoverySession discoverySessionWithDeviceTypes:@[AVCaptureDeviceTypeBuiltInTrueDepthCamera]
                                                                                         mediaType:AVMediaTypeVideo position:AVCaptureDevicePositionFront];
    auto status = [AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo];
    
    if ( status != AVAuthorizationStatusAuthorized) {
        dispatch_suspend(self.sessionQueue);
        [AVCaptureDevice requestAccessForMediaType:AVMediaTypeVideo completionHandler:^(BOOL granted) {
            authWasSuccessful = granted;
            if ( granted ) {
                dispatch_resume(self.sessionQueue);
            }
        }];
    } else {
        authWasSuccessful = true;
    }

    if ( !authWasSuccessful ) { return; }

    auto defaultVideoDevice = [_videoDeviceDiscoverySession devices].firstObject;
    
    if ( defaultVideoDevice == NULL ) {
        NSLog(@"No video device found!");
        return;
    }
    
    NSError * error = NULL;
    _videoDeviceInput = [AVCaptureDeviceInput deviceInputWithDevice:defaultVideoDevice error:&error];
    if ( error != NULL ) {
        NSLog(@"Error with video device!");
        return;
    }
    
    session.sessionPreset = AVCaptureSessionPreset1920x1080;
    [session addInput:_videoDeviceInput];
    [session addOutput:_videoOutput];
    auto settings = [NSDictionary dictionaryWithObject:[NSNumber numberWithInt:kCVPixelFormatType_32BGRA]
                                                forKey:(id)kCVPixelBufferPixelFormatTypeKey];
    _videoOutput.videoSettings = settings;

    [session addOutput:_depthOutput];
    [_depthOutput setFilteringEnabled:false];
    
    auto connection = [_depthOutput connectionWithMediaType:AVMediaTypeDepthData];
    [connection setEnabled:true];
    
    auto formats = defaultVideoDevice.activeFormat.supportedDepthDataFormats;
    int bestFormatIdx=-1;
    int idx = 0;
    for (AVCaptureDeviceFormat* format in formats) {
        auto dims = CMVideoFormatDescriptionGetDimensions(format.formatDescription);
        if ( CMFormatDescriptionGetMediaSubType(format.formatDescription) == kCVPixelFormatType_DepthFloat16 &&
             dims.width == 640 ) {
            bestFormatIdx = idx;
            NSLog(@"Got good depth format: %i ", bestFormatIdx);
        }
        idx ++;
    }
    
    if ( bestFormatIdx != -1 ) {
        [defaultVideoDevice lockForConfiguration:NULL];
        [defaultVideoDevice setActiveDepthDataFormat:formats[bestFormatIdx]];
        [defaultVideoDevice unlockForConfiguration];
    } else {
        NSLog(@"Could not find good depth format!");
        return;
    }

    _dataOutputSynchronizer = [[AVCaptureDataOutputSynchronizer alloc] initWithDataOutputs:@[_videoOutput, _depthOutput]];
    [_dataOutputSynchronizer setDelegate:self queue:self.dataOutputQueue];
    
    [session commitConfiguration];
}

- (void) setupEngine
{
    calibrated = false;
    setupDone = false;
    fullProcess = false;
    isRecording = false;
    
    currentFrameNo = 0;
    saveFrameNo = 0;
    numFusionButtonClicked = 0;
    
    self.context = [MetalContext instance];
    
    NSArray* dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    docsPath = (char*)[[dirPaths objectAtIndex:0]cStringUsingEncoding:[NSString defaultCStringEncoding]];
    memcpy(documentsPath, docsPath, strlen(docsPath));
    
    NSError* error;
    NSString* dataPath = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:@"/Output"];
    if (![[NSFileManager defaultManager] fileExistsAtPath:dataPath])
        [[NSFileManager defaultManager] createDirectoryAtPath:dataPath withIntermediateDirectories:NO attributes:nil error:&error];
    
    NSLog(@"fullProcess before: %d", fullProcess);

    if (self.setupResult == SetupResultSuccess)
    {
        fullProcess = false;
        [self.tbOut setText:@"front depth"];
        
        [_motionManager startDeviceMotionUpdates];
        
        imuMeasurement = new ITMIMUMeasurement();
        
        // 使用竖屏尺寸初始化标定源
        imageSource = new iPhoneSource(Vector2i(360, 640));
        rgbLargeSource = new iPhoneSource(Vector2i(1080, 1920));
        
        inputFakeRGBImage = new ITMUChar4Image(imageSource->getRGBImageSize(), true, false);
        inputRawDepthImage = new ITMShortImage(imageSource->getDepthImageSize(), true, false);
        inputLargeRGBImage = new ITMUChar4Image(rgbLargeSource->getRGBImageSize(), true, false);

        rgbLargeImageSize = rgbLargeSource->getRGBImageSize();
        depthImageSize = imageSource->getDepthImageSize();
        usingSensor = true;

        result = new ITMUChar4Image(imageSource->getDepthImageSize(), false);
        depthSpace = CGColorSpaceCreateDeviceRGB();
        internalSettings = new ITMLibSettings();
        mainEngine = new ITMMainEngine(internalSettings, &imageSource->calib, imageSource->getRGBImageSize(), imageSource->getDepthImageSize());
        
        setupDone = true;
    }


}

// MARK: - Navigation
- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender {
    // Get the new view controller using [segue destinationViewController].
    // Pass the point cloud to the new view controller.
    if ([segue.identifier isEqual: @"Show3D"]) {
        NSLog(@"Preparing for Show3D segue");
        PointCloudViewController* pointCloudVC = segue.destinationViewController;
        NSString *modelPath = [self saveModel:numFusionButtonClicked/2];
        NSLog(@"Setting model path in PointCloudViewController: %@", modelPath);
        pointCloudVC.modelPath = modelPath;
        
        // Stop processing depth data
        setupDone = false;
        fullProcess = false;
        
        // Don't hide navigation bar when transitioning to PointCloudViewController
        [self.navigationController setNavigationBarHidden:NO animated:YES];
        
        NSLog(@"Segue preparation completed");
    }
}

// MARK: - Actions

// TODO: hold button to record, release to stop record. click button to start/stop fusion.

- (IBAction)bProcessOne_clicked:(id)sender
{
    if (usingSensor)
    {
        isRecording = !isRecording;
        currentFrameNo = 0;
        saveFrameNo = 0;
        return;
    }
    
    if (!imageSource->hasMoreImages()) return;
    
    imageSource->getImages(inputFakeRGBImage, inputRawDepthImage);
    
    dispatch_async(self.renderingQueue, ^{
        [self updateImage];
    });
}

- (IBAction)bProcessCont_clicked:(id)sender
{
    ++numFusionButtonClicked;
    NSLog(@"Fusion button clicked %d times, usingSensor: %d", numFusionButtonClicked, usingSensor);
    
    // If usingSensor, update UI when sensorDidOutputDepthFrame()
    if (usingSensor)
    {
        // When click every the second time, end fullProcess and navigation to PointCloudViewController
        fullProcess = !fullProcess;
        if (numFusionButtonClicked > 0 && numFusionButtonClicked % 2 == 0) {
            NSLog(@"Navigating to 3D view");
            setupDone = false;  // Reset setupDone flag
            
            // 打印跳转 
            NSLog(@"Attempting to perform segue");
            [self performSegueWithIdentifier:@"Show3D" sender:sender];
            NSLog(@"Segue performed");
        }
        return;
    }
    
    // If not usingSensor, using RawFileReader imageSource Engine, update UI in while loop in renderingQueue.
    // Read previously saved rgb and depth image from documentsPath/Out (following named currentFrameNo), process and then render to UI.
    // Process and update each frame in another thread other than main_queue
    // dispatch_async allows to operate (e.g. click in main UI) in main thread 1, while processing frames in renderingQueue in another thread.
    dispatch_async(self.renderingQueue, ^{
        while (imageSource->hasMoreImages() && imuSource->hasMoreMeasurements())
        {
            imageSource->getImages(inputFakeRGBImage, inputRawDepthImage);
            imuSource->getMeasurement(imuMeasurement);
            [self updateImage];
        }
    });
}

- (NSString*) saveModel:(int)index
{
    char modelPath[1000];
    sprintf(modelPath, "%s/model%04d.stl", documentsPath, index);
    mainEngine->SaveSceneToMesh(modelPath);
    return [NSString stringWithCString:modelPath encoding:NSUTF8StringEncoding];
}

// MARK: - UI Utility Functions

// Call this on the renderingQueue.
- (void) updateImage
{
    if (fullProcess) mainEngine->turnOnMainProcessing();
    else mainEngine->turnOffMainProcessing();
        
    NSDate *timerStart = [NSDate date];
    
    // 打印输入数据的统计信息
    short *depthData = inputRawDepthImage->GetData(MEMORYDEVICE_CPU);
    
    // 打印图像尺寸信息
    NSLog(@"Image dimensions - width: %d, height: %d", 
          inputRawDepthImage->noDims.x, 
          inputRawDepthImage->noDims.y);
    
    // 使用与存储时相同的索引计算方式
    int centerX = inputRawDepthImage->noDims.x / 2;
    int centerY = inputRawDepthImage->noDims.y / 2;
    int centerIdx = centerX * inputRawDepthImage->noDims.y + centerY;  // 使用 x * height + y
    
    NSLog(@"Before ProcessFrame - Center coordinates: (%d,%d), index: %d", 
          centerX, centerY, centerIdx);
    NSLog(@"Before ProcessFrame - Center depth: FP16=%d, float=%f", 
          depthData[centerIdx], 
          *(float*)&depthData[centerIdx]);
    
    if (imuMeasurement != NULL) 
        // 打印使用imuMeasurement
        {
            NSLog(@"Using imuMeasurement");
            mainEngine->ProcessFrame(inputFakeRGBImage, inputRawDepthImage, imuMeasurement);
        } 
    else 
        mainEngine->ProcessFrame(inputFakeRGBImage, inputRawDepthImage);
    
    NSDate *timerStop = [NSDate date];
    NSTimeInterval executionTime = [timerStop timeIntervalSinceDate:timerStart];
    
    if (fullProcess)
    {
        totalProcessedFrames++;
        totalProcessingTime += executionTime;
    }
    
    // Get and update output rendered raycasted image.
    if (fullProcess) mainEngine->GetImage(result, ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST);
    else mainEngine->GetImage(result, ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH);
    
    //打印result中间像素的值
    NSLog(@"Result dimensions: %d x %d", result->noDims.x, result->noDims.y);
    NSLog(@"result中间像素的值: %d %d %d %d", 
          result->GetData(MEMORYDEVICE_CPU)[result->noDims.x/2 * result->noDims.y + result->noDims.y/2].r,
          result->GetData(MEMORYDEVICE_CPU)[result->noDims.x/2 * result->noDims.y + result->noDims.y/2].g,
          result->GetData(MEMORYDEVICE_CPU)[result->noDims.x/2 * result->noDims.y + result->noDims.y/2].b,
          result->GetData(MEMORYDEVICE_CPU)[result->noDims.x/2 * result->noDims.y + result->noDims.y/2].a);
    CGContextRef cgContext = CGBitmapContextCreate(result->GetData(MEMORYDEVICE_CPU), depthImageSize.x, depthImageSize.y, 8,
                                                   4 * depthImageSize.x, depthSpace, kCGImageAlphaNoneSkipLast);
    CGImageRef cgImageRef = CGBitmapContextCreateImage(cgContext);
    
    dispatch_sync(dispatch_get_main_queue(), ^{
        self.renderView.layer.contents = (__bridge id)cgImageRef;
         if (fullProcess && !isRecording) {
            NSString *theValue = [NSString stringWithFormat:@"%5.4lf spf", totalProcessingTime / totalProcessedFrames];
            [self.tbOut setText:theValue];
        }
    });

    CGImageRelease(cgImageRef);
    CGContextRelease(cgContext);
}

-(void) updateRGBImage:(CVPixelBufferRef)pixelBuffer {
    
    CVPixelBufferLockBaseAddress(pixelBuffer, NULL);
    
    // 获取输入图像尺寸
    int inputWidth = (int)CVPixelBufferGetWidth(pixelBuffer);    // 1920
    int inputHeight = (int)CVPixelBufferGetHeight(pixelBuffer);  // 1080
    int bytesPerRow = (int)CVPixelBufferGetBytesPerRow(pixelBuffer);
    
    NSLog(@"Input image dimensions: %d x %d", inputWidth, inputHeight);
    
    uint8_t *ptr = (uint8_t *)inputLargeRGBImage->GetData(MEMORYDEVICE_CPU);
    const uint8_t* pixelData = (const uint8_t*)CVPixelBufferGetBaseAddress(pixelBuffer);
    
    // 遍历输入图像的每个像素
    for (int y = 0; y < inputHeight; y++) {
        for (int x = 0; x < inputWidth; x++) {
            // 计算输入图像中的索引
            int src_idx = y * bytesPerRow + x * 4;
            
            // 获取原始像素值
            int b = pixelData[src_idx];     // Blue
            int g = pixelData[src_idx+1];   // Green
            int r = pixelData[src_idx+2];   // Red
            int a = pixelData[src_idx+3];   // Alpha
            
            // 计算旋转后的位置
            // 顺时针旋转90度并保持正确的上下左右方向：newX = y, newY = x
            int newX = y;
            int newY = x;
            
            // 计算输出图像中的索引
            int dst_idx = (newY * inputHeight + newX) * 4;
            
            // 存储为RGBA顺序
            ptr[dst_idx] = r;     // Red
            ptr[dst_idx+1] = g;   // Green
            ptr[dst_idx+2] = b;   // Blue
            ptr[dst_idx+3] = a;   // Alpha
            
            // 打印中间像素的值
            if (x == inputWidth/2 && y == inputHeight/2) {
                NSLog(@"原始中间像素的RGB值: %d %d %d", r, g, b);
                NSLog(@"存储后的RGB值: %d %d %d", ptr[dst_idx], ptr[dst_idx+1], ptr[dst_idx+2]);
            }
        }
    }
    
    CVPixelBufferUnlockBaseAddress(pixelBuffer, NULL);

}

// MARK: - AVCaptureDataOutputSynchronizerDelegate
static int frameIndex = 0;

- (void)dataOutputSynchronizer:(nonnull AVCaptureDataOutputSynchronizer *)synchronizer 
        didOutputSynchronizedDataCollection:(nonnull AVCaptureSynchronizedDataCollection *)synchronizedDataCollection
{
    frameIndex++;
   if (setupDone)
    {
        CMRotationMatrix rotationMatrix = self.motionManager.deviceMotion.attitude.rotationMatrix;
        
        if (imuMeasurement != NULL)
        {
            imuMeasurement->R.m00 = rotationMatrix.m11; imuMeasurement->R.m01 = rotationMatrix.m12; imuMeasurement->R.m02 = rotationMatrix.m13;
            imuMeasurement->R.m10 = rotationMatrix.m21; imuMeasurement->R.m11 = rotationMatrix.m22; imuMeasurement->R.m12 = rotationMatrix.m23;
            imuMeasurement->R.m20 = rotationMatrix.m31; imuMeasurement->R.m21 = rotationMatrix.m32; imuMeasurement->R.m22 = rotationMatrix.m33;
        }
        // Calibration iPhone depth first and only once, intrinsic reference to depthImageSize(320, 180).
        // TODO: depth distortion correction.
        AVCaptureSynchronizedDepthData* syncedDepthData =
        (AVCaptureSynchronizedDepthData*)[synchronizedDataCollection synchronizedDataForCaptureOutput:_depthOutput];
    
        AVCaptureSynchronizedSampleBufferData * syncedVideoData =
            (AVCaptureSynchronizedSampleBufferData *)[synchronizedDataCollection synchronizedDataForCaptureOutput:_videoOutput];
        
        if ( syncedDepthData.depthDataWasDropped || syncedVideoData.sampleBufferWasDropped ) {
            return;
        }
        auto depthData = syncedDepthData.depthData;

        if (!calibrated)
        {
            AVCameraCalibrationData* calibData = depthData.cameraCalibrationData;
            if (!calibData) {
                NSLog(@"Error: No calibration data available");
                return;
            }
            
            matrix_float3x3 intrinsics = calibData.intrinsicMatrix;
            CGSize referenceDimensions = calibData.intrinsicMatrixReferenceDimensions;
            // Original intrinsics: [2743.347168 0.000000 0.000000; 0.000000 2743.347168 0.000000; 2019.957275 1129.517822 1.000000]

            NSLog(@"Original intrinsics: [%f %f %f; %f %f %f; %f %f %f]",
                  intrinsics.columns[0][0], intrinsics.columns[0][1], intrinsics.columns[0][2],
                  intrinsics.columns[1][0], intrinsics.columns[1][1], intrinsics.columns[1][2],
                  intrinsics.columns[2][0], intrinsics.columns[2][1], intrinsics.columns[2][2]);
            NSLog(@"Reference dimensions: %f x %f", referenceDimensions.width, referenceDimensions.height);
            
            // Get the actual depth buffer dimensions
            auto depthBuffer = depthData.depthDataMap;
            size_t bufferWidth = CVPixelBufferGetWidth(depthBuffer);
            size_t bufferHeight = CVPixelBufferGetHeight(depthBuffer);
            NSLog(@"Depth buffer dimensions: %zu x %zu", bufferWidth, bufferHeight);
            
            if (bufferWidth == 0 || referenceDimensions.width == 0) {
                NSLog(@"Error: Invalid dimensions for ratio calculation");
                return;
            }
            
            float ratio = referenceDimensions.width / bufferWidth;
            NSLog(@"Scaling ratio: %f", ratio);
            
            if (isnan(ratio) || isinf(ratio) || ratio <= 0) {
                NSLog(@"Error: Invalid ratio: %f", ratio);
                return;
            }
            
            // Scale the intrinsic matrix
            intrinsics.columns[0][0] /= ratio;
            intrinsics.columns[1][1] /= ratio;
            intrinsics.columns[2][0] /= ratio;
            intrinsics.columns[2][1] /= ratio;
            
            float fy = intrinsics.columns[0][0];
            float fx = intrinsics.columns[1][1];
            float cy = intrinsics.columns[2][0];
            float cx = intrinsics.columns[2][1];
            
            //Scaled calibration parameters: fx=435.451904, fy=435.451904, cx=179.288544, cy=320.628143
            NSLog(@"Scaled calibration parameters: fx=%f, fy=%f, cx=%f, cy=%f", fx, fy, cx, cy);
            
            // 将内参存储到成员变量中
            intrinsics_vector = {fx, fy, cx, cy};
            
            if (isnan(fx) || isnan(fy) || isnan(cx) || isnan(cy)) {
                NSLog(@"Error: NaN values in calibration parameters");
                return;
            }
            
            ((iPhoneSource*)imageSource)->calibrate(fy, fx, cx, cy, 1);
            calibrated = true;
        }

        auto sampleBuffer = syncedVideoData.sampleBuffer;
        
        auto pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
        // 打印pixelBuffer的尺寸
        NSLog(@"pixelBuffer 尺寸: %d %d", CVPixelBufferGetWidth(pixelBuffer), CVPixelBufferGetHeight(pixelBuffer));
        
        // 保留pixelBuffer的引用计数
        CVPixelBufferRetain(pixelBuffer);
        
        [self updateRGBImage: pixelBuffer];
        
        // 释放pixelBuffer
        CVPixelBufferRelease(pixelBuffer);
        
        //
        auto depthBuffer = depthData.depthDataMap;
        
        //float16_t 原始的是640 X 360
        size_t width = CVPixelBufferGetWidth(depthBuffer);
        size_t height = CVPixelBufferGetHeight(depthBuffer);
        // stride 是每行的字节数
        size_t stride = CVPixelBufferGetBytesPerRow(depthBuffer);
        
        //NSLog(@" depth frame: %i x %i ", (int)width, (int)height);
        
        size_t pixelBpr = CVPixelBufferGetBytesPerRow(pixelBuffer);
        
        CVPixelBufferLockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
        CVPixelBufferLockBaseAddress(depthBuffer, kCVPixelBufferLock_ReadOnly);
        //baseAddress 是 uint8_t* 类型（8位无符号整数指针）的原因与内存访问和字节对齐有关：
        // 内存的基本单位：计算机内存是以字节（8位）为基本单位进行寻址的,uint8_t 正好是一个字节，可以访问内存中的任意位置
        // 使用 uint8_t* 可以精确控制内存访问，不受数据类型大小的限制.
        const uint8_t* baseAddress = (const uint8_t*)CVPixelBufferGetBaseAddress(depthBuffer);
                
        // TrueDepth 相机输出的深度数据是 FP16（16位浮点数）格式
        // ITMShortImage 类使用 short（16位有符号整数）作为其数据类型
        // 虽然类型是 short，但实际上我们用它来存储 FP16 的位模式
        // 这就是为什么我们可以通过 *((short*)&depth_fp16) 来存储 FP16 值
        uint16_t * ptr = (uint16_t *)inputRawDepthImage->GetData(MEMORYDEVICE_CPU);
        
        const int step = 1;
        
        for (size_t y = 0; y < height; y+=step) {
            // 通过 baseAddress + y * stride 可以精确定位到每一行的起始位置
            // 然后将指针转换为 __fp16* 来读取深度值
            const __fp16* data = (const __fp16*)(baseAddress + y * stride);
            
            for (size_t x = 0; x < width; x+=step, data+=step) {
                // data 是 __fp16* 类型
                // 当执行 data += step 时，指针会自动移动 sizeof(__fp16) 个字节
                __fp16 depth2 = *data;
                float depth = depth2; //float 是 32位
                
                // 处理无效深度值并限制最大深度为0.4米
                if (isnan(depth) || depth < minDepth || depth > maxDepth) {
                    depth = -1.0f;  // 使用-1.0f表示无效深度
                }
                
                // 将深度值转换为 FP16 格式
                __fp16 depth_fp16 = (__fp16)depth;

                // 计算旋转后的位置
                // 顺时针旋转90度并保持正确的上下左右方向：newX = y, newY = x
                int newX = y;
                int newY = x;
                
                // 计算输出图像中的索引
                int ptr_idx = (newY * height + newX) ;

                ptr[ptr_idx] = *((short*)&depth_fp16);
                
                // 立即验证更新后的值
                if (x == width/2 && y == height/2) {
                    NSLog(@"Before updating inputRawDepthImage - Center depth (FP16): %d", ptr[ptr_idx]);
                    
                    // 正确转换 FP16 到 float
                    __fp16 fp16_value = *(__fp16*)&ptr[ptr_idx];
                    float float_value = fp16_value;
                    
                    NSLog(@"After updating inputRawDepthImage - Center depth (float): %f", float_value);
                }
            }
        }

        // 确保解锁缓冲区
        CVPixelBufferUnlockBaseAddress(depthBuffer, kCVPixelBufferLock_ReadOnly);
        CVPixelBufferUnlockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);

        dispatch_async(self.renderingQueue, ^{
             if (isRecording)
            {  
                // 每5帧保存一次，减少保存频率
                if (currentFrameNo % 5 != 0) {
                    currentFrameNo++;
                    return;
                }
                // If recording, save sensor captured depth images and IMU rotationMatrix to documentsPath/Out naming currentFrameNo
                FILE *f_depth; char fileName_depth[2000];
                FILE *f_img; char fileName_img[2000];
                FILE *f_ply; char fileName_ply[2000];

                // 1. 保存深度图
                sprintf(fileName_depth, "%s/Output/depth_%08d.png", documentsPath, saveFrameNo);
                f_depth = fopen(fileName_depth, "wb+");
                
                // 获取深度图数据
                uint16_t *depthPtr = (uint16_t *)inputRawDepthImage->GetData(MEMORYDEVICE_CPU);
                if (depthPtr == NULL) {
                    NSLog(@"Failed to get depth data");
                    fclose(f_depth);
                    return;
                }
                
                size_t depthWidth = inputRawDepthImage->noDims.x;
                size_t depthHeight = inputRawDepthImage->noDims.y;
                // 直接写入原始深度数据
                fwrite(depthPtr, sizeof(uint16_t), depthWidth * depthHeight, f_depth);
                /*

                // 创建深度图位图上下文
                CGColorSpaceRef depthColorSpace = CGColorSpaceCreateDeviceGray();
                if (depthColorSpace == NULL) {
                    NSLog(@"Failed to create depth color space");
                    fclose(f_depth);
                    return;
                }
                
                CGContextRef depthContext = CGBitmapContextCreate(
                    depthPtr,
                    depthWidth,
                    depthHeight,
                    16, // bits per component
                    depthWidth * 2, // bytes per row
                    depthColorSpace,
                    kCGImageAlphaNone
                );
                
                if (depthContext == NULL) {
                    NSLog(@"Failed to create depth bitmap context");
                    CGColorSpaceRelease(depthColorSpace);
                    fclose(f_depth);
                    return;
                }
                
                // 创建深度图CGImage
                CGImageRef depthImageRef = CGBitmapContextCreateImage(depthContext);
                if (depthImageRef == NULL) {
                    NSLog(@"Failed to create depth CGImage");
                    CGContextRelease(depthContext);
                    CGColorSpaceRelease(depthColorSpace);
                    fclose(f_depth);
                    return;
                }
                
                // 创建深度图UIImage
                UIImage *depthUIImage = [UIImage imageWithCGImage:depthImageRef];
                if (depthUIImage == NULL) {
                    NSLog(@"Failed to create depth UIImage");
                    CGImageRelease(depthImageRef);
                    CGContextRelease(depthContext);
                    CGColorSpaceRelease(depthColorSpace);
                    fclose(f_depth);
                    return;
                }
                
                // 保存深度图到文件
                NSData *depthPngData = UIImagePNGRepresentation(depthUIImage);
                if (depthPngData == NULL) {
                    NSLog(@"Failed to create depth PNG data");
                    CGImageRelease(depthImageRef);
                    CGContextRelease(depthContext);
                    CGColorSpaceRelease(depthColorSpace);
                    fclose(f_depth);
                    return;
                }
                
                fwrite([depthPngData bytes], [depthPngData length], 1, f_depth);
                NSLog(@"Depth image saved to %s", fileName);
                
                // 清理深度图资源
                CGImageRelease(depthImageRef);
                CGContextRelease(depthContext);
                CGColorSpaceRelease(depthColorSpace);
                */
                fclose(f_depth);
                
                // 2. 保存 RGB 图
                sprintf(fileName_img, "%s/Output/rgb_%08d.png", documentsPath, saveFrameNo);
                f_img = fopen(fileName_img, "wb+");
                if (f_img == NULL) {
                    NSLog(@"Failed to open RGB file for writing");
                    return;
                }
                
                // 获取RGB图像数据
                Vector4u *rgbPtr = inputLargeRGBImage->GetData(MEMORYDEVICE_CPU);
                if (rgbPtr == NULL) {
                    NSLog(@"Failed to get RGB data");
                    fclose(f_img);
                    return;
                }
                
                size_t largeRGBWidth = inputLargeRGBImage->noDims.x;
                size_t largeRGBHeight = inputLargeRGBImage->noDims.y;
                
                // 创建RGB位图上下文
                CGColorSpaceRef rgbColorSpace = CGColorSpaceCreateDeviceRGB();
                if (rgbColorSpace == NULL) {
                    NSLog(@"Failed to create RGB color space");
                    fclose(f_img);
                    return;
                }
                
                CGContextRef rgbContext = CGBitmapContextCreate(
                    rgbPtr,
                    largeRGBWidth,
                    largeRGBHeight,
                    8, // bits per component
                    largeRGBWidth * 4, // bytes per row
                    rgbColorSpace,
                    kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big
                );
                
                if (rgbContext == NULL) {
                    NSLog(@"Failed to create RGB bitmap context");
                    CGColorSpaceRelease(rgbColorSpace);
                    fclose(f_img);
                    return;
                }
                
                // 创建RGB CGImage
                CGImageRef rgbImageRef = CGBitmapContextCreateImage(rgbContext);
                if (rgbImageRef == NULL) {
                    NSLog(@"Failed to create RGB CGImage");
                    CGContextRelease(rgbContext);
                    CGColorSpaceRelease(rgbColorSpace);
                    fclose(f_img);
                    return;
                }
                
                // 创建RGB UIImage
                UIImage *rgbUIImage = [UIImage imageWithCGImage:rgbImageRef];
                if (rgbUIImage == NULL) {
                    NSLog(@"Failed to create RGB UIImage");
                    CGImageRelease(rgbImageRef);
                    CGContextRelease(rgbContext);
                    CGColorSpaceRelease(rgbColorSpace);
                    fclose(f_img);
                    return;
                }
                CFURLRef fileURL = CFURLCreateFromFileSystemRepresentation(NULL, (const UInt8 *)fileName_img, strlen(fileName_img), false);
                CGImageDestinationRef dest = CGImageDestinationCreateWithURL(fileURL, kUTTypePNG, 1, NULL);
                CGImageDestinationAddImage(dest, rgbImageRef, NULL);
                CGImageDestinationFinalize(dest);
                CFRelease(dest);
                CFRelease(fileURL);

                // 保存RGB图到文件
                // NSData *rgbPngData = UIImagePNGRepresentation(rgbUIImage);
                // if (rgbPngData == NULL) {
                //     NSLog(@"Failed to create RGB PNG data");
                //     CGImageRelease(rgbImageRef);
                //     CGContextRelease(rgbContext);
                //     CGColorSpaceRelease(rgbColorSpace);
                //     fclose(f_img);
                //     return;
                // }
                
                // fwrite([rgbPngData bytes], [rgbPngData length], 1, f_img);
                NSLog(@"RGB image saved to %s", fileName_img);
                
                // 清理RGB资源
                CGImageRelease(rgbImageRef);
                CGContextRelease(rgbContext);
                CGColorSpaceRelease(rgbColorSpace);
                fclose(f_img);
                
                bool is_save_ply = false;
                // 3. 生成并保存点云
                if (is_save_ply) {
                    sprintf(fileName_ply, "%s/Output/pointcloud_%08d.ply", documentsPath, saveFrameNo);
                    f_ply = fopen(fileName_ply, "w");  // 使用文本模式打开文件
                    
                    // 写入 PLY 文件头
                    fprintf(f_ply, "ply\n");
                    fprintf(f_ply, "format ascii 1.0\n");
                    fprintf(f_ply, "comment Generated by InfiniTAM\n");
                    
                    // 计算有效点的数量
                    int validPointCount = 0;
                    for (int x = 0; x < depthImageSize.x; x++) {
                        for (int y = 0; y < depthImageSize.y; y++) {
                            int idx = y * depthImageSize.x + x;
                            short rawDepth = depthPtr[idx];
                            __fp16 fp16_value = *(__fp16*)&rawDepth;
                            float depthInMeters = fp16_value;
                            if (depthInMeters >= 0.01f && depthInMeters <= 0.4f) {
                                validPointCount++;
                            }
                        }
                    }
                    
                    fprintf(f_ply, "element vertex %d\n", validPointCount);
                    fprintf(f_ply, "property float x\n");
                    fprintf(f_ply, "property float y\n");
                    fprintf(f_ply, "property float z\n");
                    fprintf(f_ply, "property uchar red\n");
                    fprintf(f_ply, "property uchar green\n");
                    fprintf(f_ply, "property uchar blue\n");
                    fprintf(f_ply, "end_header\n");
                    
                    // 获取相机内参
                    float fx = intrinsics_vector[0];
                    float fy = intrinsics_vector[1];
                    float cx = intrinsics_vector[2];
                    float cy = intrinsics_vector[3];
                    
                    NSLog(@"点云生成使用的内参:");
                    NSLog(@"fx: %f, fy: %f", fx, fy);
                    NSLog(@"cx: %f, cy: %f", cx, cy);
                    
                    // 下采样 RGB 图像到深度图分辨率
                    uint8_t* downsampledRGB = (uint8_t*)malloc(depthImageSize.x * depthImageSize.y * 4);
                    Vector4u *rgbData = inputLargeRGBImage->GetData(MEMORYDEVICE_CPU);
                    
                    // 简单的最近邻下采样
                    float rgbScaleX = (float)rgbLargeImageSize.x / depthImageSize.x;
                    float rgbScaleY = (float)rgbLargeImageSize.y / depthImageSize.y;
                    
                    for (int y = 0; y < depthImageSize.y; y++) {
                        for (int x = 0; x < depthImageSize.x; x++) {
                            int srcX = (int)(x * rgbScaleX);
                            int srcY = (int)(y * rgbScaleY);
                            int srcIdx = srcY * rgbLargeImageSize.x + srcX;
                            int dstIdx = (y * depthImageSize.x + x) * 4;
                            
                            downsampledRGB[dstIdx] = rgbData[srcIdx].r;     // R
                            downsampledRGB[dstIdx + 1] = rgbData[srcIdx].g; // G
                            downsampledRGB[dstIdx + 2] = rgbData[srcIdx].b; // B
                            downsampledRGB[dstIdx + 3] = rgbData[srcIdx].a; // A
                        }
                    }
                    /*
                    // 保存downsampledRGB图像到文件
                    sprintf(fileName, "%s/Output/downsampledRGB_%08d.png", documentsPath, currentFrameNo);
                    f = fopen(fileName, "wb+");
                    CGColorSpaceRef rgbColorSpace_downsampled = CGColorSpaceCreateDeviceRGB();
                    CGContextRef rgbContext_downsampled = CGBitmapContextCreate(
                        downsampledRGB,
                        depthImageSize.x,
                        depthImageSize.y,
                        8,
                        depthImageSize.x * 4,
                        rgbColorSpace_downsampled,
                        kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big
                    );  
                    CGImageRef rgbImageRef_downsampled = CGBitmapContextCreateImage(rgbContext_downsampled);
                    UIImage *rgbUIImage_downsampled = [UIImage imageWithCGImage:rgbImageRef_downsampled];
    
                    NSData *rgbPngData_downsampled = UIImagePNGRepresentation(rgbUIImage_downsampled);
                    fwrite([rgbPngData_downsampled bytes], [rgbPngData_downsampled length], 1, f);
                    NSLog(@"RGB image saved to %s", fileName);
                    CGImageRelease(rgbImageRef_downsampled);
                    CGContextRelease(rgbContext_downsampled);
                    CGColorSpaceRelease(rgbColorSpace_downsampled);
                    fclose(f);
                    */

                    // 生成点云                
                    for (int x = 0; x < depthImageSize.x; x++) {
                        for (int y = 0; y < depthImageSize.y; y++) {
                            int idx = y * depthImageSize.x + x;
                            
                            short rawDepth = depthPtr[idx];
                            __fp16 fp16_value = *(__fp16*)&rawDepth;
                            float depthInMeters = fp16_value;
                            
                            if (x == depthImageSize.x/2 && y == depthImageSize.y/2) {
                                NSLog(@"最中间像素的点云深度: FP16=%d, float=%f", rawDepth, depthInMeters);
                            }   
                            
                            // 只处理有效的深度值（0.01m 到 0.4m）
                            if (depthInMeters >= 0.01f && depthInMeters <= 0.4f) {
                                // 计算归一化坐标
                                float normalizedX = (x - cx) / fx;
                                float normalizedY = (y - cy) / fy;
                                
                                // 计算 3D 点坐标
                                float X = normalizedX * depthInMeters;
                                float Y = normalizedY * depthInMeters;
                                float Z = depthInMeters;
                                
                                // 获取对应的 RGB 颜色
                                int rgbIdx = idx * 4;
                                int r = downsampledRGB[rgbIdx];
                                int g = downsampledRGB[rgbIdx + 1];
                                int b = downsampledRGB[rgbIdx + 2];
                                
                                // 写入点云数据，确保格式正确
                                fprintf(f_ply, "%.6f %.6f %.6f %d %d %d\n", X, Y, Z, r, g, b);
                            }
                        }
                    }
                    
                    fclose(f_ply);
                    free(downsampledRGB);
                }
                if (saveFrameNo % 5 == 0) {
                    dispatch_async(dispatch_get_main_queue(), ^{
                        NSString *theValue = [NSString stringWithFormat:@"record %d", saveFrameNo];
                        [self.tbOut setText:theValue];
                    });
                }
                currentFrameNo++;
                saveFrameNo++;
            }
            [self updateImage];
        });
        
    } // endif setupDone
    
}

@end
