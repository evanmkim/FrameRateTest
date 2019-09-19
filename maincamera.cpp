#include "maincamera.h"
#include <QString>
#include <iostream>
#include <string>
#include <algorithm>
#include <math.h>
#include "utils/qtpreviewconsumer.h"
#include "utils/Error.h"
#include "utils/Options.h"
#include <nvbuf_utils.h>
#include "EGLStream/NV/ImageNativeBuffer.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include "sys/types.h"
#include "sys/sysinfo.h"
#include <sys/time.h>
#include <unistd.h> //for sleep
#include <chrono> //for time
#include <sys/mman.h> //for mmap
#include "asmOpenCV.h"
#include "Argus/Ext/InternalFrameCount.h"

using namespace Argus;
using namespace std;


/*Program:Frame Rate Test
 *
 *Function: Purpose of this project is to try to conduct some testing on the Frame Rate of the sensor
 *One of the goal is to capture all the frames (Not just Acquired Frame and put them into a buffer,
 *Then do Image Processing to all of the Frames
 *
 * One attempt is to change the Output Stream Mode to FIFO (default mode is STREAM_MODE_MAILBOX)
 * Using the following code:
 *
 * iStreamSettings->setMode(STREAM_MODE_FIFO);
 * iStreamSettings->setFifoLength(5);
 *
 * You can refer to "Argus/Stream.h" for more information.
 *
 * The available options of the OutputStream modes are:
 *
 *   MAILBOX:
 *     In this mode, only the newest frame is made available to the consumer. When libargus
 *     completes a frame it empties the mailbox and inserts the new frame into the mailbox.
 *     The consumer then retrieves the frame from the mailbox and processes it; when
 *     finished, the frame is either placed back into the mailbox (if the mailbox is empty)
 *     or discarded (if the mailbox is not empty). This mode implies 2 things:
 *
 *       - If the consumer consumes frames slower than libargus produces frames, then some
 *         frames may be lost (never seen by the consumer).
 *
 *       - If the consumer consumes frames faster than libargus produces frames, then the
 *         consumer may see some frames more than once.
 *
 *   FIFO:
 *     When using this mode, every producer frame is made available to the consumer through
 *     the use of a fifo queue for the frames. When using this mode, the fifo queue length
 *     must be specified using setFifoLength. When libargus completes a frame it inserts it to
 *     the head of the fifo queue. If the fifo is full (already contains the number of frames
 *     equal to the fifo queue length), libargus will stall until the fifo is no longer
 *     full. The consumer consumes frames from the tail of the queue; however, if the
 *     consumer releases a frame while the queue is empty, the frame is set aside and will
 *     be returned again the next time the consumer requests a frame if another new frame
 *     has not been inserted into the fifo queue before then. Once a new frame is inserted
 *     into the fifo queue, any previously released frame will be permanently discarded.
 *     This mode implies:
 *
 *       - Frames are never discarded until the consumer has processed them.
 *
 *       - If the consumer consumes frames slower than libargus produces them,
 *         libargus will stall.
 *
 *       - If the consumer consumes frames faster than libargus produces them, then the
 *         consumer may see some frames more than once.
 *
 *   Default value: STREAM_MODE_MAILBOX
 */



//TRY READING IMAGE'S TIMESTAMP INTO A BUFFER..




MainCamera::MainCamera(QObject *parent) : QThread(parent)
{

}


void MainCamera::run()
{
    initCAM();
}

bool MainCamera::initCAM(){


    const int BUFFERSIZE = 4096;

    cout << "cameraDeviceIndex " << cameraDeviceIndex << endl;

    idx = 0;
    stopButtonPressed = false;
    uint32_t * buffer = new uint32_t [BUFFERSIZE];


/*
 * Set up Argus API Framework, identify available camera devices, create
 * a capture session for the first available device, and set up the event
 * queue for completed events
 */

///////////////////////////////////////////////////////////////
///Camera Provider
///////////////////////////////////////////////////////////////

    CameraProvider::create();
    UniqueObj<CameraProvider> cameraProvider(CameraProvider::create()); //We can put this as global
    ICameraProvider *iCameraProvider = interface_cast<ICameraProvider>(cameraProvider);
    EXIT_IF_NULL(iCameraProvider, "Cannot get core camera provider interface");
    printf("Argus Version: %s\n", iCameraProvider->getVersion().c_str());


///////////////////////////////////////////////////////////////
///Camera Device
///////////////////////////////////////////////////////////////

    std::vector<CameraDevice*> cameraDevices;
    EXIT_IF_NOT_OK(iCameraProvider->getCameraDevices(&cameraDevices),"Failed to get camera devices");
    EXIT_IF_NULL(cameraDevices.size(), "No camera devices available");
    cout << "There are " << cameraDevices.size() << " camera ports detected. " <<  endl;
    if (cameraDevices.size() <= cameraDeviceIndex)
    {
        printf("Camera device specified on the command line is not available\n");
        return EXIT_FAILURE;
    }

///////////////////////////////////////////////////////////////
///Set up session
///////////////////////////////////////////////////////////////

    UniqueObj<CaptureSession> captureSession(iCameraProvider->createCaptureSession(cameraDevices[cameraDeviceIndex]));
    ICaptureSession *iSession = interface_cast<ICaptureSession>(captureSession);
    EXIT_IF_NULL(iSession, "Cannot get Capture Session Interface");

/////////////////////////////////////////////////////////////////
///Events
/////////////////////////////////////////////////////////////////

    IEventProvider *iEventProvider = interface_cast<IEventProvider>(captureSession);
    EXIT_IF_NULL(iEventProvider, "iEventProvider is NULL");
    std::vector<EventType> eventTypes;
    eventTypes.push_back(EVENT_TYPE_CAPTURE_COMPLETE);
    UniqueObj<EventQueue> queue(iEventProvider->createEventQueue(eventTypes));
    IEventQueue *iQueue = interface_cast<IEventQueue>(queue);
    EXIT_IF_NULL(iQueue, "event queue interface is NULL");

/*
 * Creates the stream between the Argus camera image capturing
 * sub-system (producer) and the image acquisition code (consumer)
 * preview thread.  A consumer object is created from the stream
 * to be used to request the image frame.  A successfully submitted
 * capture request activates the stream's functionality to eventually
 * make a frame available for acquisition, in the preview thread,
 * and then display it on the device screen.
 */

/////////////////////////////////////////////////////////////////
///Output Stream Settings
/////////////////////////////////////////////////////////////////

    UniqueObj<OutputStreamSettings> streamSettings(iSession->createOutputStreamSettings());
    IOutputStreamSettings *iStreamSettings =interface_cast<IOutputStreamSettings>(streamSettings);
    EXIT_IF_NULL(iStreamSettings, "Cannot get OutputStreamSettings Interface");
    iStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
    iStreamSettings->setResolution(Size2D<uint32_t>(1920, 1080)); //1920, 1080 //640,480
    iStreamSettings->setEGLDisplay(g_display.get());
    iStreamSettings->setMode(STREAM_MODE_FIFO);
    iStreamSettings->setFifoLength(5);

/////////////////////////////////////////////////////////////////
///Output Stream
/////////////////////////////////////////////////////////////////

    //CREATE OUTPUT STREAM
    UniqueObj<OutputStream> stream(iSession->createOutputStream(streamSettings.get()));
    IStream *iStream = interface_cast<IStream>(stream);
    EXIT_IF_NULL(iStream, "Cannot get OutputStream Interface");

    ///METHOD 1: Using Frame Consumer to acquire each frame
    Argus::UniqueObj<EGLStream::FrameConsumer> consumer(EGLStream::FrameConsumer::create(stream.get(),5));
    EGLStream::IFrameConsumer *iFrameConsumer = Argus::interface_cast<EGLStream::IFrameConsumer>(consumer);
    EXIT_IF_NULL(iFrameConsumer, "Failed to initialize Consumer");

//    create(Argus::OutputStream* outputStream,
//    uint32_t maxFrames = 1,
//    Argus::Status* status = NULL);

/////////////////////////////////////////////////////////////////
///Request
/////////////////////////////////////////////////////////////////

    UniqueObj<Request> request(iSession->createRequest(CAPTURE_INTENT_MANUAL));
    IRequest *iRequest = interface_cast<IRequest>(request);
    iRequest = interface_cast<IRequest>(request);
    EXIT_IF_NULL(iRequest, "Failed to get capture request interface");

/////////////////////////////////////////////////////////////////
///Source Settings
/////////////////////////////////////////////////////////////////

    //INITIALIZE SOURCE SETTING INTERFACE TO GET SENSOR MODE
    Argus::ISourceSettings *iSourceSettings = Argus::interface_cast<Argus::ISourceSettings>(iRequest->getSourceSettings());
    EXIT_IF_NULL(iSourceSettings, "Failed to get source settings interface");

///////////////////////////////////////////////////////////////
///Camera Properties (For Storing Image)
///////////////////////////////////////////////////////////////

    //Declare iCameraProperties-> Store properties for storage session //1
    ICameraProperties *iCameraProperties = interface_cast<ICameraProperties>(cameraDevices[cameraDeviceIndex]);
    EXIT_IF_NULL(iCameraProperties, "Failed to get ICameraProperties interface");
    std::vector<SensorMode*> sensorModes; //1
    iCameraProperties->getBasicSensorModes(&sensorModes);
    std::vector<SensorMode*> modes;
    iCameraProperties->getAllSensorModes(&modes);
    if (sensorModes.size() == 0)
        cout <<"Failed to get sensor modes"<<endl; //exit



    //Debugging Purpose -> Listing out all Sensor Modes
    for (uint32_t i = 0; i < sensorModes.size(); i++){

        ISensorMode *iSensorMode = interface_cast<ISensorMode>(sensorModes[i]);
        cout<<i <<": Resolution: "<< iSensorMode->getResolution().height() << " x " << iSensorMode->getResolution().width() <<endl;

        if (!iSensorMode){
            cout<<"Failed to get sensor mode interface"<<endl;
            Size2D<uint32_t> resolution = iSensorMode->getResolution();
        }
    }

///////////////////////////////////////////////////////////////
///Sensor Mode
///////////////////////////////////////////////////////////////

    SensorMode *sensorMode = sensorModes[sensorModeIndex]; //2 is 60fps, 0 is 30 fps
    ISensorMode *iSensorMode = interface_cast<ISensorMode>(sensorModes[sensorModeIndex]);
    EXIT_IF_NULL(iSensorMode, "Failed to get sensor mode interface");
    emit return_Resolution(iSensorMode->getResolution().height());

///////////////////////////////////////////////////////////////
///Exposure Time
///////////////////////////////////////////////////////////////

    ///GET EXPOSURE TIME RANGE AND RESOLUTION
    ArgusSamples::Range<uint64_t> limitExposureTimeRange = iSensorMode->getExposureTimeRange();
    printf("-Sensor Exposure Range min %ju, max %ju\n", limitExposureTimeRange.min(), limitExposureTimeRange.max());
    Size2D<uint32_t> sensorResolution = iSensorMode->getResolution();

    cout<<"-Sensor Resolution: "<< iSensorMode->getResolution().height() << " x " << iSensorMode->getResolution().width() <<endl;
    EXIT_IF_NOT_OK(iSourceSettings->setSensorMode(sensorMode),"Unable to set Sensor Mode");

    //INTIALIZES THE CAMERA PARAMETERS OF THE CAMERA STARTING
    EXIT_IF_NOT_OK(iRequest->enableOutputStream(stream.get()),"Failed to enable stream in capture request");

    //const uint64_t THIRD_OF_A_SECOND = 500000;
    EXIT_IF_NOT_OK(iSourceSettings->setExposureTimeRange(ArgusSamples::Range<uint64_t>(curExposure)),"Unable to set the Source Settings Exposure Time Range");

    /// 3. GET THE GAIN RANGE FROM THE CHANGED EXPOSURE
    ArgusSamples::Range<float> sensorModeAnalogGainRange = iSensorMode->getAnalogGainRange();
    printf("-Sensor Analog Gain range min %f, max %f\n", sensorModeAnalogGainRange.min(), sensorModeAnalogGainRange.max());
    EXIT_IF_NOT_OK(iSourceSettings->setGainRange(ArgusSamples::Range<float>(sensorModeAnalogGainRange.min())), "Unable to set the Source Settings Gain Range");

    /// 4. SET THE GAIN RANGE TO MINIMUM
    ArgusSamples::Range<long unsigned int> sensorFrameDurationRange = iSensorMode->getFrameDurationRange();
    printf("-Frame Duration Range min %f, max %f\n", sensorFrameDurationRange.min(), sensorFrameDurationRange.max());
    EXIT_IF_NOT_OK(iSourceSettings->setFrameDurationRange(ArgusSamples::Range<long unsigned int>(sensorFrameDurationRange.min())), "Unable to set the Frame Duration Range");
    EXIT_IF_NOT_OK(iSession->repeat(request.get()), "Unable to submit repeat() request");

    float PreviousTimeStamp=0.0;
    float SensorTimestamp=0.0;
    uint64_t firstFrameTime = 0.0;
    float firstTimeStamp=0.0;

    auto start = std::chrono::high_resolution_clock::now();
    auto finish = std::chrono::high_resolution_clock::now();

    uint64_t PreviousFrameNum=0;
    uint64_t PreviousFrameTime=0;
    int PrevFrame=0;

///////////////////////////////////////////////////////////////
///CAPTURING LOOP
///////////////////////////////////////////////////////////////

    for (int frameCaptureLoop = 1; frameCaptureLoop < CAPTURE_COUNT; frameCaptureLoop++)
    {
        cout << endl << "############ Frame # " << frameCaptureLoop << " ###########" << endl << endl;

        auto startSettings = std::chrono::high_resolution_clock::now();

        //Pause Button vs StopButton
        if (stopButtonPressed || sensorModeApplyButtonPressed){
            cout << "Stop Button Pressed. Session Ended." << endl;
            break;
        }

        sync.lock();
         while(pauseButtonPressed){
             sync.unlock();
             sleep(1);
             sync.lock();
         }
        pauseCond.wakeAll(); // in this place, your thread will stop to execute until someone calls resume
        sync.unlock();

        const uint64_t ONE_SECOND = 1000000000;
        ///WAIT FOR EVENTS TO GET QUEUED
        // WAR Bug 200317271
        // update waitForEvents time from 1s to 2s to ensure events are queued up properly
        iEventProvider->waitForEvents(queue.get(), 2*ONE_SECOND);
        EXIT_IF_TRUE(iQueue->getSize() == 0, "No events in queue");

        ///GET EVENT CAPTURE
        const Event* event = iQueue->getEvent(iQueue->getSize() - 1);
        const IEventCaptureComplete *iEventCaptureComplete = interface_cast<const IEventCaptureComplete>(event);
        EXIT_IF_NULL(iEventCaptureComplete, "Failed to get EventCaptureComplete Interface");

        ///GET METADATA
        const CaptureMetadata *metaData = iEventCaptureComplete->getMetadata();
        const ICaptureMetadata* iMetadata = interface_cast<const ICaptureMetadata>(metaData);
        EXIT_IF_NULL(iMetadata, "Failed to get CaptureMetadata Interface");

//////////////////MIGHT NOT NEED

        ///GET EXPOSURE TIME AND ANALOG GAIN FROM METADATA
        uint64_t frameExposureTime = iMetadata->getSensorExposureTime();
        float frameGain = iMetadata->getSensorAnalogGain();
        printf("Frame metadata ExposureTime %ju, Analog Gain %f\n", frameExposureTime, frameGain); ///CHANGE THIS

        ///SUPPORTED FRAME RATE
        float FrameReadoutTime = iMetadata->getFrameReadoutTime();
        printf("FrameReadoutTime %f\n", FrameReadoutTime);

        float FrameDuration = iMetadata->getFrameDuration();
        printf("FrameDuration %f\n", (FrameDuration));

        PreviousTimeStamp=SensorTimestamp;
        SensorTimestamp = iMetadata->getSensorTimestamp();
        printf("Frame Rate (Processing Time) %f\n", 1.0/(SensorTimestamp/1000000000.0-PreviousTimeStamp/1000000000.0));

        /// SET EXPOSURE TIME WITH UI
        const uint64_t newExposureValue = curExposure;
        EXIT_IF_NOT_OK(iSourceSettings->setExposureTimeRange(ArgusSamples::Range<uint64_t>(newExposureValue)),"Unable to set the Source Settings Exposure Time Range");

        ///SET GAIN WITH UI
        float newGainValue = curGain;
        EXIT_IF_NOT_OK(iSourceSettings->setGainRange(ArgusSamples::Range<float>(newGainValue)), "Unable to set the Source Settings Gain Range");

/////////////////////////////////////////////////////////////

        auto finishSettings = std::chrono::high_resolution_clock::now();

        ////////IMAGE GENERATION STARTS HERE
        Argus::UniqueObj<EGLStream::Frame> frame(iFrameConsumer->acquireFrame());
        EGLStream::IFrame *iFrame = Argus::interface_cast<EGLStream::IFrame>(frame);
        EXIT_IF_NULL(iFrame, "Failed to get IFrame interface");

        EGLStream::Image *image = iFrame->getImage();
        EXIT_IF_NULL(image, "Failed to get Image from iFrame->getImage()");

        uint64_t FrameNum = iFrame->getNumber();
        uint64_t FrameTime = iFrame->getTime();

        if (frameCaptureLoop == 1){
            firstFrameTime = FrameTime;
            firstTimeStamp = iMetadata->getSensorTimestamp();
        }

        cout << "Real Frame Number  " << FrameNum << endl;
        cout << "Frames Dropped  " <<  FrameNum-PreviousFrameNum-1 << endl;
        PreviousFrameNum = FrameNum;
        PreviousFrameTime = FrameTime;

        //if Real frame does not equal to Frame Capture Loop, put it in a buffer

        auto finishGetImage = std::chrono::high_resolution_clock::now();

        EGLStream::NV::IImageNativeBuffer *iImageNativeBuffer = interface_cast<EGLStream::NV::IImageNativeBuffer> (image);
        EXIT_IF_NULL(iImageNativeBuffer, "Failed to get iImageNativeBuffer");
        Argus::Size2D<uint32_t> size(1920, 1080); //1920, 1080//1280,720

        int dmabuf_fd = iImageNativeBuffer->createNvBuffer(size, NvBufferColorFormat_YUV420,NvBufferLayout_Pitch);

        std::vector<cv::Mat> channels;
        std::vector<cv::Mat> RESIZEDchannels;
        cv::Mat img;

        void *data_mem1;
        void *data_mem2;
        void *data_mem3;
        channels.clear();

        auto startMapping = std::chrono::high_resolution_clock::now();

        NvBufferMemMap(dmabuf_fd, 0, NvBufferMem_Read_Write, &data_mem1);
        NvBufferMemSyncForCpu(dmabuf_fd, 0 , &data_mem1);
        //NvBufferMemSyncForDevice(dmabuf_fd, 0 , &data_mem1);
        channels.push_back(cv::Mat(1080, 1920, CV_8UC1, data_mem1, 2048));//540, 960 // 1080, 1920 // 720, 1280 //480 , 640 //360,480

        NvBufferMemMap(dmabuf_fd, 1, NvBufferMem_Read_Write, &data_mem2);
        NvBufferMemSyncForCpu(dmabuf_fd, 1 , &data_mem2);
        //NvBufferMemSyncForDevice(dmabuf_fd, 1 , &data_mem2);
        channels.push_back(cv::Mat(540, 960, CV_8UC1, data_mem2,1024)); //270, 480//540, 960 //360,640 //180,240

        NvBufferMemMap(dmabuf_fd, 2, NvBufferMem_Read_Write, &data_mem3);
        NvBufferMemSyncForCpu(dmabuf_fd, 2 , &data_mem3);
        //NvBufferMemSyncForDevice(dmabuf_fd, 2 , &data_mem3);
        channels.push_back(cv::Mat(540, 960, CV_8UC1, data_mem3, 1024)); //23040

        cv::Mat J,K,L;

        resize(channels[0], J,cv::Size(960, 540), 0, 0, cv::INTER_AREA); //640, 480 //960, 540 //480, 270
        //resize(channels[1], K,cv::Size(640, 480), 0, 0, cv::INTER_AREA);
        //resize(channels[2], L,cv::Size(640, 480), 0, 0, cv::INTER_AREA);

        K=channels[1];
        L=channels[2];

        RESIZEDchannels.push_back(J);
        RESIZEDchannels.push_back(K);
        RESIZEDchannels.push_back(L);
        auto finishMapping = std::chrono::high_resolution_clock::now();

        cv::merge ( RESIZEDchannels, img );
        cv::cvtColor ( img,img,CV_YCrCb2RGB );

        Mat imgTh;
        Mat imgProc1;
        Mat imgGray;
        Mat imgProc2;
        Mat saveimg2;

        cv::Rect ROI(0, 180, 639, 200);

        imgProc1=img.clone();

        cvtColor( imgProc1, imgGray, CV_BGR2GRAY );
        threshold(imgGray,imgTh,150,255,THRESH_BINARY_INV);

        Mat imgFF=imgTh.clone();
        floodFill(imgFF,cv::Point(5,5),Scalar(0));
        //floodFill(imgFF,cv::Point(10,460),Scalar(0));

        Rect ccomp;

        for(int m=0;m<imgFF.rows;m++)
        {
            for(int n=0;n<imgFF.cols;n++)
            {
                int iPixel=imgFF.at<uchar>(m,n);
                if(iPixel==255)
                {
                    int iArea=floodFill(imgFF,Point(n,m),Scalar(50),&ccomp);
                    if(iArea<40)
                    {
                        floodFill(imgFF,Point(n,m),Scalar(0));
                    }
                    else
                    {
                        circle(imgProc1,Point(ccomp.x+ccomp.width/2,ccomp.y+ccomp.height/2),30,Scalar(0,0,255),2,LINE_8);
                    }
                }
            }
        }

//        auto finishIP = std::chrono::high_resolution_clock::now();

//        if (captureButtonPressed){
//            string savepath = "/home/nvidia/Desktop/capture" + std::to_string(frameCaptureLoop) + ".png";
//            cv::imwrite(savepath, imgProc1);
//            captureButtonPressed=false;
//        }

        imShow[cameraDeviceIndex][1]=imgProc1.clone();
        imShow[cameraDeviceIndex][2]=imgTh.clone();
        imShow[cameraDeviceIndex][3]=imgFF.clone();
        imShow[cameraDeviceIndex][4]=imgGray.clone();

        QImage  Qimg((uchar*) img.data, img.cols, img.rows, img.step, QImage::Format_RGB888 );
        Mat tej = imShow[cameraDeviceIndex][DisplayIndex]; //cvCopy
        QImage QimgDefect = ASM::cvMatToQImage(tej);

//        emit return_QImageCAM2(Qimg.rgbSwapped());
//        emit return_QImageCAM1(Qimg.rgbSwapped());
        emit return_QImage(Qimg.rgbSwapped());
        emit return_DefectImage(QimgDefect);

//        auto finishDisplay = std::chrono::high_resolution_clock::now();

//        if (frameCaptureLoop%10==0){

//            iSession->repeat(request.get());
//        }

        NvBufferMemUnMap (dmabuf_fd, 0, &data_mem1);
        NvBufferMemUnMap (dmabuf_fd, 1, &data_mem2);
        NvBufferMemUnMap (dmabuf_fd, 2, &data_mem3);
        NvBufferDestroy (dmabuf_fd);

//        uint64_t FrameTime2 = iFrame->getTime();
//        SensorTimestamp = iMetadata->getSensorTimestamp();
//        auto finishUnMap = std::chrono::high_resolution_clock::now();
//        auto finishGPIO = std::chrono::high_resolution_clock::now();

        finish = std::chrono::high_resolution_clock::now();
        float totalduration= std::chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count();
//        float SettingDuration = std::chrono::duration_cast<std::chrono::nanoseconds>(finishSettings-startSettings).count();
//        float GetImageDuration = std::chrono::duration_cast<std::chrono::nanoseconds>(finishGetImage-finishSettings).count();
//        float Mappingduration = std::chrono::duration_cast<std::chrono::nanoseconds>(finishMapping-finishGetImage).count();
//        float IPduration = std::chrono::duration_cast<std::chrono::nanoseconds>(finishIP-finishMapping).count();
//        float Displayduration = std::chrono::duration_cast<std::chrono::nanoseconds>(finishDisplay-finishIP).count();
//        float Unloadduration = std::chrono::duration_cast<std::chrono::nanoseconds>(finishUnMap-finishDisplay).count();
//        float GPIOduration = std::chrono::duration_cast<std::chrono::nanoseconds>(finishGPIO-finishUnMap).count();
//        float oneloopduration= std::chrono::duration_cast<std::chrono::nanoseconds>(finish-startSettings).count();

//        cout <<"Setting Time Duration: " << fixed<< SettingDuration/1000000000.0 <<endl;
//        cout <<"Frame Rate using Chrono Clock: " << fixed<< (frameCaptureLoop*1.0)/(totalduration/1000000000.0) <<endl;
//        cout <<"Settings time usage: " << fixed<< ((SettingDuration*1.0)/(oneloopduration/1.0)) *100 << " %" <<endl;
//        cout <<"Get Image time usage: " << fixed<< ((GetImageDuration*1.0)/(oneloopduration/1.0)) *100 << " %" <<endl;
//        cout <<"Mapping time usage: " << fixed<< ((Mappingduration*1.0)/(oneloopduration/1.0)) *100 << " %" <<endl;
//        cout <<"Image Processing time usage " << fixed<< ((IPduration*1.0)/(oneloopduration/1.0))*100 << " %" <<endl;
//        cout <<"Display duration time usage " << fixed<< ((Displayduration*1.0)/(oneloopduration/1.0))*100 << " %" <<endl;
//        cout <<"Unmap time usage " << fixed<< ((Unloadduration*1.0)/(oneloopduration/1.0))*100 << " %" <<endl;
//        cout <<"GPIO time usage " << fixed<< ((GPIOduration*1.0)/(oneloopduration/1.0))*100 << " %" << endl;

        emit return_FrameRate((frameCaptureLoop*1.0)/(totalduration/1000000000.0));
        emit return_CurrFrameRate(1.0/(SensorTimestamp/1000000000.0-PreviousTimeStamp/1000000000.0));

    }

    iSession->stopRepeat();
    iSession->waitForIdle();

    queue.reset();
    stream.reset();
    cameraProvider.reset();
}




//////////////////////////////////////////////////////////
///Push Buttons
//////////////////////////////////////////////////////////

void MainCamera::preparePause(bool pauseButton)
{
    pauseButtonPressed = pauseButton;
}

void MainCamera::prepareStop(bool stopButton)
{
    stopButtonPressed = true;
    cout<< "Stop Button: " << stopButtonPressed << endl;
}

void MainCamera::prepareSensorModeChange(bool sensorModeApplyButton)
{
    sensorModeApplyButtonPressed = true;
    cout<< "sensorModeApplyButtonPressed: " << sensorModeApplyButtonPressed << endl;

}

void MainCamera::captureJPEG(bool checked)
{
    captureButtonPressed = true;
    cout << captureButtonPressed <<endl;

}

//////////////////////////////////////////////////////////
/// Set Values
//////////////////////////////////////////////////////////

void MainCamera::set_Exposure(int valueExposureSlider)
{
    curExposure = valueExposureSlider*1000;
    cout<< "Current Set Exposure: " << curExposure <<endl;
}

void MainCamera::set_Focus(int valueFocusSlider)
{
    curFocus = valueFocusSlider;
    cout<< "Curerent Focus Position: " << curFocus <<endl;
}

void MainCamera::set_Gain(float valueGainSlider)
{
    curGain = valueGainSlider;
    cout<< "set_GainCAM1: " << curGain <<endl;
}

void MainCamera::set_sensorMode(int value)
{
    sensorModeIndex = value;
    cout<< "Sensor Mode: " << sensorModeIndex <<endl;
}

void MainCamera::set_colourAnalysis(bool colourButton)
{
    colourButtonPressed = colourButton;
    cout<< "Colour Button is Submitted" << endl;
}

void MainCamera::set_DisplayOriginal(bool checked)
{
    if(checked)
    DisplayIndex =1;
}

void MainCamera::set_DisplayFloodFill(bool checked)
{
    if(checked)
    DisplayIndex =3;
}

void MainCamera::set_DisplayThreshold(bool checked){
    if(checked)
    DisplayIndex =2;
}

void MainCamera::set_DisplayGray(bool checked){
    if(checked)
    DisplayIndex =4;
}

//////////////////////////////////////////////////////////
///May not be in Use
//////////////////////////////////////////////////////////

void MainCamera::resume()
{
    sync.lock();
    pause = false;
    sync.unlock();
    pauseCond.wakeAll();
}

void MainCamera::paused()
{
    sync.lock();
    pause = true;
    sync.unlock();
}
