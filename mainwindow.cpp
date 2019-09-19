#include "mainwindow.h"
#include "ui_mainwindow.h"
//#include "ArgusCamera.h"
#include <iostream>
#include <string>
#include <sstream>
#include <QString>
#include <algorithm>
//<linux/delay.h>
#include <QKeyEvent>



MainWindow::MainWindow(QWidget *parent) :

    QMainWindow(parent),
    ui(new Ui::MainWindow)
{


    ui->setupUi(this);



    ///CAM1 UI INITIALIZATION

    //SLIDER
    ui->ExposureTimeSlider->setRange(30,40000); //do not hard code this
    ui->ExposureTimeSlider->setValue(5000);
    //ui->FocusSlider->setRange(2,1499999);
    ui->GainSlider->setRange(10.0,2500.0); //do not hard code this
    ui->GainSlider->setValue(10.0);
    //BUTTON
    ui->pauseButton->setCheckable(true);
    ui->triggerModeButton->setCheckable(true);
    //RADIO
    ui->radioOriginal->setChecked(true);
    //SPINBOX
    ui->sensorModespinBox->setMaximum(2);

    ///THREAD INITIALIZATION
    ArgusCamera1 = new MainCamera(0);

    cout << " Test " << endl;



    ///MAIN CAMERA: CAM1

    //SLIDER
    connect(ui->ExposureTimeSlider,SIGNAL(valueChanged(int)),ArgusCamera1,SLOT(set_Exposure(int)));
    //connect(ui->FocusSlider,SIGNAL(valueChanged(int)),ArgusCamera1,SLOT(set_Focus(int)));
    connect(ui->GainSlider, SIGNAL(valueChanged(int)),this, SLOT(notifyGainChanged(int)));
    connect(this, SIGNAL(floatValueChanged(float)),ArgusCamera1, SLOT(set_Gain(float)));
    //SPINBOX
    connect(ui->sensorModespinBox, SIGNAL(valueChanged(int)),ArgusCamera1, SLOT(set_sensorMode(int)));
    //BUTTON
    connect(ui->pauseButton, SIGNAL(clicked(bool)), ArgusCamera1, SLOT(preparePause(bool)));
    connect(ui->stopButton, SIGNAL(clicked(bool)), ArgusCamera1, SLOT(prepareStop(bool)));
//    connect(ui->stopButton, SIGNAL(clicked(bool)), ArgusCamera2, SLOT(prepareStop(bool)));
//    connect(ui->colourInitButton, SIGNAL(clicked(bool)), ArgusCamera1, SLOT(set_colourAnalysis(bool)));
    //RADIO
    connect(ui->radioOriginal,SIGNAL(clicked(bool)),ArgusCamera1, SLOT(set_DisplayOriginal(bool)));
    connect(ui->radioFloodFill,SIGNAL(clicked(bool)),ArgusCamera1, SLOT(set_DisplayFloodFill(bool)));
    connect(ui->radioThreshold,SIGNAL(clicked(bool)),ArgusCamera1, SLOT(set_DisplayThreshold(bool)));
    connect(ui->radioGray,SIGNAL(clicked(bool)),ArgusCamera1, SLOT(set_DisplayGray(bool)));

    //***INCOMPLETE*** SENSOR MODE APPLY BUTTON
    //connect(ui->sensorModeApplyButton,SIGNAL(clicked(bool)),ArgusCamera1, SLOT(set_sensorMode(int)));
    //connect(ui->sensorModeApplyButton, SIGNAL(clicked(bool)), ArgusCamera1, SLOT(prepareSensorModeChange(bool)));
    //connect(ui->sensorModeApplyButton, SIGNAL(clicked(bool)), ArgusCamera2, SLOT(prepareSensorModeChange(bool)));
    //connect(ArgusCamera1,&ArgusCamera::return_SessionEnding,this,&MainWindow::StartSession);

    //UI DISPLAY VALUES
    connect(ArgusCamera1,&MainCamera::return_Resolution,this,&MainWindow::get_Resolu);
    connect(ArgusCamera1,&MainCamera::return_FrameRate,this,&MainWindow::get_FrameRate);
    connect(ArgusCamera1,&MainCamera::return_CurrFrameRate,this,&MainWindow::get_CurrFrameRate);
    connect(ArgusCamera1,&MainCamera::return_QImage,this,&MainWindow::get_QImage);
    connect(ArgusCamera1,&MainCamera::return_DefectImage,this,&MainWindow::get_DefectImage);

    connect(ui->captureButton,SIGNAL(clicked(bool)), ArgusCamera1,SLOT(captureJPEG(bool)));
    connect(ui->main_stopButtonCAM1, SIGNAL(clicked(bool)), ArgusCamera1, SLOT(prepareStop(bool)));

}


MainWindow::~MainWindow()
{
    delete ui;
}




////////////////////////////////////////////////////////////////////
///PUSH BUTTONS
////////////////////////////////////////////////////////////////////


void MainWindow::on_startButton_clicked()
{
    ArgusCamera1->start();

    cout << "Button Pressed " << endl;

}

///***INCOMPLETE***

void MainWindow::StartSession(bool)
{

}

///***INCORRECT USE***
void MainWindow::on_exitButton_clicked()
{
    //ArgusCamera2->quit();
}

void MainWindow::on_pauseButton_clicked(bool checked) //this is like a toggle
{
    if (checked){
        pauseButtonPressed = true;
        ui->pauseButton->setText("Resume");
    }
    else {
        pauseButtonPressed = false;
        ui->pauseButton->setText("Pause");
    }
}

///***INCOMPLETE***
void MainWindow::on_captureButton_clicked(bool checked)
{
    cout<<"capture image" <<endl;
}

void MainWindow::on_stopButton_clicked()
{
        stopButtonPressed = true;
}


/////////////////////////////////////////////////////////////////////
///SLIDER
/// ////////////////////////////////////////////////////////////////

void MainWindow::on_ExposureTimeSlider_valueChanged(int newValue)
{
    QString strExpTime=QString::number(newValue);
    ui->label->setText(strExpTime+" µs");
}

void MainWindow::on_GainSlider_valueChanged(int value)
{
    float floatValue = value/10.0;
    emit floatValueChanged(floatValue);

    QString strGainTime=QString::number(floatValue);
    ui->labelGain->setText(strGainTime);
}

//void MainWindow::on_FocusSlider_valueChanged(int newValue)
//{
//    QString strExpTime=QString::number(newValue);
//    ui->labelFrameDuration->setText(strExpTime+" µs");
//}

///INCOMPLETE
int MainWindow::get_minExposure(int newValue)
{
    return newValue;
}


///////////////////////////////////////////////////////////////////////////
///DISPLAY VALUE (SIGNALS)
///////////////////////////////////////////////////////////////////////////

//Gain

void MainWindow::notifyGainChanged(int value){
    float floatValue = value/10.0;
    emit floatValueChanged(floatValue);
}

//void MainWindow::notifyGainChangedCAM2(int value){
//    float floatValue = value/10.0;

//    cout << "CAM 2 slider changing gain " << floatValue << endl;
//    emit floatValueChangedCAM2(floatValue);
//}

//Frame Rate

void MainWindow::get_FrameRate(double dFrameRate)
{
    QString strFrameRate=QString::number(dFrameRate)+"  fps";
    ui->labelFrameRate->setText(strFrameRate);
}

void MainWindow::get_CurrFrameRate(double dFrameRate)
{
    QString strFrameRate=QString::number(dFrameRate)+"  fps";
    ui->labelCurrFrameRate->setText(strFrameRate);
}

//Resolution

void MainWindow::get_Resolu(int sensorResolution)
{
    //QString strExpTime=QString::fromStdString("get res");
    QString strExpTime=QString::number(sensorResolution)+" p";
    ui->labelResolution->setText(strExpTime);
}

//Image Display

void MainWindow::get_QImage(QImage img_temp)
{
    image=img_temp;
    QMatrix rm;
    rm.rotate(0);
    ui->QimageLabel->setPixmap(QPixmap::fromImage(image).transformed(rm).scaled(ui->QimageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

void MainWindow::get_DefectImage(QImage img_temp)
{
    image=img_temp;
    QMatrix rm;
    rm.rotate(0);
    ui->QimageDefect->setPixmap(QPixmap::fromImage(image).transformed(rm).scaled(ui->QimageDefect->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}


///SENSOR MODE
void MainWindow::on_sensorModespinBox_valueChanged(int arg1)
{
    QString strExpTime=QString::number(arg1);
    //ui->labelResolution->setText(strExpTime);
}

void MainWindow::on_sensorModeApplyButton_clicked()
{
//    ui->stopButton->click();

//    ArgusCamera1 = new ArgusCamera(0); //start running camera on start pressed
//    ArgusCamera2 = new ArgusCamera(1);

//   ui->startButton->click();

}

void MainWindow::on_sensorModeApplyButton_pressed()
{
    ui->stopButton->clicked();
    ui->startButton->clicked();
}

void MainWindow::on_sensorModeApplyButton_released()
{
    ui->startButton->clicked();
}

//void MainWindow::on_sensorModeApplyButton_2_pressed()
//{
//    ui->stopButton->clicked();
//}

//void MainWindow::on_sensorModeApplyButton_2_released()
//{
//    ui->startButton->clicked();
//}

////////////////////////////////////////////////
///***INCOMPLETE***
////////////////////////////////////////////////
void MainWindow::on_triggerModeButton_clicked(bool checked)
{

}

void MainWindow::on_triggerModeButton_clicked()
{
//    //if DetectValue is high

//    /*CHECK FOR GPIO INTERRUPT*/
//    unsigned int Detectvalue=high;
//    gpioGetValue(ButtonSigPin,&Detectvalue);

//    if (Detectvalue==high) {//***Add condition on if picture not taken
////        cout<< "Detectvalue==high    :" << Detectvalue <<endl;
////        string savepath = "/home/nvidia/Desktop/capture" + std::to_string(frameCaptureLoop) + ".png";
////        cv::imwrite(savepath, tej);

//        triggermode1->start();

//     }
//    else {
//        cout<< "Detectvalue==low    :" << Detectvalue <<endl; //Take Picture
//     }

////    mSN = new QSocketNotifier; // defined in .h
////    QFile file("/dev/testDriver");
////    if(file.open(QFile::ReadOnly)) {
////      QSocketNotifier mSN(file.handle(), , QSocketNotifier::Read);
////      mSN.setEnabled(true);
////      connect(mSN, SIGNAL(activated(int)), &this, SLOT(readyRead()));
////    }

}


///MAIN PAGE

void MainWindow::on_main_startButtonCAM1_clicked()
{
    ArgusCamera1->start();
}

void MainWindow::on_main_stopButtonCAM1_clicked()
{
    stopButtonPressed = true;
}

void MainWindow::on_StrtBut_clicked()
{
    ArgusCamera1->start();

}
