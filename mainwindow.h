#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "QtMultimedia"
#include <QtMultimediaWidgets>
#include <QMainWindow>
#include <QImage>

//#include <QString>
#include <string>

#include <maincamera.h>
#undef Bool
//SIGNAL

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

//private:
//    QMutex sync;
//    QWaitCondition pauseCond;
//    bool pause;

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    int ExposureValue=300000; //change to public, set initial value = starting position of slider
    float GainInitialValue=1.0;
    int sensorMode =0;
    bool pauseButtonPressed = false;
    bool stopButtonPressed = false;
//    bool colourButtonPressed = false;
    bool captureButtonPressed = false;
    int FocusValue=300000;

//    int FlagProcessImage;
//    int FlagDisplayImage;


signals:
    void floatValueChanged(float);
    void floatValueChangedCAM2(float);

private slots:

    //SLIDERS
    void on_ExposureTimeSlider_valueChanged(int newValue); //change the label display besides the slider
    void notifyGainChanged(int value);
    void on_GainSlider_valueChanged(int value);
//    void on_FocusSlider_valueChanged(int value);

    //PUSH BUTTONS
    void on_startButton_clicked();
    void on_pauseButton_clicked(bool checked);
//    void on_colourInitButton_clicked(bool checked);
    void on_stopButton_clicked();

    //SPINBOX
    void on_sensorModespinBox_valueChanged(int arg1);

    //UNUSED
    int get_minExposure(int ExposureValue);

    //Dislay Data
    void get_QImage(QImage);
    void get_DefectImage(QImage);
    void get_FrameRate(double);
    void get_CurrFrameRate(double);

//    void get_colourL(double);
//    void get_colourA(double);
//    void get_colourB(double);

//    void get_colourBl(double);
//    void get_colourG(double);
//    void get_colourR(double);

    void on_captureButton_clicked(bool);
    void get_Resolu(int);
//    void get_QImageCAM2(QImage);

//    void get_colourDisplay();

//    void get_ResoluCAM2(int);
//    void get_FrameRateCAM2(double);
//    void get_CurrFrameRateCAM2(double);

//    void notifyGainChangedCAM2(int value);
    void StartSession(bool);
//    void on_resetButton_clicked();

//    void on_radioThreshold_clicked();
//    void on_radioOriginal_clicked();

//    void on_ExposureTimeSlider_2_valueChanged(int);
//    void on_GainSlider_2_valueChanged(int);
//    void on_FocusSlider_valueChanged_2(int);


    void on_exitButton_clicked();

    void on_sensorModeApplyButton_clicked();

    void on_sensorModeApplyButton_pressed();

    void on_sensorModeApplyButton_released();

//    void on_sensorModeApplyButton_2_pressed();

//    void on_sensorModeApplyButton_2_released();

    void on_triggerModeButton_clicked(bool checked);

    void on_triggerModeButton_clicked();

    void on_main_startButtonCAM1_clicked();

//    void on_main_startButtonCAM2_clicked();

    void on_main_stopButtonCAM1_clicked();

//    void on_main_stopButtonCAM2_clicked();

    void on_StrtBut_clicked();

private:
    Ui::MainWindow *ui;
    //ARGUSCAMERA::ArgusCamera *ArgusCamera1;
    MainCamera *ArgusCamera1;
    //ARGUSCAMERA::ArgusCamera *ArgusCamera2;
    //triggerMode *triggermode1;
    //triggermode1 = new triggerMode();
    QImage image;
    QImage Defectimage;
    double dFrameRate;
    double currFrameRate;
    std::vector<double> LAB[];
    QMutex sync;
    QWaitCondition pauseCond;
    bool pause;
    //jetsonTX1GPIONumber ButtonSigPin = gpio184;

    double BGR[];

};

#endif // MAINWINDOW_H

