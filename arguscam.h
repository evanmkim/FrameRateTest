#ifndef ARGUSCAM_H
#define ARGUSCAM_H

#include <QMainWindow>
#include <QObject>
#include <QWidget>
#include <QThread>
class ArgusCam: public QThread
{
    Q_OBJECT
public:
    ArgusCam();

    void show_cam();//all general functions
    bool initCAM();

protected:
    void run();

struct ExecuteOptions
    {
        uint32_t cameraIndex;
        uint32_t useAverageMap;
    };


};

#endif // ARGUSCAM_H
