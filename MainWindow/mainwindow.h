#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QTimer>
#include <QTableView>
#include <QStandardItemModel>
#include <QStandardItem>
#include <QtDebug>

#if defined(WIN32)
#include <direct.h>
#endif
#include <iostream>
#include <pthread.h>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>
using namespace std;

#include "RobotArm/robotarm.h"
#include "Logging/logger.h"
#include "Settings/customsettings.h"
#include "Dynamixel/dynamixel.h"

const int NUM_JOINT = 3;
const int NUM_DOF = 3;
const double ENC2DEG = 0.088;
const double DEG2ENC = 11.363636364;
const double DEG2RAD = 0.017453293; // 3.14159265358979323846/180.0;
const double RAD2DEG = 57.295779513; // 180.0/3.14159265358979323846;
const double ENC2RPM = 	0.229;
const double RPM2DEG = 6;
const double RAW2mA = 2.69;
const double mA2RAW = 0.371747212;
const int offset[3] = {2301, 2232 - 170, 3786 + 1363};
const double preDefinedPathX[6] = {0.0, -0.17, -0.17, 0.17, 0.17, 0.0};
const double preDefinedPathY[6] = {0.25, 0.25, 0.25, 0.25, 0.25, 0.25};
const double preDefinedPathZ[6] = {-0.015, -0.015, 0.045, 0.045, -0.015, -0.015};

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    static void *mainThreadFunc(void* arg);
    static void *logThreadFunc(void* arg);
    static void *motionThreadFunc(void* arg);
    static void *premotionThreadFunc(void* arg);

private:
    Ui::MainWindow *ui;
    pthread_t mainThread, logThread, motionThread, premotionThread;
    vector<QPushButton*> controlSet;
    Logger *logger;
    RobotArm *robot;
    CustomSettings *customSettings;
    DxlControl* module;
    bool state;
    string device_name;
    int baud_rate;
    bool main_thread_run, log_thread_run, logging, motion_thread_run, premotion_thread_run;
    int32_t present_position[3], present_velocity[3];
    int16_t present_current[3];
    uint8_t torque_enable[3], present_torque_enable[3];
    int32_t goal_position[3];
    double present_q[3], present_pos[3], desired_pos[3], desired_q[3];
    vector<double> logData;
    vector<double> path_x, path_y, path_z;
    bool run_cmd;
    QStandardItemModel *model, *pathModel;
    int rowClickedIndex, colClickedIndex, rowPressedIndex, colPressedIndex;
    vector<double> px, py, pz, at, t;

    void mainThreadRun();
    void mainThreadStop();
    void logThreadRun();
    void logThreadStop();
    void motionThreadRun();
    void motionThreadStop();
    void premotionThreadRun();
    void premotionThreadStop();
    void componentEnable(bool flag);
    void guiUpdate();
    void jointPositionENC2DEG(int32_t pos_enc[], double pos_deg[]);
    void jointPositionENC2RAD(int32_t pos_enc[], double pos_rad[]);
    void cartesianPoseScaleUp(double pose_small[], double pose_big[]);
    void cartesianPoseScaleDown(double pose_big[], double pose_small[]);
    void jointPositionRAD2ENC(double pos_rad[], int32_t pos_enc[]);
    void jointPositionDEG2ENC(double pos_deg[], int32_t pos_enc[]);
    void jointVelocityENC2RPM(int32_t vel_enc[], double vel_rpm[]);
    void jointVelocityENC2RAD(int32_t vel_enc[], double vel_rad[]);
    void jointCurrentRAW2mA(int16_t cur_raw[], double cur_mA[]);
    void jointCurrentmA2RAW(double cur_mA[], int16_t cur_raw[]);
    void jointPositionRAD2DEG(double pos_rad[], double pos_deg[]);
    void pathGenerator(double x0, double xf, double tf, double ta, double h, std::vector<double> *path);

public slots:
    // button event
    void btnConnectClicked();
    void btnServoOnClicked();
    void btnServoOffClicked();
    void btnLoggingStartClicked();
    void btnLoggingStopClicked();
    void btnControlSetClicked();
    void btnSetJointClicked();
    void btnSetCartClicked();
    void btnAppendClicked();
    void btnInsertClicked();
    void btnDeleteClicked();
    void btnClearClicked();
    void btnApplyClicked();
    void btnRunClicked();
    void btnStopClicked();
    // tableview event
    void tvCellClicked(const QModelIndex &index);
    void verticalSectionClicked(int index);
    void horizontalSectionClicked(int index);
    // event
    void keyPressEvent(QKeyEvent *event);
};
#endif // MAINWINDOW_H
