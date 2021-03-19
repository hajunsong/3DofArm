#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    connect(ui->btnConnect, SIGNAL(clicked()), this, SLOT(btnConnectClicked()));
    connect(ui->btnServoOn, SIGNAL(clicked()), this, SLOT(btnServoOnClicked()));
    connect(ui->btnServoOff, SIGNAL(clicked()), this, SLOT(btnServoOffClicked()));
    connect(ui->btnLoggingStart, SIGNAL(clicked()), this, SLOT(btnLoggingStartClicked()));
    connect(ui->btnLoggingStop, SIGNAL(clicked()), this, SLOT(btnLoggingStopClicked()));
    connect(ui->btnSetJoint, SIGNAL(clicked()), this, SLOT(btnSetJointClicked()));
    connect(ui->btnSetCart, SIGNAL(clicked()), this, SLOT(btnSetCartClicked()));

    controlSet.push_back(ui->btn1);
    controlSet.push_back(ui->btn2);
    controlSet.push_back(ui->btn3);
    controlSet.push_back(ui->btn4);
    controlSet.push_back(ui->btn5);
    controlSet.push_back(ui->btn6);
    controlSet.push_back(ui->btn7);
    controlSet.push_back(ui->btn8);
    controlSet.push_back(ui->btn9);
    controlSet.push_back(ui->btn10);
    controlSet.push_back(ui->btn11);
    controlSet.push_back(ui->btn12);
    for(unsigned int i = 0; i < controlSet.size(); i++){
        connect(controlSet[i], SIGNAL(clicked()), this, SLOT(btnControlSetClicked()));
    }

    ui->gbMotor->setEnabled(false);
    ui->gbLogging->setEnabled(false);
    ui->gbRobotControl->setEnabled(false);
    ui->gbRobotData->setEnabled(false);
    ui->gbMotion->setEnabled(false);

    robot = new RobotArm(NUM_JOINT, NUM_DOF);

    module = new DxlControl();

    customSettings = new CustomSettings(ui);
    customSettings->loadConfigFile();

    device_name = ui->txtPort->text().toStdString();
    baud_rate = ui->cbBaud->currentText().toInt();

    state = true;
    main_thread_run = false;

    memset(present_position, 0, sizeof(int32_t)*3);
    memset(present_velocity, 0, sizeof(int32_t)*3);
    memset(present_current, 0, sizeof(int16_t)*3);
    memset(present_torque_enable, 0, sizeof(uint8_t)*3);
    memset(present_q, 0, sizeof(double)*3);
    memset(desired_q, 0, sizeof(double)*3);
    memset(present_pos, 0, sizeof(double)*3);

    memset(goal_position, 0, sizeof(int32_t)*3);

    log_thread_run = false;
    logging = false;

    motion_thread_run = false;

    run_cmd = false;

    connect(ui->btnAppend, SIGNAL(clicked()), this, SLOT(btnAppendClicked()));
    connect(ui->btnInsert, SIGNAL(clicked()), this, SLOT(btnInsertClicked()));
    connect(ui->btnDelete, SIGNAL(clicked()), this, SLOT(btnDeleteClicked()));
    connect(ui->btnClear, SIGNAL(clicked()), this, SLOT(btnClearClicked()));
    connect(ui->btnApply, SIGNAL(clicked()), this, SLOT(btnApplyClicked()));
    connect(ui->btnRun, SIGNAL(clicked()), this, SLOT(btnRunClicked()));
    connect(ui->btnStop, SIGNAL(clicked()), this, SLOT(btnStopClicked()));
    ui->btnRun->setEnabled(false);
    ui->btnStop->setEnabled(false);

    connect(ui->tvPathData, SIGNAL(clicked(QModelIndex)), this, SLOT(tvCellClicked(QModelIndex)));
    connect(ui->tvPathData->horizontalHeader(), SIGNAL(sectionClicked(int)), this, SLOT(horizontalSectionClicked(int)));
    connect(ui->tvPathData->verticalHeader(), SIGNAL(sectionClicked(int)), this, SLOT(verticalSectionClicked(int)));

    pathModel = new QStandardItemModel(1, 5, this);
    ui->tvPathData->setModel(pathModel);

    for(int i = 0; i < 1; i++){
        for(int j = 0; j < 5; j++){
            QModelIndex indx = pathModel->index(i, j, QModelIndex());
            pathModel->setData(indx, "");
        }
    }

    QStringList hHeader;
    hHeader.append("Time");
    hHeader.append("X");
    hHeader.append("Y");
    hHeader.append("Z");
    hHeader.append("Acc time");
    pathModel->setHorizontalHeaderLabels(hHeader);
}

MainWindow::~MainWindow()
{
    mainThreadStop();

    for(int i = 0; i < NUM_JOINT; i++){
        module->dxl_deinit(i);
    }
    usleep(100000);

    customSettings->saveConfigFile();
    controlSet.clear();
    logData.clear();
    path_x.clear();
    path_y.clear();
    path_z.clear();
    px.clear();
    py.clear();
    pz.clear();
    at.clear();
    t.clear();
    delete robot;
    delete module;
    if(logging){
        delete logger;
    }
    delete customSettings;
    delete ui;
}

void *MainWindow::mainThreadFunc(void *arg)
{
    MainWindow* pThis = static_cast<MainWindow*>(arg);

    printf("Start main thread\n");
    pThis->main_thread_run = true;

    while(pThis->main_thread_run){
        pThis->module->getGroupSyncReadIndirectAddress(pThis->present_position, pThis->present_velocity, pThis->present_current, pThis->present_torque_enable, 3);

//        printf("Present Position : %d, %d, %d\n", pThis->present_position[0], pThis->present_position[1], pThis->present_position[2]);
//        printf("Present Velocity : %d, %d, %d\n", pThis->present_velocity[0], pThis->present_velocity[1], pThis->present_velocity[2]);
//        printf("Present Current : %d, %d, %d\n", pThis->present_current[0], pThis->present_current[1], pThis->present_current[2]);
//        printf("Present Torque Enable : %d, %d, %d\n", pThis->present_torque_enable[0], pThis->present_torque_enable[1], pThis->present_torque_enable[2]);

        pThis->jointPositionENC2RAD(pThis->present_position, pThis->present_q);
//        printf("Present Position(rad) : %5.5f, %5.5f, %5.5f\n", pThis->present_q[0], pThis->present_q[1], pThis->present_q[2]);
        pThis->robot->run_kinematics(pThis->present_q, pThis->present_pos);
        pThis->guiUpdate();

        if(pThis->logging){
            pThis->logData.push_back(pThis->present_q[0]*RAD2DEG);
            pThis->logData.push_back(pThis->present_q[1]*RAD2DEG);
            pThis->logData.push_back(pThis->present_q[2]*RAD2DEG);
            pThis->logData.push_back(pThis->present_pos[0]*1000);
            pThis->logData.push_back(pThis->present_pos[1]*1000);
            pThis->logData.push_back(pThis->present_pos[2]*1000);
        }

        if(pThis->run_cmd){
            pThis->module->setGroupSyncWriteGoalPosition(pThis->goal_position, NUM_JOINT);
            pThis->run_cmd = false;
        }

        usleep(10000);
    }

    printf("Finished main thread\n");

    return nullptr;
}

void *MainWindow::logThreadFunc(void *arg)
{
    MainWindow* pThis = static_cast<MainWindow*>(arg);

    printf("Start log thread\n");
    pThis->log_thread_run = true;

    while(pThis->log_thread_run){
        if(pThis->logData.size() >= 6){
            QString msg = QString::number(pThis->present_q[0]) + "," + QString::number(pThis->present_q[1]) + "," + QString::number(pThis->present_q[2]) + "," + QString::number(pThis->present_pos[0]) + "," +
                    QString::number(pThis->present_pos[1]) + "," + QString::number(pThis->present_pos[2]);
            pThis->logger->write(msg);
        }

        usleep(5000);
    }

    printf("Finished log thread\n");

    return nullptr;
}

void *MainWindow::motionThreadFunc(void *arg){
    MainWindow* pThis = static_cast<MainWindow*>(arg);

    printf("Start motion thread\n");
    pThis->motion_thread_run = true;

    if(pThis->path_x.size() > 0 && pThis->path_y.size() > 0 && pThis->path_z.size()){
        if(pThis->path_x.size() == pThis->path_y.size() && pThis->path_x.size() == pThis->path_z.size()){
            int path_size = pThis->path_x.size();
            double desired_pos_temp[3] = {0,0,0};
            for(int i = 0; i < path_size; i++){
                if(pThis->motion_thread_run){
                    desired_pos_temp[0] = pThis->path_x[i];
                    desired_pos_temp[1] = pThis->path_y[i];
                    desired_pos_temp[2] = pThis->path_z[i];

                    pThis->robot->run_inverse_kinematics(desired_pos_temp, pThis->desired_q);
                    pThis->jointPositionRAD2ENC(pThis->desired_q, pThis->goal_position);
                    pThis->run_cmd = true;
                    usleep(2000);
                }
            }
            pThis->run_cmd = false;
        }
    }

    printf("Finished motion thread\n");
    pThis->motion_thread_run = false;
    pThis->run_cmd = false;

    return nullptr;
}

void *MainWindow::premotionThreadFunc(void *arg)
{
    MainWindow* pThis = static_cast<MainWindow*>(arg);

    printf("Start premotion thread\n");
    pThis->premotion_thread_run = true;

    for(unsigned int i = 0; i < pThis->px.size(); i++){
        if(pThis->premotion_thread_run){
            pThis->desired_pos[0] = pThis->px[i]*0.001;
            pThis->desired_pos[1] = pThis->py[i]*0.001;
            pThis->desired_pos[2] = pThis->pz[i]*0.001;

            pThis->path_x.clear();
            pThis->path_y.clear();
            pThis->path_z.clear();

            if(i == 0){
                pThis->pathGenerator(pThis->present_pos[0], pThis->desired_pos[0], pThis->t[1] - pThis->t[0], pThis->at[0], 0.01, &pThis->path_x);
                pThis->pathGenerator(pThis->present_pos[1], pThis->desired_pos[1], pThis->t[1] - pThis->t[0], pThis->at[0], 0.01, &pThis->path_y);
                pThis->pathGenerator(pThis->present_pos[2], pThis->desired_pos[2], pThis->t[1] - pThis->t[0], pThis->at[0], 0.01, &pThis->path_z);
            }
            else{
                pThis->pathGenerator(pThis->present_pos[0], pThis->desired_pos[0], pThis->t[i] - pThis->t[i-1], pThis->at[i], 0.01, &pThis->path_x);
                pThis->pathGenerator(pThis->present_pos[1], pThis->desired_pos[1], pThis->t[i] - pThis->t[i-1], pThis->at[i], 0.01, &pThis->path_y);
                pThis->pathGenerator(pThis->present_pos[2], pThis->desired_pos[2], pThis->t[i] - pThis->t[i-1], pThis->at[i], 0.01, &pThis->path_z);
            }
            pThis->motionThreadRun();
            pthread_join(pThis->motionThread, nullptr);
            usleep(500000);
        }
    }

    printf("Finished premotion thread\n");
    pThis->premotion_thread_run = false;

    pThis->ui->btnRun->setEnabled(true);
    pThis->ui->btnStop->setEnabled(false);

    return nullptr;
}

void MainWindow::mainThreadRun()
{
    main_thread_run = false;
    pthread_create(&mainThread, nullptr, mainThreadFunc, this);
}

void MainWindow::mainThreadStop()
{
    if(main_thread_run){
        main_thread_run = false;
        usleep(100000);
        pthread_join(mainThread, nullptr);
    }
}

void MainWindow::logThreadRun()
{
    log_thread_run = false;
    pthread_create(&logThread, nullptr, logThreadFunc, this);
}

void MainWindow::logThreadStop()
{
    if(log_thread_run){
        log_thread_run = false;
        usleep(100000);
        pthread_join(logThread, nullptr);
    }
}

void MainWindow::motionThreadRun()
{
    motion_thread_run = false;
    pthread_create(&motionThread, nullptr, motionThreadFunc, this);
}

void MainWindow::motionThreadStop()
{
    if(motion_thread_run){
        motion_thread_run = false;
        usleep(100000);
        pthread_join(motionThread, nullptr);
    }
}

void MainWindow::premotionThreadRun()
{
    ui->btnStop->setEnabled(true);
    ui->btnRun->setEnabled(false);

    premotion_thread_run = false;
    pthread_create(&premotionThread, nullptr, premotionThreadFunc, this);
}

void MainWindow::premotionThreadStop()
{
    if(premotion_thread_run){
        ui->btnStop->setEnabled(false);
        ui->btnRun->setEnabled(true);

        premotion_thread_run = false;
        usleep(100000);
        pthread_join(premotionThread, nullptr);
    }
}

void MainWindow::componentEnable(bool flag)
{
    ui->gbMotor->setEnabled(flag);
    ui->gbLogging->setEnabled(flag);
    ui->gbRobotData->setEnabled(flag);
    ui->gbMotion->setEnabled(flag);
}

void MainWindow::guiUpdate()
{
    ui->txtCurJoint1->setText(QString::number((present_q[0]*RAD2DEG)));
    ui->txtCurJoint2->setText(QString::number((present_q[1]*RAD2DEG)));
    ui->txtCurJoint3->setText(QString::number((present_q[2]*RAD2DEG)));
    ui->txtCurEndX->setText(QString::number((present_pos[0]*1000)));
    ui->txtCurEndY->setText(QString::number((present_pos[1]*1000)));
    ui->txtCurEndZ->setText(QString::number((present_pos[2]*1000)));

    if(present_torque_enable[0] == 1 && present_torque_enable[1] == 1 && present_torque_enable[2] == 1){
        ui->btnServoOn->setEnabled(false);
        ui->btnServoOff->setEnabled(true);
        ui->gbRobotControl->setEnabled(true);
        ui->gbMotion->setEnabled(true);
    }
    else{
        ui->btnServoOn->setEnabled(true);
        ui->btnServoOff->setEnabled(false);
        ui->gbRobotControl->setEnabled(false);
        ui->gbMotion->setEnabled(false);
    }
}

void MainWindow::btnConnectClicked()
{
    state = module->init(device_name, baud_rate);
    if(state){
        ui->btnConnect->setEnabled(false);
        componentEnable(true);
        ui->btnServoOff->setEnabled(false);
        ui->btnLoggingStop->setEnabled(false);
    }
    else{
        cout << "Com port initialization " << "FAIL" << endl;
        ui->btnConnect->setEnabled(true);
        componentEnable(false);
        return;
    }

    if(state){
        for(int i = 0; i < NUM_JOINT; i++){
            state = module->dxl_init(i, JointOpMode::extended_position_mode);
            if(state){
                module->initGroupSyncReadIndirectAddress(i);
            }
            else{
                break;
            }
        }
    }

    if(!state){
        cout << "Module initialization " << "FAIL" << endl;
        componentEnable(false);
        return;
    }

    if(state){
        mainThreadRun();
    }
}

void MainWindow::btnServoOnClicked()
{
    mainThreadStop();
    module->setGroupSyncWriteTorqueEnable(TORQUE_ENABLE, NUM_JOINT);
    mainThreadRun();
    run_cmd = false;
}

void MainWindow::btnServoOffClicked()
{
    mainThreadStop();
    module->setGroupSyncWriteTorqueEnable(TORQUE_DISABLE, NUM_JOINT);
    mainThreadRun();
    run_cmd = false;
}

void MainWindow::btnLoggingStartClicked()
{
    QDateTime *date = new QDateTime();
#if defined(WIN32)
    _mkdir("logging");
#else
    mkdir("logging", 0777);
#endif
    QString fileName = "logging/" + date->currentDateTime().toString("yyyy-MM-dd-hh-mm-ss") + ".csv";
    logger = new Logger(this, fileName);
    delete date;

    logger->write("q1[deg],q2[deg],q3[deg],End X[mm],End Y[mm],End Z[mm]");

    logging = true;
    logThreadRun();
    ui->btnLoggingStart->setEnabled(false);
    ui->btnLoggingStop->setEnabled(true);
}

void MainWindow::btnLoggingStopClicked()
{
    logging = false;
    logThreadStop();
    delete logger;
    ui->btnLoggingStart->setEnabled(true);
    ui->btnLoggingStop->setEnabled(false);
}

void MainWindow::btnControlSetClicked(){
    QString objName = sender()->objectName();
    int indx = objName.split("btn")[1].toUInt() - 1;

    if(indx < 6){
        if(indx%2 == 0){ // plus
            memcpy(desired_q, present_q, sizeof(double)*3);
            desired_q[indx/2] = present_q[indx/2] + 5.0*DEG2RAD;
            jointPositionRAD2ENC(desired_q, goal_position);
        }
        else{ // minus
            memcpy(desired_q, present_q, sizeof(double)*3);
            desired_q[indx/2] = present_q[indx/2] - 5.0*DEG2RAD;
            jointPositionRAD2ENC(desired_q, goal_position);
        }
    }
    else{
        indx -= 6;
        if(indx%2 == 0){ // plus
            memcpy(desired_pos, present_pos, sizeof(double)*3);
            desired_pos[indx/2] = present_pos[indx/2] + 10*0.001;
            robot->run_inverse_kinematics(desired_pos, desired_q);
            jointPositionRAD2ENC(desired_q, goal_position);
        }
        else{ // minus
            memcpy(desired_pos, present_pos, sizeof(double)*3);
            desired_pos[indx/2] = present_pos[indx/2] - 10*0.001;
            robot->run_inverse_kinematics(desired_pos, desired_q);
            jointPositionRAD2ENC(desired_q, goal_position);
        }
    }

    run_cmd = true;
}

void MainWindow::btnSetJointClicked()
{
    if(ui->txtCmdJoint1->text().length() > 0 && ui->txtCmdJoint2->text().length() > 0 && ui->txtCmdJoint3->text().length() > 0){
        desired_q[0] = ui->txtCmdJoint1->text().toDouble()*DEG2RAD;
        desired_q[1] = ui->txtCmdJoint2->text().toDouble()*DEG2RAD;
        desired_q[2] = ui->txtCmdJoint3->text().toDouble()*DEG2RAD;

        jointPositionRAD2ENC(desired_q, goal_position);

        run_cmd = true;
    }
}

void MainWindow::btnSetCartClicked()
{
    if(ui->txtCmdEndX->text().length() > 0 && ui->txtCmdEndY->text().length() > 0 && ui->txtCmdEndZ->text().length() > 0){
        desired_pos[0] = ui->txtCmdEndX->text().toDouble()*0.001;
        desired_pos[1] = ui->txtCmdEndY->text().toDouble()*0.001;
        desired_pos[2] = ui->txtCmdEndZ->text().toDouble()*0.001;

        path_x.clear();
        path_y.clear();
        path_z.clear();

        pathGenerator(present_pos[0], desired_pos[0], 2, 0.3, 0.01, &path_x);
        pathGenerator(present_pos[1], desired_pos[1], 2, 0.3, 0.01, &path_y);
        pathGenerator(present_pos[2], desired_pos[2], 2, 0.3, 0.01, &path_z);

        motionThreadRun();
    }
}

void MainWindow::jointPositionENC2DEG(int32_t pos_enc[], double pos_deg[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_deg[i] = (pos_enc[i] - offset[i])*ENC2DEG;
    }
}

void MainWindow::jointPositionENC2RAD(int32_t pos_enc[], double pos_rad[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_rad[i] = (pos_enc[i] - offset[i])*ENC2DEG*DEG2RAD;
    }
}

void MainWindow::cartesianPoseScaleUp(double pose_small[], double pose_big[])
{
    for(int i = 0; i < NUM_DOF; i++){
        if (i < 3){
            pose_big[i] = pose_small[i]*1000;
        }
        else{
            pose_big[i] = pose_small[i]*RAD2DEG;
        }
    }
}

void MainWindow::cartesianPoseScaleDown(double pose_big[], double pose_small[])
{
    for(int i = 0; i < NUM_DOF; i++){
        if (i < 3){
            pose_small[i] = pose_big[i]*0.001;
        }
        else{
            pose_small[i] = pose_big[i]*DEG2RAD;
        }
    }
}

void MainWindow::jointPositionRAD2ENC(double pos_rad[], int32_t pos_enc[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_enc[i] = static_cast<int32_t>(pos_rad[i]*RAD2DEG*DEG2ENC) + offset[i];
    }
}

void MainWindow::jointPositionDEG2ENC(double pos_deg[], int32_t pos_enc[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_enc[i] = static_cast<int32_t>(pos_deg[i]*DEG2ENC) + offset[i];
    }
}

void MainWindow::jointVelocityENC2RPM(int32_t vel_enc[], double vel_rpm[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        vel_rpm[i] = vel_enc[i]*ENC2RPM;
    }
}

void MainWindow::jointVelocityENC2RAD(int32_t vel_enc[], double vel_rad[]){
    for(int i = 0; i < NUM_JOINT; i++){
        vel_rad[i] = vel_enc[i]*ENC2RPM*RPM2DEG*DEG2RAD;
    }
}

void MainWindow::jointCurrentRAW2mA(int16_t cur_raw[], double cur_mA[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        cur_mA[i] = cur_raw[i]*RAW2mA;
    }
}

void MainWindow::jointCurrentmA2RAW(double cur_mA[], int16_t cur_raw[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        cur_raw[i] = static_cast<int16_t>(cur_mA[i]*mA2RAW);
    }
}

void MainWindow::jointPositionRAD2DEG(double pos_rad[], double pos_deg[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_deg[i] = pos_rad[i]*RAD2DEG;
    }
}

void MainWindow::pathGenerator(double x0, double xf, double tf, double ta, double h, std::vector<double> *path)
{
    //    printf("Start path generator, x0 : %f, xf : %f, tf : %f, ta : %f, h : %f\n", x0, xf, tf, ta, h);
    double td = tf - ta;
    double vd = (xf - x0)/td;
    double xa = x0 + 0.5*ta*vd;
    double xd = xf - 0.5*ta*vd;

    double pos0, posf, vel0, velf, acc0, accf, ts;
    double a0, a1, a2, a3, a4, a5;

    // section of acceleration
    pos0 = x0; posf = xa; vel0 = 0; velf = vd; acc0 = 0; accf = 0; ts = ta;
    a0 = pos0;
    a1 = vel0;
    a2 = acc0/2;
    a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*pow(ts,2))/(2*pow(ts,3));
    a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*pow(ts,2))/(2*pow(ts, 4));
    a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*pow(ts,2))/(2*pow(ts,5));

    for(double t = 0; t < ts; t += h){
        path->push_back(a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4)+ a5*pow(t,5));
    }

    // section of constant velocity
    pos0 = xa; posf = xd; vel0 = vd; velf = vd; acc0 = 0; accf = 0; ts = td - ta;
    a0 = pos0;
    a1 = vel0;
    a2 = acc0/2;
    a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*pow(ts,2))/(2*pow(ts,3));
    a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*pow(ts,2))/(2*pow(ts, 4));
    a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*pow(ts,2))/(2*pow(ts,5));

    for(double t = 0; t < ts; t += h){
        path->push_back(a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4)+ a5*pow(t,5));
    }

    // section of deceleration
    pos0 = xd; posf = xf; vel0 = vd; velf = 0; acc0 = 0; accf = 0; ts = tf - td;
    a0 = pos0;
    a1 = vel0;
    a2 = acc0/2;
    a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*pow(ts,2))/(2*pow(ts,3));
    a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*pow(ts,2))/(2*pow(ts, 4));
    a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*pow(ts,2))/(2*pow(ts,5));

    for(double t = 0; t < ts; t += h){
        path->push_back(a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4)+ a5*pow(t,5));
    }
}

void MainWindow::btnAppendClicked()
{
    if (rowClickedIndex >= 0){
        pathModel->insertRow(pathModel->rowCount());
    }
}

void MainWindow::btnInsertClicked()
{
    if (rowClickedIndex >= 0){
        pathModel->insertRow(rowClickedIndex);
    }
}

void MainWindow::btnDeleteClicked()
{
    if (rowClickedIndex >= 0){
        pathModel->removeRows(rowClickedIndex, 1);
        rowClickedIndex = -1;
    }
    if(pathModel->rowCount() == 0){
        pathModel->insertRow(0);
    }
}

void MainWindow::btnClearClicked()
{
    pathModel->removeRows(0, pathModel->rowCount());
    pathModel->insertRow(0);
}

void MainWindow::btnApplyClicked()
{
    int8_t tvRow = static_cast<int8_t>(pathModel->rowCount());

    for(int i = 0; i < tvRow; i++){
        t.push_back(pathModel->data(pathModel->index(i, 0)).toDouble());
        px.push_back(pathModel->data(pathModel->index(i, 1)).toDouble());
        py.push_back(pathModel->data(pathModel->index(i, 2)).toDouble());
        pz.push_back(pathModel->data(pathModel->index(i, 3)).toDouble());
        at.push_back(pathModel->data(pathModel->index(i, 4)).toDouble());
    }

    for(int i = 0; i < tvRow; i++){
        qDebug() << t[i] << "\t" << px[i] << "\t" << py[i] << "\t" << pz[i] << "\t" << at[i];
    }

    ui->btnRun->setEnabled(true);
}

void MainWindow::btnRunClicked()
{
    if(t.size() > 0){
        premotionThreadRun();
    }
}

void MainWindow::btnStopClicked()
{
    memcpy(desired_pos, present_pos, sizeof(double)*NUM_DOF);
    run_cmd = false;
    motion_thread_run = false;
    premotionThreadStop();
}

void MainWindow::tvCellClicked(const QModelIndex &index)
{
    rowClickedIndex = index.row();
    colClickedIndex = index.column();
//    qDebug() << "Clicked row : " << rowClickedIndex << ", col : " << colClickedIndex;
}

void MainWindow::verticalSectionClicked(int index)
{
    rowClickedIndex = index;
//    qDebug() << "Clicked vertical : " << index;
    colClickedIndex = -1;
}

void MainWindow::horizontalSectionClicked(int index)
{
    colClickedIndex = index;
//    qDebug() << "Clicked horizontal : " << index;
    rowClickedIndex = -1;
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_F11){
//        qDebug() << "Pressed F11";
        QString path = qApp->applicationDirPath();
        if (ui->gbMotion->isEnabled()){
            ui->tvPathData->model()->removeRows(0, ui->tvPathData->model()->rowCount());
            ui->tvPathData->model()->insertRows(0,6);

            double path[6*5] = {
                0.000,   0.00, 0.25, -0.015, 0.0002,
                0.003,  -0.17, 0.25, -0.015, 0.0002,
                0.006,  -0.17, 0.25,  0.045, 0.0002,
                0.009,   0.17, 0.25,  0.045, 0.0002,
                0.012,  0.17, 0.25, -0.015, 0.0002,
                0.015,  0.00, 0.25, -0.015, 0.0002
            };

            for(int i = 0; i < 6; i++){
                for(int j = 0; j < 5; j++){
                    QModelIndex indx = ui->tvPathData->model()->index(i, j, QModelIndex());
                    ui->tvPathData->model()->setData(indx, QString::number(path[i*5 + j]*1000));
                }
            }
        }
    }
}
