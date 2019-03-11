#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QDebug"
#include "qmessagebox.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    deviceInitial();
    UIInitial();
}

void MainWindow::deviceInitial()
{
    QString sensorType, table, IsInitialed;
    bool flag;
    paraType_.insert(std::pair<std::string, int>("sensitivity", SENSITIVITY));
    paraType_.insert(std::pair<std::string, int>("damp", DAMP));
    paraType_.insert(std::pair<std::string, int>("stiffness", STIFFNESS));
    paraType_.insert(std::pair<std::string, int>("threshold", THRESHOLD));
    paraType_.insert(std::pair<std::string, int>("limit", LIMIT));
    paraType_.insert(std::pair<std::string, int>("pos", POS));

    ft_sensor_data_process_ = new FTSensorDataProcess();
    robot_control_ = new RobotControl();
    flag = ft_sensor_data_process_->openDatabase("global");
//    table = QString("create table base(para varchar(20) primary key, ""name varchar(20))");
//    flag = ft_sensor_data_process_->createFTDBTable(table);
//    flag = ft_sensor_data_process_->insertFTDBData("base","type","optoforce");

    flag = ft_sensor_data_process_->getFTDBData("base", "type", sensorType);
    if(!flag || sensorType == "")
    {
        QString str = "Data base error!";
        QMessageBox::information(this,"Title",str);
        //default sensor -> optoforce
        flag = ft_sensor_data_process_->openDatabase("optoforce");
        ui->cBSensorName->setCurrentText("optoforce");
        flag = ft_sensor_data_process_->sensorTypeSelect(sensorType);
    }
    else
    {
        flag = ft_sensor_data_process_->openDatabase(sensorType);
        ui->cBSensorName->addItem("optoforce");
        ui->cBSensorName->addItem("Robotiq");
        ui->cBSensorName->addItem("ATI");
        int index = (sensorType=="optoforce")?0:(sensorType=="Robotiq")?1:2;
        ui->cBSensorName->setCurrentIndex(index);
    }

    flag = ft_sensor_data_process_->getFTDBData("base", "type", IsInitialed);
    if(IsInitialed == "")
    {
        table = QString("create table base(para varchar(20) primary key, ""name varchar(20))");
        flag = ft_sensor_data_process_->createFTDBTable(table);
        flag = ft_sensor_data_process_->insertFTDBData("base","dragMode","position");
        flag = ft_sensor_data_process_->insertFTDBData("base","calculateMethod","Jacobian");
        flag = ft_sensor_data_process_->insertFTDBData("base","controlModel","velocity");
        flag = ft_sensor_data_process_->insertFTDBData("base","controlPeriod","5");
        flag = ft_sensor_data_process_->insertFTDBData("base","bufferSizeLimit","42");
        flag = ft_sensor_data_process_->insertFTDBData("base","filter1","0");
        flag = ft_sensor_data_process_->insertFTDBData("base","filter2","0");
        double value[6] = {0.0};
        table = QString("create table parameter(para varchar(20) primary key, ""v1 varchar(20), v2 varchar(20),v3 varchar(20),v4 varchar(20),v5 varchar(20),v6 varchar(20))");
        flag = ft_sensor_data_process_->createFTDBTable(table);
        flag = ft_sensor_data_process_->insertFTDBData("parameter","sensitivity", value);
        flag = ft_sensor_data_process_->insertFTDBData("parameter","damp", value);
        flag = ft_sensor_data_process_->insertFTDBData("parameter","stiffness", value);
        flag = ft_sensor_data_process_->insertFTDBData("parameter","threshold", value);
        flag = ft_sensor_data_process_->insertFTDBData("parameter","limit", value);
        flag = ft_sensor_data_process_->insertFTDBData("parameter","pos", value);
        flag = ft_sensor_data_process_->insertFTDBData("parameter","ft","0");

        flag = ft_sensor_data_process_->insertFTDBData("base","IsInitialed","yes");
    }
    updateUI();

    flag = robot_control_->initRobotService();
    connect(RobotControl::instance(), SIGNAL(signal_handduiding_failed()),
            this, SLOT(slot_handduiding_failed()));
    if(!flag)
    {
        QString str = QString("connect to control box failed!");
        QMessageBox::information(this,"Title",str);
    }
    else
    {
        robot_control_->setToolProperty();
        hand_guiding_ = new std::thread(boost::bind(&MainWindow::handGuiding, this));
    }
}

void MainWindow::handGuiding()
{
    robot_control_->startHandGuiding();
}

void MainWindow::UIInitial()
{
    ui->lEPosX->setValidator(new QDoubleValidator(-1.0, 1.0, 5, this));
    ui->lEPosY->setValidator(new QDoubleValidator(-1.0, 1.0, 5, this));
    ui->lEPosZ->setValidator(new QDoubleValidator(-1.0, 1.0, 5, this));
    ui->lEPosRX->setValidator(new QDoubleValidator(-180.0, 180.0, 3, this));
    ui->lEPosRY->setValidator(new QDoubleValidator(-180.0, 180.0, 3, this));
    ui->lEPosRZ->setValidator(new QDoubleValidator(-180.0, 180.0, 3, this));
    ui->lEControlPeriod->setValidator(new QIntValidator(3, 15, this));
    ui->lEBuffSizeLimit->setValidator(new QIntValidator(18, 72, this));

    timer_.setInterval(200);
    connect(&timer_, SIGNAL(timeout()), this, SLOT(updateData()));
    timer_.start();
}

MainWindow::~MainWindow()
{
    robot_control_->s_thread_handguiding = false;
    usleep(10*1000);
    hand_guiding_->join();
    if(hand_guiding_ != NULL)
        delete hand_guiding_;
    delete robot_control_;
    delete ft_sensor_data_process_;
    delete ui;
}

void MainWindow::updateUI()
{
    bool flag;
    QString dragMode, calculateMethod, controlModel, controlPeriod, bufferSizeLimit, filter1, filter2;
    flag = ft_sensor_data_process_->getFTDBData("base", "dragMode", dragMode);
    if(dragMode == "position")
    {
        ui->rBPos->setChecked(true);
        FTSensorDataProcess::s_dragMode = 0;
    }
    else if(dragMode == "ori")
    {
        ui->rBOri->setChecked(true);
        FTSensorDataProcess::s_dragMode = 1;
    }
    else
    {
        ui->rBPose->setChecked(true);
        FTSensorDataProcess::s_dragMode = 2;
    }

    flag = ft_sensor_data_process_->getFTDBData("base", "calculateMethod", calculateMethod);
    if(calculateMethod == "Jacobian")
    {
        ui->rBJacobian->setChecked(true);
        FTSensorDataProcess::s_calculateMethod = 0;
    }
    else
    {
        ui->rBIK->setChecked(true);
        FTSensorDataProcess::s_calculateMethod = 1;
    }

    flag = ft_sensor_data_process_->getFTDBData("base", "controlModel", controlModel);
    if(controlModel == "velocity")
    {
        ui->rBVelocity->setChecked(true);
        FTSensorDataProcess::s_controlModel = 0;
    }
    else
    {
        ui->rBAcceleration->setChecked(true);
        FTSensorDataProcess::s_controlModel = 1;
    }

    flag = ft_sensor_data_process_->getFTDBData("base", "controlPeriod", controlPeriod);
    ui->lEControlPeriod->setText(controlPeriod);
    RobotControl::s_control_period = controlPeriod.toInt();

    flag = ft_sensor_data_process_->getFTDBData("base", "bufferSizeLimit", bufferSizeLimit);
    ui->lEBuffSizeLimit->setText(bufferSizeLimit);
    FTSensorDataProcess::s_bufferSizeLimit = bufferSizeLimit.toInt();

    flag = ft_sensor_data_process_->getFTDBData("base", "filter1", filter1);
    ui->hSFilter1->setValue(filter1.toInt());
    FTSensorDataProcess::s_filter1 = filter1.toInt();

    flag = ft_sensor_data_process_->getFTDBData("base", "filter2", filter2);
    ui->hSFilter2->setValue(filter2.toInt());
    FTSensorDataProcess::s_filter2 = filter2.toInt();

    flag = ft_sensor_data_process_->getFTDBData("parameter", "sensitivity", FTSensorDataProcess::s_sensitivity);
    ui->lESensitivityFx->setText(QString::number(FTSensorDataProcess::s_sensitivity[0]));
    ui->lESensitivityFy->setText(QString::number(FTSensorDataProcess::s_sensitivity[1]));
    ui->lESensitivityFz->setText(QString::number(FTSensorDataProcess::s_sensitivity[2]));
    ui->lESensitivityTx->setText(QString::number(FTSensorDataProcess::s_sensitivity[3]));
    ui->lESensitivityTy->setText(QString::number(FTSensorDataProcess::s_sensitivity[4]));
    ui->lESensitivityTz->setText(QString::number(FTSensorDataProcess::s_sensitivity[5]));

    flag = ft_sensor_data_process_->getFTDBData("parameter", "damp", FTSensorDataProcess::s_damp);
    ui->lEDampVx->setText(QString::number(FTSensorDataProcess::s_damp[0]));
    ui->lEDampVy->setText(QString::number(FTSensorDataProcess::s_damp[1]));
    ui->lEDampVz->setText(QString::number(FTSensorDataProcess::s_damp[2]));
    ui->lEDampWx->setText(QString::number(FTSensorDataProcess::s_damp[3]));
    ui->lEDampWy->setText(QString::number(FTSensorDataProcess::s_damp[4]));
    ui->lEDampWz->setText(QString::number(FTSensorDataProcess::s_damp[5]));

    flag = ft_sensor_data_process_->getFTDBData("parameter", "stiffness", FTSensorDataProcess::s_stiffness);
    ui->lEStiffPosX->setText(QString::number(FTSensorDataProcess::s_stiffness[0]));
    ui->lEStiffPosY->setText(QString::number(FTSensorDataProcess::s_stiffness[1]));
    ui->lEStiffPosZ->setText(QString::number(FTSensorDataProcess::s_stiffness[2]));
    ui->lEStiffOriX->setText(QString::number(FTSensorDataProcess::s_stiffness[3]));
    ui->lEStiffOriY->setText(QString::number(FTSensorDataProcess::s_stiffness[4]));
    ui->lEStiffOriZ->setText(QString::number(FTSensorDataProcess::s_stiffness[5]));


    flag = ft_sensor_data_process_->getFTDBData("parameter", "threshold", FTSensorDataProcess::s_threshold);
    ui->lEForceXThreshold->setText(QString::number(FTSensorDataProcess::s_threshold[0]));
    ui->lEForceYThreshold->setText(QString::number(FTSensorDataProcess::s_threshold[1]));
    ui->lEForceZThreshold->setText(QString::number(FTSensorDataProcess::s_threshold[2]));
    ui->lETorqueXThreshold->setText(QString::number(FTSensorDataProcess::s_threshold[3]));
    ui->lETorqueYThreshold->setText(QString::number(FTSensorDataProcess::s_threshold[4]));
    ui->lETorqueZThreshold->setText(QString::number(FTSensorDataProcess::s_threshold[5]));

    flag = ft_sensor_data_process_->getFTDBData("parameter", "limit", FTSensorDataProcess::s_limit);
    ui->lEForceXLimit->setText(QString::number(FTSensorDataProcess::s_limit[0]));
    ui->lEForceYLimit->setText(QString::number(FTSensorDataProcess::s_limit[1]));
    ui->lEForceZLimit->setText(QString::number(FTSensorDataProcess::s_limit[2]));
    ui->lETorqueXLimit->setText(QString::number(FTSensorDataProcess::s_limit[3]));
    ui->lETorqueYLimit->setText(QString::number(FTSensorDataProcess::s_limit[4]));
    ui->lETorqueZLimit->setText(QString::number(FTSensorDataProcess::s_limit[5]));

    flag = ft_sensor_data_process_->getFTDBData("parameter", "pos", RobotControl::s_tool_pose);
    ui->lEPosX->setText(QString::number(RobotControl::s_tool_pose[0]));
    ui->lEPosY->setText(QString::number(RobotControl::s_tool_pose[1]));
    ui->lEPosZ->setText(QString::number(RobotControl::s_tool_pose[2]));
    ui->lEPosRX->setText(QString::number(RobotControl::s_tool_pose[3]));
    ui->lEPosRY->setText(QString::number(RobotControl::s_tool_pose[4]));
    ui->lEPosRZ->setText(QString::number(RobotControl::s_tool_pose[5]));

}

void MainWindow::on_cBSensorName_currentIndexChanged(int index)
{
    bool flag;
    QString sensorType = ui->cBSensorName->currentText();
    flag = ft_sensor_data_process_->openDatabase("global");
    flag = ft_sensor_data_process_->setFTDBData("base", "type", sensorType);
    flag = ft_sensor_data_process_->openDatabase(sensorType);
    if(!flag)
    {
        QString str = "Data base error!";
        QMessageBox::information(this,"Title",str);
    }
    else
        updateUI();

    flag = ft_sensor_data_process_->sensorTypeSelect(sensorType);
    if(!flag)
    {
        ui->pBStart->setEnabled(false);
        QString str = QString("Cannot connect to the %1 sensor").arg(sensorType);
        QMessageBox::information(this,"Title",str);
    }
    else
    {
//        ui->pBStart->setEnabled(true);
    }
}

void MainWindow::on_pBPos1_clicked()
{
    ui->pBPos1->setEnabled(false);
    if(robot_control_->moveToTargetPose(1) == 0)
        ft_sensor_data_process_->obtainCalibrationPos(1); //calibration sensor data of pose1?;
}

void MainWindow::on_pBPos2_clicked()
{
    ui->pBPos2->setEnabled(false);
    if(robot_control_->moveToTargetPose(2) == 0)
        ft_sensor_data_process_->obtainCalibrationPos(2);
}

void MainWindow::on_pBPos3_clicked()
{
    ui->pBPos3->setEnabled(false);
    if(robot_control_->moveToTargetPose(3) == 0)
        ft_sensor_data_process_->obtainCalibrationPos(3);
}

void MainWindow::on_pBCalibration_clicked()
{
    if(ui->pBCalibration->text() == "Calibrate")
    {
        if(ui->pBPos1->isEnabled() || ui->pBPos2->isEnabled() || ui->pBPos3->isEnabled())
        {

            if(!ft_sensor_data_process_->getFTSensorOffsetFromDB())
            {
                QString str = "Get sensor offset failed!";
                QMessageBox::information(this,"Title",str);
            }
            else
                ui->pBStart->setEnabled(true);
        }
        else
        {
            if(robot_control_->ObtainCenterofMass())
            {
                ft_sensor_data_process_->setFTSensorOffsetToDB();
                ui->pBStart->setEnabled(true);
            }
        }
        ui->pBPos1->setEnabled(false);
        ui->pBPos2->setEnabled(false);
        ui->pBPos3->setEnabled(false);

        ui->pBCalibration->setText("Recalibrate");
    }
    else
    {
        ui->pBCalibration->setText("Calibrate");
        ui->pBPos1->setEnabled(true);
        ui->pBPos2->setEnabled(true);
        ui->pBPos3->setEnabled(true);
    }
}

void MainWindow::updateData()
{
    double data[SENSOR_DIMENSION]={0};
    memcpy(data, FTSensorDataProcess::s_sensor_data, sizeof(double)*SENSOR_DIMENSION);
    ui->lEForceX->setText(QString::number(data[0]));
    ui->lEForceY->setText(QString::number(data[1]));
    ui->lEForceZ->setText(QString::number(data[2]));
    ui->lETorqueX->setText(QString::number(data[3]));
    ui->lETorqueY->setText(QString::number(data[4]));
    ui->lETorqueZ->setText(QString::number(data[5]));
}

void MainWindow::on_rBPos_clicked()
{
    ft_sensor_data_process_->setFTDBData("base", "dragMode", "position");
    FTSensorDataProcess::s_dragMode = DRAG_MODE::POSITION;
}

void MainWindow::on_rBOri_clicked()
{
    ft_sensor_data_process_->setFTDBData("base", "dragMode", "ori");
    FTSensorDataProcess::s_dragMode = 1;
}

void MainWindow::on_rBPose_clicked()
{
    ft_sensor_data_process_->setFTDBData("base", "dragMode", "pose");
    FTSensorDataProcess::s_dragMode = 2;
}

void MainWindow::on_rBJacobian_clicked()
{
    ft_sensor_data_process_->setFTDBData("base", "calculateMethod", "Jacobian");
    FTSensorDataProcess::s_calculateMethod = 0;
}

void MainWindow::on_rBIK_clicked()
{
    ft_sensor_data_process_->setFTDBData("base", "calculateMethod", "IK");
    FTSensorDataProcess::s_calculateMethod = 1;
}

void MainWindow::on_rBVelocity_clicked()
{
    ft_sensor_data_process_->setFTDBData("base", "controlModel", "velocity");
    FTSensorDataProcess::s_controlModel = 0;
}

void MainWindow::on_rBAcceleration_clicked()
{
    ft_sensor_data_process_->setFTDBData("base", "controlModel", "acceleration");
    FTSensorDataProcess::s_controlModel = 1;
}

void MainWindow::on_hSFilter1_valueChanged(int value)
{
    ft_sensor_data_process_->setFTDBData("base", "filter1", QString::number(value));
    FTSensorDataProcess::s_filter1 = value;
}

void MainWindow::on_hSFilter2_valueChanged(int value)
{
    ft_sensor_data_process_->setFTDBData("base", "filter2", QString::number(value));
    FTSensorDataProcess::s_filter2 = value;
}

void MainWindow::on_lEControlPeriod_textChanged(const QString &arg1)
{
    ft_sensor_data_process_->setFTDBData("base", "controlPeriod", arg1);
    RobotControl::s_control_period = arg1.toInt();
}

void MainWindow::on_lEBuffSizeLimit_textChanged(const QString &arg1)
{
    ft_sensor_data_process_->setFTDBData("base", "bufferSizeLimit", arg1);
    FTSensorDataProcess::s_bufferSizeLimit = arg1.toInt();
}

void MainWindow::updateDataBase(QString arg1, QString table, QString name, int index)
{
    ft_sensor_data_process_->setFTDBData(table, name, arg1, index);
    int para_type = paraType_[name.toStdString()];

    switch(para_type)
    {
        case SENSITIVITY: FTSensorDataProcess::s_sensitivity[index-1] = arg1.toDouble();break;
        case DAMP: FTSensorDataProcess::s_damp[index-1] = arg1.toDouble();break;
        case STIFFNESS: FTSensorDataProcess::s_stiffness[index-1] = arg1.toDouble();break;
        case THRESHOLD: FTSensorDataProcess::s_threshold[index-1] = arg1.toDouble();break;
        case LIMIT: FTSensorDataProcess::s_limit[index-1] = arg1.toDouble();break;
        case POS: RobotControl::s_tool_pose[index-1] = arg1.toDouble();break;
    }
}

void MainWindow::on_lESensitivityFz_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "sensitivity", 1);
}

void MainWindow::on_lESensitivityFx_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "sensitivity", 2);
}

void MainWindow::on_lESensitivityFy_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "sensitivity", 3);
}

void MainWindow::on_lESensitivityTx_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "sensitivity", 4);
}

void MainWindow::on_lESensitivityTy_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "sensitivity", 5);
}

void MainWindow::on_lESensitivityTz_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "sensitivity", 6);
}

void MainWindow::on_lEDampVx_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "damp", 1);
}

void MainWindow::on_lEDampVy_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "damp", 2);
}

void MainWindow::on_lEDampVz_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "damp", 3);
}

void MainWindow::on_lEDampWx_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "damp", 4);
}

void MainWindow::on_lEDampWy_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "damp", 5);
}

void MainWindow::on_lEDampWz_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "damp", 6);
}

void MainWindow::on_lEStiffPosX_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "stiffness", 1);
}

void MainWindow::on_lEStiffPosY_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "stiffness", 2);
}

void MainWindow::on_lEStiffPosZ_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "stiffness", 3);
}

void MainWindow::on_lEStiffOriX_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "stiffness", 4);
}

void MainWindow::on_lEStiffOriY_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "stiffness", 5);
}

void MainWindow::on_lEStiffOriZ_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "stiffness", 6);
}

void MainWindow::on_lEForceXThreshold_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "threshold", 1);
}

void MainWindow::on_lEForceYThreshold_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "threshold", 2);
}

void MainWindow::on_lEForceZThreshold_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "threshold", 3);
}

void MainWindow::on_lETorqueXThreshold_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "threshold", 4);
}

void MainWindow::on_lETorqueYThreshold_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "threshold", 5);
}

void MainWindow::on_lETorqueZThreshold_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "threshold", 6);
}

void MainWindow::on_lEForceXLimit_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "limit", 1);
}

void MainWindow::on_lEForceYLimit_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "limit", 2);
}

void MainWindow::on_lEForceZLimit_textChanged(const QString &arg1)
{
     updateDataBase(arg1, "parameter", "limit", 3);
}

void MainWindow::on_lETorqueXLimit_textChanged(const QString &arg1)
{
     updateDataBase(arg1, "parameter", "limit", 4);
}

void MainWindow::on_lETorqueYLimit_textChanged(const QString &arg1)
{
     updateDataBase(arg1, "parameter", "limit", 5);
}

void MainWindow::on_lETorqueZLimit_textChanged(const QString &arg1)
{
     updateDataBase(arg1, "parameter", "limit", 6);
}

void MainWindow::on_lEPosX_textChanged(const QString &arg1)
{
     updateDataBase(arg1, "parameter", "pos", 1);
     robot_control_->setToolProperty();
}

void MainWindow::on_lEPosY_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "pos", 2);
    robot_control_->setToolProperty();
}

void MainWindow::on_lEPosZ_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "pos", 3);
    robot_control_->setToolProperty();
}

void MainWindow::on_lEPosRX_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "pos", 4);
    robot_control_->setToolProperty();
}

void MainWindow::on_lEPosRY_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "pos", 5);
    robot_control_->setToolProperty();
}

void MainWindow::on_lEPosRZ_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "pos", 6);
    robot_control_->setToolProperty();
}

void MainWindow::on_pBStart_clicked()
{
    if(ui->pBStart->text() == "Start")
    {
        ui->pBStart->setText("Stop");
        robot_control_->enterTcp2CANMode(true);
        ui->cBSensorName->setEnabled(false);
    }
    else
    {
        ui->pBStart->setText("Start");
        robot_control_->enterTcp2CANMode(false);
        ui->cBSensorName->setEnabled(true);
    }
}

void MainWindow::slot_handduiding_failed()
{
    ui->pBStart->setText("Start");
    robot_control_->enterTcp2CANMode(false);
}

void MainWindow::on_lEOutPut_textEdited(const QString &arg1)
{
    ui->lEOutPut->setText("");
}

void MainWindow::on_pBRobot_clicked()
{
//    ui->pBRobot->setEnabled(false);
    robot_control_->initRobotService();
}
