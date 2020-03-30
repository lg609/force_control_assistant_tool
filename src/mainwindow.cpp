#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QDebug"
#include "qmessagebox.h"

HandGuidingForm::HandGuidingForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::HandGuidingForm)
{
    ui->setupUi(this);
    robot_control_ = new RobotControl("aubo_i5");
    ft_sensor_data_process_ = new FTSensorDataProcess();
    ft_sensor_util_ = new FTSensorUtil();

    initialDevice();
    initialUI();
}

HandGuidingForm::~HandGuidingForm()
{
    delete robot_control_;
    delete ft_sensor_data_process_;
    delete ft_sensor_util_;
    delete ui;
}

void HandGuidingForm::initialDevice()
{
    displayMessage("Initial F/T sensor...", 2000);
    QString sensorType, connectName, table, IsInitialed;
    bool flag;
    double data[6] = {0.0};

    if(!ft_sensor_util_->openDatabase("global"))
    {
        QString str = "Data base error!";
        displayMessage(str);
        return;
    }
//    table = QString("create table calibrate(para varchar(20) primary key, ""v1 varchar(20), v2 varchar(20),v3 varchar(20),v4 varchar(20),v5 varchar(20),v6 varchar(20))");
//    flag = ft_sensor_data_process_->createFTDBTable(table);
//    double value[6] = {0};
//    flag = ft_sensor_data_process_->insertFTDBData("calibrate","pos1", value);
//    flag = ft_sensor_data_process_->insertFTDBData("calibrate","pos2", value);
//    flag = ft_sensor_data_process_->insertFTDBData("calibrate","pos3", value);

    for(int i = 0; i< CALIBRATION_POS::POSE_Total; i++)
    {
        flag = ft_sensor_util_->getFTDBData("calibrate", "pos"+QString::number(i+1), data);
//        robot_control_->setCalibrationPose(data, i);
    }

    flag = ft_sensor_util_->insertFTDBData("base","connectName", "ttyUSB1");
    flag = ft_sensor_util_->getFTDBData("base", "connectName", connectName);
    ft_sensor_data_process_->setConnectName(connectName.toStdString());
    flag = ft_sensor_util_->getFTDBData("base", "type", sensorType);
    if(!flag || sensorType == "")
    {
        QString str = "Data base error!";
        displayMessage(str);
        //default sensor -> optoforce
        flag = ft_sensor_util_->openDatabase("optoforce");
        ui->cBSensorName->setCurrentText("optoforce");
        flag = ft_sensor_data_process_->sensorTypeSelect(sensorType.toStdString());
    }
    else
    {
        flag = ft_sensor_util_->openDatabase(sensorType);
        cBSensorName_add_finished_ = false;
        ui->cBSensorName->addItem("optoforce");
        ui->cBSensorName->addItem("Robotiq");
        ui->cBSensorName->addItem("ATI");
        ui->cBSensorName->addItem("KunWei");
        cBSensorName_add_finished_ = true;
        int index = (sensorType=="optoforce")?0:(sensorType=="Robotiq")?1:(sensorType=="ATI")?2:3;
        ui->cBSensorName->setCurrentIndex(index);
    }

    flag = ft_sensor_util_->getFTDBData("base", "type", IsInitialed);
    if(IsInitialed == "")
    {
        table = QString("create table base(para varchar(20) primary key, ""name varchar(20))");
        flag = ft_sensor_util_->createFTDBTable(table);
        flag = ft_sensor_util_->insertFTDBData("base","dragMode","position");
        flag = ft_sensor_util_->insertFTDBData("base","calculateMethod","Jacobian");
        flag = ft_sensor_util_->insertFTDBData("base","controlModel","velocity");
        flag = ft_sensor_util_->insertFTDBData("base","controlPeriod","5");
        flag = ft_sensor_util_->insertFTDBData("base","bufferSizeLimit","42");
        flag = ft_sensor_util_->insertFTDBData("base","filter1","0");
        flag = ft_sensor_util_->insertFTDBData("base","filter2","0");
        double value[6] = {0.0};
        table = QString("create table parameter(para varchar(20) primary key, ""v1 varchar(20), v2 varchar(20),v3 varchar(20),v4 varchar(20),v5 varchar(20),v6 varchar(20))");
        flag = ft_sensor_util_->createFTDBTable(table);
        flag = ft_sensor_util_->insertFTDBData("parameter","sensitivity", value);
        flag = ft_sensor_util_->insertFTDBData("parameter","damp", value);
        flag = ft_sensor_util_->insertFTDBData("parameter","stiffness", value);
        flag = ft_sensor_util_->insertFTDBData("parameter","threshold", value);
        flag = ft_sensor_util_->insertFTDBData("parameter","limit", value);
        flag = ft_sensor_util_->insertFTDBData("parameter","pos", value);
        flag = ft_sensor_util_->insertFTDBData("parameter","ft","0");

        flag = ft_sensor_util_->insertFTDBData("base","IsInitialed","yes");
    }
    updateUI();

    ui->cBRealTime->setCheckState(Qt::Checked);
    displayMessage("Initial robot service...", 2000);
//    connect(ft_sensor_util_, SIGNAL(signal_sensor_over_range(QString)), this, SLOT(slot_sensor_overrange(QString)));
//    connect(robot_control_, SIGNAL(signal_handduiding_failed(QString)), this, SLOT(slot_handduiding_failed(QString)));
}

void HandGuidingForm::updateUI()
{
    bool flag = false;
    QString dragMode, calculateMethod, controlModel, controlSpace, controlPeriod, bufferSizeLimit, filter1, filter2;
    flag = ft_sensor_util_->getFTDBData("base", "dragMode", dragMode);
    if(dragMode == "position")
    {
        ui->rBPos->setChecked(true);
        robot_control_->setDragMode(0);
    }
    else if(dragMode == "ori")
    {
        ui->rBOri->setChecked(true);
        robot_control_->setDragMode(1);
    }
    else
    {
        ui->rBPose->setChecked(true);
        robot_control_->setDragMode(2);
    }

    flag = ft_sensor_util_->getFTDBData("base", "calculateMethod", calculateMethod);
    if(calculateMethod == "Jacobian")
    {
        ui->rBJacobian->setChecked(true);
        robot_control_->setCalculateMethod(0);
    }
    else
    {
        ui->rBIK->setChecked(true);
        robot_control_->setCalculateMethod(1);
    }

    flag = ft_sensor_util_->getFTDBData("base", "controlModel", controlModel);
    if(controlModel == "velocity")
    {
        ui->rBVelocity->setChecked(true);
        robot_control_->setControlModel(0);
    }
    else
    {
        ui->rBAcceleration->setChecked(true);
        robot_control_->setControlModel(1);
    }

    flag = ft_sensor_util_->getFTDBData("base", "controlSpace", controlSpace);
    if(controlSpace == "jointSpace")
    {
        ui->rBJointSpace->setChecked(true);
        robot_control_->setControlSpace(0);
    }
    else
    {
        ui->rBOperateSpace->setChecked(true);
        robot_control_->setControlSpace(1);
    }

    flag = ft_sensor_util_->getFTDBData("base", "controlPeriod", controlPeriod);
    ui->lEControlPeriod->setText(controlPeriod);
    robot_control_->setControlPeriod(controlPeriod.toInt());

    flag = ft_sensor_util_->getFTDBData("base", "bufferSizeLimit", bufferSizeLimit);
    ui->lEBuffSizeLimit->setText(bufferSizeLimit);
    robot_control_->setBufferSizeLimit(bufferSizeLimit.toInt());

    flag = ft_sensor_util_->getFTDBData("base", "filter1", filter1);
    ui->hSFilter1->setValue(filter1.toInt());
    robot_control_->setFilter1(filter1.toInt());

    flag = ft_sensor_util_->getFTDBData("base", "filter2", filter2);
    ui->hSFilter2->setValue(filter2.toInt());
    robot_control_->setFilter2(filter2.toInt());

    double data[6] = {0.0};
    flag = ft_sensor_util_->getFTDBData("parameter", "sensitivity", data);
    robot_control_->setMass(data);
    ui->lESensitivityFx->setText(QString::number(data[0]));
    ui->lESensitivityFy->setText(QString::number(data[1]));
    ui->lESensitivityFz->setText(QString::number(data[2]));
    ui->lESensitivityTx->setText(QString::number(data[3]));
    ui->lESensitivityTy->setText(QString::number(data[4]));
    ui->lESensitivityTz->setText(QString::number(data[5]));

    flag = ft_sensor_util_->getFTDBData("parameter", "damp", data);
    robot_control_->setDamp(data);
    ui->lEDampVx->setText(QString::number(data[0]));
    ui->lEDampVy->setText(QString::number(data[1]));
    ui->lEDampVz->setText(QString::number(data[2]));
    ui->lEDampWx->setText(QString::number(data[3]));
    ui->lEDampWy->setText(QString::number(data[4]));
    ui->lEDampWz->setText(QString::number(data[5]));

    flag = ft_sensor_util_->getFTDBData("parameter", "stiffness", data);
    robot_control_->setStiffness(data);
    ui->lEStiffPosX->setText(QString::number(data[0]));
    ui->lEStiffPosY->setText(QString::number(data[1]));
    ui->lEStiffPosZ->setText(QString::number(data[2]));
    ui->lEStiffOriX->setText(QString::number(data[3]));
    ui->lEStiffOriY->setText(QString::number(data[4]));
    ui->lEStiffOriZ->setText(QString::number(data[5]));

    flag = ft_sensor_util_->getFTDBData("parameter", "threshold", data);
    robot_control_->setThreshold(data);
    ui->lEForceXThreshold->setText(QString::number(data[0]));
    ui->lEForceYThreshold->setText(QString::number(data[1]));
    ui->lEForceZThreshold->setText(QString::number(data[2]));
    ui->lETorqueXThreshold->setText(QString::number(data[3]));
    ui->lETorqueYThreshold->setText(QString::number(data[4]));
    ui->lETorqueZThreshold->setText(QString::number(data[5]));

    flag = ft_sensor_util_->getFTDBData("parameter", "limit", data);
    robot_control_->setLimit(data);
    ui->lEForceXLimit->setText(QString::number(data[0]));
    ui->lEForceYLimit->setText(QString::number(data[1]));
    ui->lEForceZLimit->setText(QString::number(data[2]));
    ui->lETorqueXLimit->setText(QString::number(data[3]));
    ui->lETorqueYLimit->setText(QString::number(data[4]));
    ui->lETorqueZLimit->setText(QString::number(data[5]));

    flag = ft_sensor_util_->getFTDBData("parameter", "pos", data);
    robot_control_->setToolPose(data);
    ui->lEPosX->setText(QString::number(data[0]));
    ui->lEPosY->setText(QString::number(data[1]));
    ui->lEPosZ->setText(QString::number(data[2]));
    ui->lEPosRX->setText(QString::number(data[3]));
    ui->lEPosRY->setText(QString::number(data[4]));
    ui->lEPosRZ->setText(QString::number(data[5]));
    displayMessage("Set Tool Property");
    robot_control_->setToolProperty();
}

void HandGuidingForm::initialUI()
{
    displayMessage("Initial UI...", 1000);

    ui->lEPosX->setValidator(new QDoubleValidator(-1.0, 1.0, 5, this));
    ui->lEPosY->setValidator(new QDoubleValidator(-1.0, 1.0, 5, this));
    ui->lEPosZ->setValidator(new QDoubleValidator(-1.0, 1.0, 5, this));
    ui->lEPosRX->setValidator(new QDoubleValidator(-180.0, 180.0, 3, this));
    ui->lEPosRY->setValidator(new QDoubleValidator(-180.0, 180.0, 3, this));
    ui->lEPosRZ->setValidator(new QDoubleValidator(-180.0, 180.0, 3, this));
    ui->lEControlPeriod->setValidator(new QIntValidator(3, 15, this));
    ui->lEBuffSizeLimit->setValidator(new QIntValidator(18, 72, this));

    ui->gBControlSpace->setEnabled(false);
    ui->paintFrame->installEventFilter(this);
    ui->cB_Fx->setStyleSheet("color:red;");
    ui->cB_Fy->setStyleSheet("color:green;");
    ui->cB_Fz->setStyleSheet("color:blue;");
    ui->cB_Tx->setStyleSheet("color:magenta;");
    ui->cB_Ty->setStyleSheet("color:cyan;");
    ui->cB_Tz->setStyleSheet("color:darkGrey;");
//    ui->gBConstrains->setEnabled(false);

    timer_.setInterval(update_period_);
    connect(&timer_, SIGNAL(timeout()), this, SLOT(updateData()));
    timer_.start();
}

void HandGuidingForm::updateData()
{
//    Wrench endWrench = robot_control_->getRobotEndWrench();
//    ui->lEForceX->setText(QString::number(endWrench[0],'f',5));
//    ui->lEForceY->setText(QString::number(endWrench[1],'f',5));
//    ui->lEForceZ->setText(QString::number(endWrench[2],'f',5));
//    ui->lETorqueX->setText(QString::number(endWrench[3],'f',5));
//    ui->lETorqueY->setText(QString::number(endWrench[4],'f',5));
//    ui->lETorqueZ->setText(QString::number(endWrench[5],'f',5));

//    Wrench data = ft_sensor_data_process_->getSensorData();
//    ui->label_Fx->setText(QString::number(data[0],'f',5));
//    ui->label_Fy->setText(QString::number(data[1],'f',5));
//    ui->label_Fz->setText(QString::number(data[2],'f',5));
//    ui->label_Tx->setText(QString::number(data[3],'f',5));
//    ui->label_Ty->setText(QString::number(data[4],'f',5));
//    ui->label_Tz->setText(QString::number(data[5],'f',5));
//    ui->label_F3D->setText(QString::number(sqrt(data[0]*data[0]+data[1]*data[1]+data[2]*data[2]),'f',5));
//    ui->label_T3D->setText(QString::number(sqrt(data[3]*data[3]+data[4]*data[4]+data[5]*data[5]),'f',5));

     //scan IO information
    if(ft_sensor_data_process_->getSensorCalibrateStatus())
    {
        if(robot_control_->getHandGuidingSwitch() == 1)
            ui->pBStart->setStyleSheet("background-color:green");
        else
            ui->pBStart->setStyleSheet("background-color:red");
    }

    ui->paintFrame->update(); //    paintEvent();
    ui->cBRealTime->setCheckState(Qt::Checked);
    ft_sensor_util_->enableDisplaySenssorData();
}

void HandGuidingForm::slot_handduiding_failed(QString str)
{
    ui->pBStart->setText("Start");
//    robot_control_->enterTcp2CANMode(false);
    displayMessage(str, 2000);
    displayMessage("Press Reset Button, Then press Reclibrate, then press Pose3");
}

void HandGuidingForm::slot_sensor_overrange(QString str)
{
    ui->pBStart->setText("Start");
//    robot_control_->enterTcp2CANMode(false);
    displayMessage(str);
}

void HandGuidingForm::updateDataBase(QString arg1, QString table, QString name, int index)
{
    ft_sensor_util_->setFTDBData(table, name, arg1, index);
    robot_control_->updateControlPara(arg1.toDouble(), index-1, name.toStdString());

     //???
    double sigmaValue[6];
    robot_control_->getMaxSigma(sigmaValue);
    ui->label_Fx_Sigma->setText(QString::number(sigmaValue[0]));
    ui->label_Fy_Sigma->setText(QString::number(sigmaValue[1]));
    ui->label_Fz_Sigma->setText(QString::number(sigmaValue[2]));
}

/******************** Configuration ********************/
void HandGuidingForm::on_rBSpecifyTool_clicked()
{
    ui->toolFrame->setEnabled(true);
    ui->comBToolSet->setEnabled(false);
}

void HandGuidingForm::on_rBSelectTool_clicked()
{
    ui->toolFrame->setEnabled(false);
    ui->comBToolSet->setEnabled(true);
}

void HandGuidingForm::on_lEPosX_textChanged(const QString &arg1)
{
     updateDataBase(arg1, "parameter", "pos", 1);
     robot_control_->setToolProperty();
}

void HandGuidingForm::on_lEPosY_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "pos", 2);
    robot_control_->setToolProperty();
}

void HandGuidingForm::on_lEPosZ_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "pos", 3);
    robot_control_->setToolProperty();
}

void HandGuidingForm::on_lEPosRX_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "pos", 4);
    robot_control_->setToolProperty();
}

void HandGuidingForm::on_lEPosRY_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "pos", 5);
    robot_control_->setToolProperty();
}

void HandGuidingForm::on_lEPosRZ_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "pos", 6);
    robot_control_->setToolProperty();
}

void HandGuidingForm::on_lE_Port_editingFinished()
{
    bool flag;
    QString sensorType = ui->cBSensorName->currentText();
    QString connectName = ui->lE_Port->text();
    flag = ft_sensor_util_->openDatabase("global");
    flag = ft_sensor_util_->setFTDBData("base", "connectName", connectName);
    flag = ft_sensor_util_->openDatabase(sensorType);
    if(!flag)
    {
        QString str = "Data base error!";
        displayMessage(str);
    }
    else
    {
        flag = ft_sensor_data_process_->sensorTypeSelect(sensorType.toStdString(), connectName.toStdString());
        if(flag)
            ui->label_Connect->setText("Connect");
        else
            ui->label_Connect->setText("Disconnect");
    }

}

void HandGuidingForm::on_cBSensorName_currentIndexChanged(int index)
{
    if(!cBSensorName_add_finished_)
        return;
    bool flag;
    QString sensorType = ui->cBSensorName->currentText();
    flag = ft_sensor_util_->openDatabase("global");
    flag = ft_sensor_util_->setFTDBData("base", "type", sensorType);
    flag = ft_sensor_util_->openDatabase(sensorType);
    if(!flag)
    {
        QString str = "Data base error!";
        displayMessage(str);
    }
    else
        updateUI();

    flag = ft_sensor_data_process_->sensorTypeSelect(sensorType.toStdString(), ui->lE_Port->text().toStdString());
    if(!flag)
    {
        ui->pBStart->setEnabled(false);
        QString str = QString("Cannot connect to the %1 sensor").arg(sensorType);
        displayMessage(str);
    }
    else
    {
//        ui->pBStart->setEnabled(true);
    }
    displayMessage("Switch to " + sensorType + "sensor.");
}

void HandGuidingForm::on_pBCalibration_clicked()
{
    if(ui->pBCalibration->text() == "Calibrate")
    {
        double result[10] = {0.0};
        if(ui->pBPos1->isEnabled() || ui->pBPos2->isEnabled() || ui->pBPos3->isEnabled())
        {

            ::Wrench sensorOffset;
            double value[6] = {0.0};
            if(!ft_sensor_util_->getFTSensorOffsetFromDB(sensorOffset))
            {
                QString str = "Get sensor offset failed!";
                displayMessage(str);
            }
            else
            {
                ft_sensor_util_->getFTDBData("parameter", "toolProperty", value);
//                robot_control_->setToolDynamics(RigidBodyInertia(value[0],value[0]*Vector(value[1],value[2],value[3])));
                ui->pBStart->setEnabled(true);
                ft_sensor_data_process_->setSensorCalibrateStatus(true);
                displayMessage("obatin sensor offset from database.");
            }
            memcpy(result, value, sizeof(double)*4);
            memcpy(&result[4], sensorOffset.data, sizeof(double)*SENSOR_DIMENSION);
        }
        else
        {
            if(robot_control_->obtainCenterofMass(result))
            {
                bool flag;
                ::Wrench sensorOffset;
                memcpy(sensorOffset.data, &result[4], sizeof(double)*SENSOR_DIMENSION);
                ft_sensor_util_->setFTSensorOffsetToDB(sensorOffset);
                double value[6] = {0.0};
                flag = ft_sensor_util_->insertFTDBData("parameter","toolProperty", value);
//                double m = robot_control_->getToolDynamics().getMass();
//                Vector vec = robot_control_->getToolDynamics().getCOG();
//                flag = ft_sensor_data_process_->setFTDBData("parameter","toolProperty", QString::number(m), 1);
//                flag = ft_sensor_data_process_->setFTDBData("parameter","toolProperty", QString::number(vec(0)), 2);
//                flag = ft_sensor_data_process_->setFTDBData("parameter","toolProperty", QString::number(vec(1)), 3);
//                flag = ft_sensor_data_process_->setFTDBData("parameter","toolProperty", QString::number(vec(2)), 4);

                ui->pBStart->setEnabled(true);
                FTSensorDataProcess::s_sensor_data_calibrated = true;
                displayMessage("update sensor offset from measurment.");
//                QString str = "mass: " + QString::number(m) + "center:" + QString::number(vec(0),'f',4)
//                        + "," + QString::number(vec(1),'f',4) + "," + QString::number(vec(2), 'f', 4);
//                displayMessage(str);
            }
        }
        ui->lE_Weight->setText(QString::number(result[0],'f',4));
        ui->lE_Gx->setText(QString::number(result[1],'f',4));
        ui->lE_Gy->setText(QString::number(result[2],'f',4));
        ui->lE_Gz->setText(QString::number(result[3],'f',4));
        ui->lE_Fx_Bias->setText(QString::number(result[4],'f',4));
        ui->lE_Fy_Bias->setText(QString::number(result[5],'f',4));
        ui->lE_Fz_Bias->setText(QString::number(result[6],'f',4));
        ui->lE_Tx_Bias->setText(QString::number(result[7],'f',4));
        ui->lE_Ty_Bias->setText(QString::number(result[8],'f',4));
        ui->lE_Tz_Bias->setText(QString::number(result[9],'f',4));
        ui->pBPos1->setEnabled(false);
        ui->pBPos2->setEnabled(false);
        ui->pBPos3->setEnabled(false);

        ui->pBCalibration->setText("Recalibrate");
    }
    else
    {
        FTSensorDataProcess::s_sensor_data_calibrated = true;
        ui->pBStart->setEnabled(false);
        ui->pBCalibration->setText("Calibrate");
        ui->pBPos1->setEnabled(true);
        ui->pBPos2->setEnabled(true);
        ui->pBPos3->setEnabled(true);
    }
}

void HandGuidingForm::getCalibrationPose(int index)
{
    if(ui->cBFirst->checkState() == Qt::Checked)
    {
        double joint_angle[6];
        bool flag;
        robot_control_->getCalibrationPose(index, joint_angle);

        QString sensorType = ui->cBSensorName->currentText();
        flag = ft_sensor_util_->openDatabase("global");
        for(int i = 0; i < SENSOR_DIMENSION; i++)
            ft_sensor_util_->setFTDBData("calibrate","pos"+QString::number(index+1), QString::number(joint_angle[i]), i+1);
        flag = ft_sensor_util_->openDatabase(sensorType);
    }
    else
        robot_control_->moveToTargetPose(index);

    displayMessage("robot arrives at position"+QString::number(index+1));
    ft_sensor_data_process_->obtainCalibrationPos(index); //calibration sensor data of pose1
}

void HandGuidingForm::on_pBPos1_clicked()
{
     ui->pBPos1->setEnabled(false);
     getCalibrationPose(CALIBRATION_POS::POSE_X);
}

void HandGuidingForm::on_pBPos2_clicked()
{
    ui->pBPos2->setEnabled(false);
    getCalibrationPose(CALIBRATION_POS::POSE_Y);
}

void HandGuidingForm::on_pBPos3_clicked()
{
    ui->pBPos3->setEnabled(false);
    getCalibrationPose(CALIBRATION_POS::POSE_Z);
}


/******************** HandGuiding Function ********************/
void HandGuidingForm::on_pBRobot_clicked()
{
//    ui->pBRobot->setEnabled(false);
//    robot_control_->initRobotService();
}

void HandGuidingForm::displayMessage(const QString str, int timeout)
{
    ui->lbOutPut->setText(str);
}

void HandGuidingForm::on_lE_pos_wcr_textChanged(const QString &arg1)
{
//   robot_control_->getRobotKine()->s_w_cr[0] = arg1.toDouble();
}

void HandGuidingForm::on_lE_pos_wth_textChanged(const QString &arg1)
{
//  robot_control_->getRobotKine()->s_w_th[0] = arg1.toDouble();
}

void HandGuidingForm::on_lE_pos_lambda_textChanged(const QString &arg1)
{
//    robot_control_->getRobotKine()->s_w_lambda[0] = arg1.toDouble();
}

void HandGuidingForm::on_rBenable_constraints_clicked()
{
    robot_control_->enableConstraints();
}

void HandGuidingForm::on_pBStart_clicked()
{
    if(ui->pBStart->text() == "Start")
    {
        ui->pBStart->setText("Stop");
        ui->cBSensorName->setEnabled(false);
        ui->gBDragMode->setEnabled(false);
        ui->gBModel->setEnabled(false);
        ui->gBCalculateMethod->setEnabled(false);
        displayMessage("start handguiding mode.");
    }
    else
    {
        ui->pBStart->setText("Start");
        ui->cBSensorName->setEnabled(true);
        ui->gBDragMode->setEnabled(true);
        ui->gBModel->setEnabled(true);
        ui->gBCalculateMethod->setEnabled(true);
        displayMessage("exit handguiding mode.");
    }
}

void HandGuidingForm::on_rBPos_clicked()
{
    ft_sensor_util_->setFTDBData("base", "dragMode", "position");
    robot_control_->setDragMode(DRAG_MODE::POSITION);
}

void HandGuidingForm::on_rBOri_clicked()
{
    ft_sensor_util_->setFTDBData("base", "dragMode", "ori");
    robot_control_->setDragMode(DRAG_MODE::ORI);
}

void HandGuidingForm::on_rBPose_clicked()
{
    ft_sensor_util_->setFTDBData("base", "dragMode", "pose");
    robot_control_->setDragMode(DRAG_MODE::POSE);
}

void HandGuidingForm::on_rBJacobian_clicked()
{
    ft_sensor_util_->setFTDBData("base", "calculateMethod", "Jacobian");
//    robot_control_->setCalculateMethod(JACOBIAN);
}

void HandGuidingForm::on_rBIK_clicked()
{
    ft_sensor_util_->setFTDBData("base", "calculateMethod", "IK");
//    robot_control_->setCalculateMethod(IK);
}

void HandGuidingForm::on_rBVelocity_clicked()
{
    ft_sensor_util_->setFTDBData("base", "controlModel", "velocity");
//    robot_control_->setControlModel(VELOCITY);
}

void HandGuidingForm::on_rBAcceleration_clicked()
{
    ft_sensor_util_->setFTDBData("base", "controlModel", "acceleration");
//    robot_control_->setControlModel(ACCLERATION);
}

void HandGuidingForm::on_rBJointSpace_clicked()
{
    ft_sensor_util_->setFTDBData("base", "controlSpace", "jointSpace");
//    robot_control_->setControlSpace(JOINT_SPACE);
}

void HandGuidingForm::on_rBOperateSpace_clicked()
{
    ft_sensor_util_->setFTDBData("base", "controlSpace", "operateSpace");
//    robot_control_->setControlSpace(OPERATION_SPACE);
}

void HandGuidingForm::on_lEControlPeriod_textChanged(const QString &arg1)
{
    ft_sensor_util_->setFTDBData("base", "controlPeriod", arg1);
    robot_control_->setControlPeriod(arg1.toInt());
}

void HandGuidingForm::on_lEBuffSizeLimit_textChanged(const QString &arg1)
{
    ft_sensor_util_->setFTDBData("base", "bufferSizeLimit", arg1);
    robot_control_->setBufferSizeLimit(arg1.toInt());
}



/******************** RealTime Data ********************/
void HandGuidingForm::on_cBRealTime_clicked()
{
    if(ui->cBRealTime->checkState() == Qt::Checked)
    {
        ui->cB_Fx->setEnabled(true);
        ui->cB_Fy->setEnabled(true);
        ui->cB_Fz->setEnabled(true);
        ui->cB_Tx->setEnabled(true);
        ui->cB_Ty->setEnabled(true);
        ui->cB_Tz->setEnabled(true);
        ft_sensor_util_->enableDisplaySenssorData();
    }
    else
    {
        ui->cB_Fx->setEnabled(false);
        ui->cB_Fy->setEnabled(false);
        ui->cB_Fz->setEnabled(false);
        ui->cB_Tx->setEnabled(false);
        ui->cB_Ty->setEnabled(false);
        ui->cB_Tz->setEnabled(false);
        ft_sensor_util_->disableDisplaySenssorData();
    }
}

void HandGuidingForm::on_hSFilter1_valueChanged(int value)
{
    ft_sensor_util_->setFTDBData("base", "filter1", QString::number(value));
    robot_control_->setFilter1(value);
}

void HandGuidingForm::on_hSFilter2_valueChanged(int value)
{
    ft_sensor_util_->setFTDBData("base", "filter2", QString::number(value));
    robot_control_->setFilter2(value);
}

void HandGuidingForm::on_pBScreenShot_clicked()
{
    QScreen *screen=QGuiApplication::primaryScreen();
    QPixmap pixmap=screen->grabWindow(ui->paintFrame->winId());
    pixmap.save("screen.jpg","jpg");
}

void HandGuidingForm::drawRealtimeData(QFrame *frame)
{
    if(ui->cBRealTime->checkState() != Qt::Checked)
        return;
    QPainter painter(frame);
    QPen pen;
    pen.setColor(Qt::black);
    pen.setStyle(Qt::SolidLine);
    pen.setWidthF(0.5);
    painter.setPen(pen);
//    painter.setViewport(0, 0, width(), height());
//    painter.setWindow(0, 0, width(), height());
    int width = frame->width();
    int height = frame->height();
    painter.fillRect(0, 0,  width, height, Qt::white);
    painter.drawLine(QPointF(0, 0), QPointF(0, height));
    painter.drawLine(QPointF(0, height/2), QPointF(width, height/2));
    painter.drawLine(QPointF(width-1, 0), QPointF(width-1, height));
    byte flag = 0;
    if(ui->cB_Fx->checkState() == Qt::Checked)
        flag |= 0x01;
    if(ui->cB_Fy->checkState() == Qt::Checked)
        flag |= 0x02;
    if(ui->cB_Fz->checkState() == Qt::Checked)
        flag |= 0x04;
    if(ui->cB_Tx->checkState() == Qt::Checked)
        flag |= 0x08;
    if(ui->cB_Ty->checkState() == Qt::Checked)
        flag |= 0x10;
    if(ui->cB_Tz->checkState() == Qt::Checked)
        flag |= 0x20;
    double extremum[4] = {100,-100,100,-100}; //minimum and maximum values of force and torque.
    int count = ft_sensor_util_->getDisplayDataExtremum(extremum, flag);
    if(count > ft_sensor_util_-> getPlotDataLength())
    {
        //clear the extra data for the plot
//        std::vector<Wrench>::iterator itePre = ft_sensor_util_->m_sensor_data_display.begin();
//        ft_sensor_util_->m_sensor_data_display.erase(itePre, itePre + count - ft_sensor_util_-> getPlotDataLength());
    }
    double maxForce = abs(extremum[1]);
    if(maxForce < abs(extremum[0]))
        maxForce = abs(extremum[0]);

    double maxTorque = abs(extremum[3]);
    if(maxTorque < abs(extremum[2]))
        maxTorque = abs(extremum[2]);

    double forceRatio = height / 2 / maxForce * 0.95;
    double torqueRatio = height / 2 / maxTorque * 0.95;

    if(flag & 0x01)
    {
        painter.setPen(QPen(Qt::red,2,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin));
        for(int i = 0; i < count-1; i++)
            painter.drawLine(QPointF(i, height / 2 - ft_sensor_util_->m_sensor_data_display[i][0]*forceRatio),
                    QPointF(i+1, height / 2 - ft_sensor_util_->m_sensor_data_display[i+1][0]*forceRatio));   // Fx
    }

    if(flag & 0x02)
    {
        painter.setPen(QPen(Qt::green,2,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin));
        for(int i = 0; i < count-1; i++)
            painter.drawLine(QPointF(i, height / 2 - ft_sensor_util_->m_sensor_data_display[i][1]*forceRatio),
                             QPointF(i+1, height / 2 - ft_sensor_util_->m_sensor_data_display[i+1][1]*forceRatio));   // Fy

    }

    if(flag & 0x04)
    {
        painter.setPen(QPen(Qt::blue,2,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin));
        for(int i = 0; i < count-1; i++)
            painter.drawLine(QPointF(i, height / 2 - ft_sensor_util_->m_sensor_data_display[i][2]*forceRatio),
                             QPointF(i+1, height / 2 - ft_sensor_util_->m_sensor_data_display[i+1][2]*forceRatio));   // Fz
    }

    if(flag & 0x08)
    {
        painter.setPen(QPen(Qt::magenta,2,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin));
        for(int i = 0; i < count-1; i++)
            painter.drawLine(QPointF(i, height / 2 - ft_sensor_util_->m_sensor_data_display[i][3]*torqueRatio),
                             QPointF(i+1, height / 2 - ft_sensor_util_->m_sensor_data_display[i+1][3]*torqueRatio));   // Tx

    }

    if(flag & 0x10)
    {
        painter.setPen(QPen(Qt::cyan,2,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin));
        for(int i = 0; i < count-1; i++)
            painter.drawLine(QPointF(i, height / 2 - ft_sensor_util_->m_sensor_data_display[i][4]*torqueRatio),
                             QPointF(i+1, height / 2 - ft_sensor_util_->m_sensor_data_display[i+1][4]*torqueRatio));   // Ty

    }

    if(flag & 0x20)
    {
        painter.setPen(QPen(Qt::darkGray,2,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin));
        for(int i = 0; i < count-1; i++)
            painter.drawLine(QPointF(i, height / 2 - ft_sensor_util_->m_sensor_data_display[i][5]*torqueRatio),
                             QPointF(i+1, height / 2 - ft_sensor_util_->m_sensor_data_display[i+1][5]*torqueRatio));   // Tz

    }

    ui->label_Fmax->setText(QString::number(extremum[1],'f',5));
    ui->label_Tmax->setText(QString::number(extremum[3],'f',5));
    ui->label_Fmin->setText(QString::number(extremum[0],'f',5));
    ui->label_Tmin->setText(QString::number(extremum[2],'f',5));
}

bool HandGuidingForm::eventFilter(QObject *watched, QEvent *event)
{
    if(watched == ui->paintFrame && event->type() == QEvent::Paint)
    {
        drawRealtimeData(ui->paintFrame);
        return true;
    }
    else
        return QWidget::eventFilter(watched,event);
}


/******************** Admittance Control ********************/
void HandGuidingForm::on_cB_Enable_FT_Control_clicked()
{
    if(ui->cB_Enable_FT_Control->checkState() == Qt::Checked)
        robot_control_->enableAdmittanceControl();
    else
        robot_control_->disableAdmittanceControl();
}

void HandGuidingForm::on_cB_Fx_control_clicked()
{
    if(ui->cB_Fx_control->checkState() == Qt::Checked)
        ui->lE_Fx_control->setEnabled(true);
    else
        ui->lE_Fx_control->setEnabled(false);
}

void HandGuidingForm::on_cB_Fy_control_clicked()
{
    if(ui->cB_Fy_control->checkState() == Qt::Checked)
        ui->lE_Fy_control->setEnabled(true);
    else
        ui->lE_Fy_control->setEnabled(false);
}

void HandGuidingForm::on_cB_Fz_control_clicked()
{
    if(ui->cB_Fz_control->checkState() == Qt::Checked)
        ui->lE_Fz_control->setEnabled(true);
    else
        ui->lE_Fz_control->setEnabled(false);
}

void HandGuidingForm::on_cB_Tx_control_clicked()
{
    if(ui->cB_Tx_control->checkState() == Qt::Checked)
        ui->lE_Tx_control->setEnabled(true);
    else
        ui->lE_Tx_control->setEnabled(false);
}

void HandGuidingForm::on_cB_Ty_control_clicked()
{
    if(ui->cB_Ty_control->checkState() == Qt::Checked)
        ui->lE_Ty_control->setEnabled(true);
    else
        ui->lE_Ty_control->setEnabled(false);
}

void HandGuidingForm::on_cB_Tz_control_clicked()
{
    if(ui->cB_Tz_control->checkState() == Qt::Checked)
        ui->lE_Tz_control->setEnabled(true);
    else
        ui->lE_Tz_control->setEnabled(false);
}

void HandGuidingForm::on_lE_Fx_control_textChanged(const QString &arg1)
{
//    robot_control_->setAdmittanceControlFT(arg1.toDouble(), CONTROL_AXIS::FX);
}

void HandGuidingForm::on_lE_Fy_control_textChanged(const QString &arg1)
{
//    robot_control_->setAdmittanceControlFT(arg1.toDouble(), CONTROL_AXIS::FY);
}

void HandGuidingForm::on_lE_Fz_control_textChanged(const QString &arg1)
{
//    robot_control_->setAdmittanceControlFT(arg1.toDouble(), CONTROL_AXIS::FZ);
}

void HandGuidingForm::on_lE_Tx_control_textChanged(const QString &arg1)
{
//    robot_control_->setAdmittanceControlFT(arg1.toDouble(), CONTROL_AXIS::TX);
}

void HandGuidingForm::on_lE_Ty_control_textChanged(const QString &arg1)
{
//    robot_control_->setAdmittanceControlFT(arg1.toDouble(), CONTROL_AXIS::TY);
}

void HandGuidingForm::on_lE_Tz_control_textChanged(const QString &arg1)
{
//    robot_control_->setAdmittanceControlFT(arg1.toDouble(), CONTROL_AXIS::TZ);
}

void HandGuidingForm::on_lE_Force_P_Gain_textChanged(const QString &arg1)
{
    robot_control_->updateAdmittancePIDPara(arg1.toDouble(), 0);
}

void HandGuidingForm::on_lE_Force_I_Gain_textChanged(const QString &arg1)
{
    robot_control_->updateAdmittancePIDPara(arg1.toDouble(), 1);
}

void HandGuidingForm::on_lE_Force_D_Gain_textChanged(const QString &arg1)
{
    robot_control_->updateAdmittancePIDPara(arg1.toDouble(), 2);
}

void HandGuidingForm::on_lE_Torque_P_Gain_textChanged(const QString &arg1)
{
    robot_control_->updateAdmittancePIDPara(arg1.toDouble(), 3);
}

void HandGuidingForm::on_lE_Torque_I_Gain_textChanged(const QString &arg1)
{
    robot_control_->updateAdmittancePIDPara(arg1.toDouble(), 4);
}

void HandGuidingForm::on_lE_Torque_D_Gain_textChanged(const QString &arg1)
{
    robot_control_->updateAdmittancePIDPara(arg1.toDouble(), 5);
}

void HandGuidingForm::on_lE_Force_Vmax_Gain_textChanged(const QString &arg1)
{
    robot_control_->updateAdmittancePIDPara(arg1.toDouble(), 6);
}

void HandGuidingForm::on_lE_Force_Amax_Gain_textChanged(const QString &arg1)
{
    robot_control_->updateAdmittancePIDPara(arg1.toDouble(), 7);
}

void HandGuidingForm::on_lE_Torque_Vmax_Gain_textChanged(const QString &arg1)
{
    robot_control_->updateAdmittancePIDPara(arg1.toDouble(), 8);
}

void HandGuidingForm::on_lE_Torque_Amax_Gain_textChanged(const QString &arg1)
{
    robot_control_->updateAdmittancePIDPara(arg1.toDouble(), 9);
}

/******************** Setup ********************/
/******************** Setup ********************/
/******************** Setup ********************/
void HandGuidingForm::on_lESensitivityFz_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "sensitivity", 1);
}

void HandGuidingForm::on_lESensitivityFx_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "sensitivity", 2);
}

void HandGuidingForm::on_lESensitivityFy_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "sensitivity", 3);
}

void HandGuidingForm::on_lESensitivityTx_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "sensitivity", 4);
}

void HandGuidingForm::on_lESensitivityTy_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "sensitivity", 5);
}

void HandGuidingForm::on_lESensitivityTz_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "sensitivity", 6);
}

void HandGuidingForm::on_lEDampVx_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "damp", 1);
}

void HandGuidingForm::on_lEDampVy_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "damp", 2);
}

void HandGuidingForm::on_lEDampVz_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "damp", 3);
}

void HandGuidingForm::on_lEDampWx_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "damp", 4);
}

void HandGuidingForm::on_lEDampWy_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "damp", 5);
}

void HandGuidingForm::on_lEDampWz_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "damp", 6);
}

void HandGuidingForm::on_lEStiffPosX_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "stiffness", 1);
}

void HandGuidingForm::on_lEStiffPosY_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "stiffness", 2);
}

void HandGuidingForm::on_lEStiffPosZ_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "stiffness", 3);
}

void HandGuidingForm::on_lEStiffOriX_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "stiffness", 4);
}

void HandGuidingForm::on_lEStiffOriY_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "stiffness", 5);
}

void HandGuidingForm::on_lEStiffOriZ_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "stiffness", 6);
}

void HandGuidingForm::on_lEForceXThreshold_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "threshold", 1);
}

void HandGuidingForm::on_lEForceYThreshold_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "threshold", 2);
}

void HandGuidingForm::on_lEForceZThreshold_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "threshold", 3);
}

void HandGuidingForm::on_lETorqueXThreshold_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "threshold", 4);
}

void HandGuidingForm::on_lETorqueYThreshold_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "threshold", 5);
}

void HandGuidingForm::on_lETorqueZThreshold_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "threshold", 6);
}

void HandGuidingForm::on_lEForceXLimit_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "limit", 1);
}

void HandGuidingForm::on_lEForceYLimit_textChanged(const QString &arg1)
{
    updateDataBase(arg1, "parameter", "limit", 2);
}

void HandGuidingForm::on_lEForceZLimit_textChanged(const QString &arg1)
{
     updateDataBase(arg1, "parameter", "limit", 3);
}

void HandGuidingForm::on_lETorqueXLimit_textChanged(const QString &arg1)
{
     updateDataBase(arg1, "parameter", "limit", 4);
}

void HandGuidingForm::on_lETorqueYLimit_textChanged(const QString &arg1)
{
     updateDataBase(arg1, "parameter", "limit", 5);
}

void HandGuidingForm::on_lETorqueZLimit_textChanged(const QString &arg1)
{
     updateDataBase(arg1, "parameter", "limit", 6);
}

void HandGuidingForm::on_comBHandGuidingSwitchIO_currentTextChanged(const QString &arg1)
{
    robot_control_->setHandGuidingSwitchIO(arg1.toStdString());
}

void HandGuidingForm::on_lE_Robot_IP_editingFinished()
{
//    robot_control_->setRobotIP(ui->lE_Robot_IP->text().toStdString());
}
