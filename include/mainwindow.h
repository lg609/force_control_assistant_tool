#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "FTSensorDataProcess.h"
#include "robotcontrol.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();  

private slots:
    void on_cBSensorName_currentIndexChanged(int index);

    void on_pBPos1_clicked();

    void on_pBPos2_clicked();

    void on_pBPos3_clicked();

    void on_pBCalibration_clicked();

    void updateData();

    void on_rBPos_clicked();

    void on_rBOri_clicked();

    void on_rBPose_clicked();

    void on_rBJacobian_clicked();

    void on_rBIK_clicked();

    void on_rBVelocity_clicked();

    void on_rBAcceleration_clicked();

    void on_hSFilter1_valueChanged(int value);

    void on_hSFilter2_valueChanged(int value);

    void on_lEControlPeriod_textChanged(const QString &arg1);

    void on_lEBuffSizeLimit_textChanged(const QString &arg1);

    void on_lESensitivityFz_textChanged(const QString &arg1);

    void on_lESensitivityFx_textChanged(const QString &arg1);

    void on_lESensitivityFy_textChanged(const QString &arg1);

    void on_lESensitivityTx_textChanged(const QString &arg1);

    void on_lESensitivityTy_textChanged(const QString &arg1);

    void on_lESensitivityTz_textChanged(const QString &arg1);

    void on_lEDampVx_textChanged(const QString &arg1);

    void on_lEDampVy_textChanged(const QString &arg1);

    void on_lEDampVz_textChanged(const QString &arg1);

    void on_lEDampWx_textChanged(const QString &arg1);

    void on_lEDampWy_textChanged(const QString &arg1);

    void on_lEDampWz_textChanged(const QString &arg1);

    void on_lEStiffPosX_textChanged(const QString &arg1);

    void on_lEStiffPosY_textChanged(const QString &arg1);

    void on_lEStiffPosZ_textChanged(const QString &arg1);

    void on_lEStiffOriX_textChanged(const QString &arg1);

    void on_lEStiffOriY_textChanged(const QString &arg1);

    void on_lEStiffOriZ_textChanged(const QString &arg1);

    void on_lEForceXThreshold_textChanged(const QString &arg1);

    void on_lEForceYThreshold_textChanged(const QString &arg1);

    void on_lEForceZThreshold_textChanged(const QString &arg1);

    void on_lETorqueXThreshold_textChanged(const QString &arg1);

    void on_lETorqueYThreshold_textChanged(const QString &arg1);

    void on_lETorqueZThreshold_textChanged(const QString &arg1);

    void on_lEForceXLimit_textChanged(const QString &arg1);

    void on_lEForceYLimit_textChanged(const QString &arg1);

    void on_lEForceZLimit_textChanged(const QString &arg1);

    void on_lETorqueXLimit_textChanged(const QString &arg1);

    void on_lETorqueYLimit_textChanged(const QString &arg1);

    void on_lETorqueZLimit_textChanged(const QString &arg1);

    void on_lEPosX_textChanged(const QString &arg1);

    void on_lEPosY_textChanged(const QString &arg1);

    void on_lEPosZ_textChanged(const QString &arg1);

    void on_lEPosRX_textChanged(const QString &arg1);

    void on_lEPosRY_textChanged(const QString &arg1);

    void on_lEPosRZ_textChanged(const QString &arg1);

    void on_pBStart_clicked();

    void on_pBRobot_clicked();

private:
    void UIInitial();
    void deviceInitial();
    void updateUI();
    void updateDataBase(QString arg1, QString table, QString name, int index);
    void handGuiding();
    Ui::MainWindow *ui;
    FTSensorDataProcess *ft_sensor_data_process_;
    RobotControl *robot_control_;
    std::thread* hand_guiding_;
    QTimer timer_;
    std::map<std::string, int> paraType_;

private slots:
    void slot_handduiding_failed();
//    void on_lineEdit_textEdited(const QString &arg1);
    void on_lEOutPut_textEdited(const QString &arg1);
    void on_pushButton_3_clicked();

};

#endif // MAINWINDOW_H
