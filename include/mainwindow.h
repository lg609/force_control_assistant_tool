#ifndef HandGuidingForm_H
#define HandGuidingForm_H

//#include <QWidget>
#include <QPainter>
#include <QPen>
#include <QTimer>
#include <QScreen>
#include <QPixmap>
#include <QMainWindow>
#include <QFrame>

#include "robot_control.h"

namespace Ui {
class HandGuidingForm;
}

class HandGuidingForm : public QWidget
{
    Q_OBJECT

public:
    explicit HandGuidingForm(QWidget *parent = 0);
    //!
    ~HandGuidingForm();

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

    void on_hSFilter1_valueChanged(int value);

    void on_hSFilter2_valueChanged(int value);

    void on_lEControlPeriod_textChanged(const QString &arg1);

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

protected:
    bool eventFilter(QObject *watched, QEvent *event);  //draw dynamic plot
    void drawRealtimeData(QFrame *frame);
    void getCalibrationPose(int index);

private:
    //!
    void initialUI();
    //!
    void initialPara();
    //!
    void updateUI();
    //!
    void updateDataBase(QString arg1, QString table, QString name, int index);
    //!
    void displayMessage(const QString str, int timeout = 0);

private:
    Ui::HandGuidingForm *ui;
    RobotControl        *robot_control_;

    QTimer timer_;      //update data flow
    bool  cBSensorName_add_finished_;
    const int update_period_ = 200;  //ms: the update period of the timer

private slots:
    void slot_handduiding_failed(QString str);
    void slot_sensor_overrange(QString str);

    void on_lE_pos_wcr_textChanged(const QString &arg1);
    void on_lE_pos_wth_textChanged(const QString &arg1);
    void on_lE_pos_lambda_textChanged(const QString &arg1);
    void on_rBenable_constraints_clicked();
    void on_rBJointSpace_clicked();
    void on_rBOperateSpace_clicked();
    void on_cBRealTime_clicked();
    void on_cB_Enable_FT_Control_clicked();
    void on_cB_Fx_control_clicked();
    void on_cB_Fy_control_clicked();
    void on_cB_Fz_control_clicked();
    void on_cB_Tx_control_clicked();
    void on_cB_Ty_control_clicked();
    void on_cB_Tz_control_clicked();
    void on_lE_Fx_control_textChanged(const QString &arg1);
    void on_lE_Fy_control_textChanged(const QString &arg1);
    void on_lE_Fz_control_textChanged(const QString &arg1);
    void on_lE_Tx_control_textChanged(const QString &arg1);
    void on_lE_Ty_control_textChanged(const QString &arg1);
    void on_lE_Tz_control_textChanged(const QString &arg1);
    void on_lE_Force_P_Gain_textChanged(const QString &arg1);
    void on_lE_Force_I_Gain_textChanged(const QString &arg1);
    void on_lE_Force_D_Gain_textChanged(const QString &arg1);
    void on_lE_Force_Vmax_Gain_textChanged(const QString &arg1);
    void on_lE_Force_Amax_Gain_textChanged(const QString &arg1);
    void on_lE_Torque_P_Gain_textChanged(const QString &arg1);
    void on_lE_Torque_I_Gain_textChanged(const QString &arg1);
    void on_lE_Torque_D_Gain_textChanged(const QString &arg1);
    void on_lE_Torque_Vmax_Gain_textChanged(const QString &arg1);
    void on_lE_Torque_Amax_Gain_textChanged(const QString &arg1);
    void on_rBSpecifyTool_clicked();
    void on_rBSelectTool_clicked();
    void on_pBScreenShot_clicked();
    void on_lE_Port_editingFinished();
    void on_lE_Robot_IP_editingFinished();
    void on_lEPosX_2_textChanged(const QString &arg1);
    void on_lEPosY_2_textChanged(const QString &arg1);
    void on_lEPosZ_2_textChanged(const QString &arg1);
    void on_lEPosRX_2_textChanged(const QString &arg1);
    void on_lEPosRY_2_textChanged(const QString &arg1);
    void on_lEPosRZ_2_textChanged(const QString &arg1);
    void on_lE_ori_wcr_textChanged(const QString &arg1);
    void on_lE_ori_wth_textChanged(const QString &arg1);
    void on_lE_ori_lambda_textChanged(const QString &arg1);
    void on_rBSerial_clicked();
    void on_rBParallel_clicked();
    void on_lEOverEstimateDis_textChanged(const QString &arg1);
    void on_pushButton_clicked();
};

#endif // HandGuidingForm_H
