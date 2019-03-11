#include "robotcontrol.h"

int RobotControl::s_control_period = 5;//unit s
bool RobotControl::s_start_handguiding = false;
bool RobotControl::s_thread_handguiding = true;
double RobotControl::s_accumulate_time = 0.0;
double RobotControl::s_tool_pose[CARTESIAN_FREEDOM] = {0};           //TOOL POSE relative to the robot end
double RobotControl::m_toolPosition[3] = {0};
double RobotControl::m_toolOrientation[9] = {0};

static RobotControl *s_instance = 0;


double pose1_jointangle[6] = {-15,15,75,60,80,0};
double pose2_jointangle[6] = {-36,27,95,-27,0,0};
double pose3_jointangle[6] = {-36,21,100,-6,90,0};

RobotControl *RobotControl::instance()
{
    return s_instance;
}

RobotControl::RobotControl():tcp2canMode_(false)
{
    for(int i = 0; i < aubo_robot_namespace::ARM_DOF; i++)
    {
        joint_max_acc_.jointPara[i] = MAX_ACCELERATION;
        joint_max_velc_.jointPara[i] = MAX_VELOCITY;
    }
}

RobotControl::~RobotControl()
{
    if(tcp2canMode_)
        robotServiceSend.robotServiceLeaveTcp2CanbusMode();
    //log out
    robotServiceSend.robotServiceRobotShutdown();
    robotServiceSend.robotServiceLogout();
}

bool RobotControl::initRobotService()
{
    bool flag = false;
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** login  ***/
    ret = robotServiceSend.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    ret = robotServiceReceive.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
        std::cout<<"login sucess."<<std::endl;
    else
    {
        std::cerr<<"login failed."<<std::endl;
        flag = false;
    }

    /** Initialize the real robot　**/
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    //tool dynamical parameters
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

    ret = robotServiceSend.rootServiceRobotStartup(toolDynamicsParam/**tool dynamical parameters**/,
                                                   6        /*collision class*/,
                                                   true     /*Allow read pose, true*/,
                                                   true,    /*Default true */
                                                   1000,    /*defaault 1000 */
                                                   result); /*initial result*/
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
        std::cout<<"initial sucess."<<std::endl;
    else
        std::cerr<<"initial failed."<<std::endl;

    /** Initialize move profile ***/
    ret = robotServiceSend.robotServiceInitGlobalMoveProfile();
    /** Set max ACC ***/
    ret = robotServiceSend.robotServiceSetGlobalMoveJointMaxAcc(joint_max_acc_);
    /** Set max Velocity ***/
    ret = robotServiceSend.robotServiceSetGlobalMoveJointMaxVelc(joint_max_velc_);
    /** Get the intial value of theoretical waypoint**/
    ret = robotServiceReceive.robotServiceGetCurrentWaypointInfo(theoretical_way_point_);
    flag = !(bool)ret;
    for(int i = 0; i < aubo_robot_namespace::ARM_DOF; i++)
    {
        last_send_joints_[i] = theoretical_way_point_.jointpos[i];
        //penult_send_joints_[i] = last_send_joints_[i];
    }
    //first ensure the right teach mode
    enterTcp2CANMode(false);

    double R1[9];
    Util::quaternionToOriMatrix(theoretical_way_point_.orientation, R1);
    Util::hMatrixMultiply(R1, theoretical_way_point_.cartPos.positionVector, m_toolOrientation, m_toolPosition, &initial_way_point_);

    //set the tool's parameters
    setToolProperty();
    return ret;
}

void RobotControl::startHandGuiding()
{
    QTime current_time;
    int ret, msec1 = 0, msec2 = 0, addSize, rib_buffer_size_ = 0;
    double ts;
    float R[SENSOR_DIMENSION], Q[SENSOR_DIMENSION];
    if(FTSensorDataProcess::s_controlModel == CONTROL_MODE::ACCLERATION)
    {
        memcpy(R, new float[SENSOR_DIMENSION]{0.00241, 0.0023, 0.0077, 0.0090, 0.0020, 0.00241}, sizeof(float)*SENSOR_DIMENSION);
        memcpy(Q, new float[SENSOR_DIMENSION]{1e-5,1e-5,1e-5,1e-7,1e-7,1e-7}, sizeof(float)*SENSOR_DIMENSION);
    }
    else
    {
        memcpy(R, new float[SENSOR_DIMENSION]{0.0241, 0.023, 0.077, 0.090, 0.090, 0.0241}, sizeof(float)*SENSOR_DIMENSION);
        memcpy(Q, new float[SENSOR_DIMENSION]{1e-5,1e-5,1e-5,1e-5,1e-5,1e-5}, sizeof(float)*SENSOR_DIMENSION);
    }
    float P[] = {1,1,1,1,1,1};
    float pp[6] = {0};
    float K[6] = {0};
    aubo_robot_namespace::RobotDiagnosis robot_diagnosis_info_;
    std::vector<aubo_robot_namespace::wayPoint_S> wayPointVector;
    //initial the move profile
    for(int i = 0; i < aubo_robot_namespace::ARM_DOF; i++)
    {
        last_acclerations_[i] = 0;
        current_acclerations_[i] = 0;
        current_velocities_[i] = 0.0;
        last_velocities_[i] = 0.0;
        relative_position_[i] = 0.0;
    }
    s_accumulate_time = 0.0;

    while(s_thread_handguiding)
    {
        current_time = QTime::currentTime();
        msec1 = current_time.msec();
        if(s_start_handguiding)
        {
            ret = robotServiceReceive.robotServiceGetRobotDiagnosisInfo(robot_diagnosis_info_);  //Get the current way point, but there is a delay.
            rib_buffer_size_ = robot_diagnosis_info_.macTargetPosDataSize;
            addSize = (FTSensorDataProcess::s_bufferSizeLimit - rib_buffer_size_) / aubo_robot_namespace::ARM_DOF;
            if(addSize > 0)
            {
                wayPointVector.clear();
                aubo_robot_namespace::wayPoint_S wp;
                addSize = 2;
                for(int bufferCount = 1; bufferCount <= addSize; bufferCount++)
                {
                    //use the model to get the pose increament
                    for(int i = 0; i < aubo_robot_namespace::ARM_DOF; i++)
                    {
                        //First-order lag filtering
                        average_sensor_data_[i] = FTSensorDataProcess::s_sensor_data[i] * 0.4 + average_sensor_data_[i] * 0.6;

                        if(FTSensorDataProcess::s_controlModel == CONTROL_MODE::ACCLERATION)
                        {
                            current_acclerations_[i] = FTSensorDataProcess::s_sensitivity[i] * average_sensor_data_[i] - FTSensorDataProcess::s_damp[i] * last_velocities_[i] - FTSensorDataProcess::s_stiffness[i] * relative_position_[i];   //a = k*force   Newton
                            current_velocities_[i] = last_velocities_[i] +  s_control_period * (current_acclerations_[i] + last_acclerations_[i]) / 2000.0;
                        }
                        else
                            current_velocities_[i] = FTSensorDataProcess::s_sensitivity[i] * average_sensor_data_[i];
                    }

                    for(int i = 0; i < 3; i++)
                    {
                        relative_pose_.relativePosition[i] = s_control_period * (last_velocities_[i] + current_velocities_[i]) / 2000.0;
                        relative_axis_angles_[i] = s_control_period * (last_velocities_[i+3] + current_velocities_[i+3]) / 2000.0;
                        relative_position_[i] += relative_pose_.relativePosition[i];
                        relative_position_[i+3] += relative_axis_angles_[i];

                    }

                    // std::cout<<"relative_position_"<<relative_position_[0]<<","<<relative_position_[1]<<","<<relative_position_[2];

                    if(!calculateTheoreticalWaypoint())
                    {
                        std::cout<<"calculation error!";
                        emit signal_handduiding_failed();
                    }
                    if(Util::checkForSafe(theoretical_way_point_.jointpos, last_send_joints_, aubo_robot_namespace::ARM_DOF))
                    {
                        for(int ks = 0; ks < aubo_robot_namespace::ARM_DOF; ks++)
                        {
                            wp.jointpos[ks] = theoretical_way_point_.jointpos[ks];
                            if(FTSensorDataProcess::s_filter2 != 0)
                            {
                                pp[ks] = P[ks] + Q[ks];
                                K[ks] = pp[ks] / (pp[ks] + R[ks]);
                                wp.jointpos[ks] = last_send_joints_[ks] + K[ks] * (wp.jointpos[ks] - last_send_joints_[ks]);
                                P[ks] = (1 - K[ks]) * pp[ks];
                            }
                            last_send_joints_[ks] = wp.jointpos[ks];
                        }
                        wayPointVector.push_back(wp);
//                        std::cout<<theoretical_way_point_.jointpos[0]<<","<<theoretical_way_point_.jointpos[1]<<","<<theoretical_way_point_.jointpos[2]<<","<<theoretical_way_point_.jointpos[3]<<","<<theoretical_way_point_.jointpos[4]<<","<<theoretical_way_point_.jointpos[5]<<","<<
//                                                                                          last_send_joints_[0]<<","<<last_send_joints_[1]<<","<<last_send_joints_[2]<<","<<last_send_joints_[3]<<","<<last_send_joints_[4]<<","<<last_send_joints_[5];
                        //update the last state
                        for(int i = 0; i < aubo_robot_namespace::ARM_DOF; i++)
                        {
                            last_acclerations_[i] = current_acclerations_[i];
                            last_velocities_[i] = current_velocities_[i];

                            theoretical_way_point_.jointpos[i] = last_send_joints_[i]; //update the command joint positions
                            intermediate_way_point_.jointpos[i] = theoretical_way_point_.jointpos[i];
                        }
                    }
                    else
                    {
                        std::cout<<"robot OverSpeed!";
                        emit signal_handduiding_failed();
                    }
                }
                ret = robotServiceSend.robotServiceSetRobotPosData2Canbus(wayPointVector);    //
//                for(int i = 0; i < wayPointVector.size(); i++)
//                    qDebug()<<i<<" "<<wayPointVector[i].jointpos[0]<<","<<wayPointVector[i].jointpos[1]<<","<<wayPointVector[i].jointpos[2]<<","<<wayPointVector[i].jointpos[3]<<","<<wayPointVector[i].jointpos[4]<<","<<wayPointVector[i].jointpos[5];
                if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
                {
                    std::cout<<"Set data error;"<<ret;
                    emit signal_handduiding_failed();
                }
            }
            else
            {
                //            continue;
            }
        }

        current_time =QTime::currentTime();
        msec2 = current_time.msec();
        ts = msec2 - msec1;
        if(ts < 0) ts += 1000;
        s_accumulate_time += ts;
        if(s_accumulate_time < s_control_period && s_accumulate_time >= 0)
        {
            usleep((s_control_period - s_accumulate_time) * 1000);         //guarantee a stable feed
        }
        s_accumulate_time -= s_control_period;
        if(s_accumulate_time < 0) s_accumulate_time = 0;
    }
}

bool RobotControl::calculateTheoreticalWaypoint()
{
    double relTran[9], R1[9], R2[9], relPos[3];
    aubo_robot_namespace::wayPoint_S temp_way_point;
    if(FTSensorDataProcess::s_calculateMethod == CALCULATE_METHOD::JACOBIAN)
    {
        double dq[6] = {0, 0, 0, 0, 0, 0};
        double ds[] = {relative_pose_.relativePosition[0], relative_pose_.relativePosition[1], relative_pose_.relativePosition[2],
                       relative_axis_angles_[0], relative_axis_angles_[1], relative_axis_angles_[2]};
        if(FTSensorDataProcess::s_dragMode == DRAG_MODE::POSITION)
        {
            ds[3] = 0;
            ds[4] = 0;
            ds[5] = 0;
        }
        else if(FTSensorDataProcess::s_dragMode == DRAG_MODE::ORI)
        {
            ds[0] = 0;
            ds[1] = 0;
            ds[2] = 0;
        }

        bool flag = Util::getAngleVelocity(ds, last_send_joints_, dq);
        //bool flag = Util::getAngleVelocity(ds, current_way_point_.jointpos, dq);

        if(!flag)
        {
            std::cout<<"nan"<<last_send_joints_[0]<<","<<last_send_joints_[1]<<","<<last_send_joints_[2]<<","<<last_send_joints_[3]<<","<<last_send_joints_[4]<<","<<last_send_joints_[5]<<","
                   <<ds[0]<<","<<ds[1]<<","<<ds[2]<<","<<ds[3]<<","<<ds[4]<<","<<ds[5];
            return false;
        }

//        std::cout<<"dqqq"<<dq[0]<<","<<dq[1]<<","<<dq[2]<<","<<dq[3]<<","<<dq[4]<<","<<dq[5];
        for(int i = 0; i < aubo_robot_namespace::ARM_DOF; i++)
        {
            theoretical_way_point_.jointpos[i] = last_send_joints_[i] + dq[i];
            //theoretical_way_point_.jointpos[i] = 0.7 * theoretical_way_point_.jointpos[i] + 0.2 * last_send_joints_[i] + 0.1 * penult_send_joints_[i];
            //theoretical_way_point_.jointpos[i] = current_way_point_.jointpos[i] + dq[i];
        }
    }
    else
    {
        relative_pose_.ena = true;
        intermediate_way_point_ = theoretical_way_point_;
        double m_wNorm = sqrt(relative_axis_angles_[0] * relative_axis_angles_[0] + relative_axis_angles_[1] * relative_axis_angles_[1] + relative_axis_angles_[2] * relative_axis_angles_[2]);
        if(m_wNorm == 0 || FTSensorDataProcess::s_dragMode == DRAG_MODE::POSITION)
        {
            relative_pose_.relativeOri.w = 1;
            relative_pose_.relativeOri.x = 0;
            relative_pose_.relativeOri.y = 0;
            relative_pose_.relativeOri.z = 0;
            relTran[0] = 1.0;relTran[1] = 0.0;relTran[2] = 0.0;
            relTran[3] = 0.0;relTran[4] = 1.0;relTran[5] = 0.0;
            relTran[6] = 0.0;relTran[7] = 0.0;relTran[8] = 1.0;
        }
        else
        {
            if(FTSensorDataProcess::s_dragMode == DRAG_MODE::ORI)
            {
                for(int i = 0; i < 3; i++)
                    relative_pose_.relativePosition[i] = 0;
            }

            for(int i = 0; i < 3; i++)
            {
                relative_axis_angles_[i] = relative_axis_angles_[i] / m_wNorm;  //normalize w
            }
            Util::angleAxisToTran(m_wNorm, relative_axis_angles_, relTran);
            if(Util::tranToQuaternion(relTran, ori))
            {
                relative_pose_.relativeOri.w = ori.w;
                relative_pose_.relativeOri.x = ori.x;
                relative_pose_.relativeOri.y = ori.y;
                relative_pose_.relativeOri.z = ori.z;
            }
            else
            {
                std::cout<<"Orientation computation error!";
                return false;
            }
        }

        for (int i = 0; i < 3; i++)
            relPos[i] = relative_pose_.relativePosition[i];
        Util::hMatrixMultiply(m_toolOrientation, m_toolPosition, relTran, relPos, &temp_way_point);

        Util::quaternionToOriMatrix(temp_way_point.orientation, R2);
        Util::quaternionToOriMatrix(intermediate_way_point_.orientation, R1);
        Util::hMatrixMultiply(R1, intermediate_way_point_.cartPos.positionVector, R2, temp_way_point.cartPos.positionVector, &intermediate_way_point_);

        //intermediate_way_point_ tool_end_pose_ describled in base coordinate
        //start
        aubo_robot_namespace::Ori tcpOriInEnd, tcpOri, endOri;
        tcpOri.w = intermediate_way_point_.orientation.w;
        tcpOri.x = intermediate_way_point_.orientation.x;
        tcpOri.y = intermediate_way_point_.orientation.y;
        tcpOri.z = intermediate_way_point_.orientation.z;

        tcpOriInEnd.w = userCoord.toolDesc.toolInEndOrientation.w;
        tcpOriInEnd.x = userCoord.toolDesc.toolInEndOrientation.x;
        tcpOriInEnd.y = userCoord.toolDesc.toolInEndOrientation.y;
        tcpOriInEnd.z = userCoord.toolDesc.toolInEndOrientation.z;

        robotServiceSend.toolOrientation2EndOrientation(tcpOriInEnd, tcpOri, endOri);

        intermediate_way_point_.orientation.w = endOri.w;
        intermediate_way_point_.orientation.x = endOri.x;
        intermediate_way_point_.orientation.y = endOri.y;
        intermediate_way_point_.orientation.z = endOri.z;

        Util::quaternionToOriMatrix(intermediate_way_point_.orientation, R1);
        double toolRelMove[3] = {0};

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                //incre pos: R * incTrans
                if (j==0)
                    toolRelMove[i] += R1[i*3+j] * userCoord.toolDesc.toolInEndPosition.x;
                else if (j==1)
                    toolRelMove[i] += R1[i*3+j] * userCoord.toolDesc.toolInEndPosition.y;
                else
                    toolRelMove[i] += R1[i*3+j] * userCoord.toolDesc.toolInEndPosition.z;
            }
        }
        for (int i=0;i<3;i++)
            intermediate_way_point_.cartPos.positionVector[i] -= toolRelMove[i];
        //end
        int ret = robotServiceReceive.robotServiceRobotIk(intermediate_way_point_.jointpos, intermediate_way_point_.cartPos.position, intermediate_way_point_.orientation, theoretical_way_point_); //less than 1 ms
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cout<<"robot IK error;"<<ret;
            return false;
        }
    }
    return true;
}

void RobotControl::setToolProperty()
{
    userCoord.coordType = aubo_robot_namespace::EndCoordinate;
    userCoord.toolDesc.toolInEndPosition.x = s_tool_pose[0];
    userCoord.toolDesc.toolInEndPosition.y = s_tool_pose[1];
    userCoord.toolDesc.toolInEndPosition.z = s_tool_pose[2];
    m_toolPosition[0] = userCoord.toolDesc.toolInEndPosition.x;
    m_toolPosition[1] = userCoord.toolDesc.toolInEndPosition.y;
    m_toolPosition[2] = userCoord.toolDesc.toolInEndPosition.z;

    double ori[4];
    Util::EulerAngleToQuaternion(new double[3]{s_tool_pose[3],s_tool_pose[4],s_tool_pose[5]}, ori);
    userCoord.toolDesc.toolInEndOrientation.w = ori[0];
    userCoord.toolDesc.toolInEndOrientation.x = ori[1];
    userCoord.toolDesc.toolInEndOrientation.y = ori[2];
    userCoord.toolDesc.toolInEndOrientation.z = ori[3];
    Util::quaternionToOriMatrix(userCoord.toolDesc.toolInEndOrientation, m_toolOrientation);
}

int RobotControl::enterTcp2CANMode(bool flag)
{
    int ret;
    if(flag)
    {
        // enter Tcp2Canbus Mode
        ret = robotServiceSend.robotServiceEnterTcp2CanbusMode();
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            std::cout<<"Enter TCP2CAN mode failed!";
        else
        {
            tcp2canMode_ = true;
            s_start_handguiding = true;
        }
    }
    else
    {
//        if(tcp2canMode_)
            ret = robotServiceSend.robotServiceLeaveTcp2CanbusMode();
        s_start_handguiding = false;
    }
    return ret;
}

bool RobotControl::ObtainCenterofMass()
{
    //obtain pose1,2,3
    //move to pose 1,,wait for sensor stable ,,store the joint angle and sensor data;;
    //move to pose 2,,,,,,,
//    s_tool_pose[3] = 0;
//    s_tool_pose[4] = 0;
//    s_tool_pose[5] = -M_PI/2;
//    setToolProperty();
//    float pose1_sensordata[SENSOR_DIMENSION] = {-26.6504,23.0249,1.98346,0.781332,0.296835,-2.71919};
//    float pose2_sensordata[SENSOR_DIMENSION] = {-31.6855,27.5977,1.96821,0.671184,0.170454,-2.71761};
//    float pose3_sensordata[SENSOR_DIMENSION] = {-31.0513,23.029,4.62102,0.779261,0.189995,-2.7181};

//    float pose1_jointangle[6] = {-15,15,75,60,50,0};
//    float pose2_jointangle[6] = {-36,27,95,-27,10,0};
//    float pose3_jointangle[6] = {-36,21,100,-6,90,0};

//    float FTSensorDataProcess::calibrationMessurement_[POSE_NUM][SENSOR_DIMENSION] = {0};
    double pose_jointangle[POSE_NUM][6] = {0};

    float center_mass[3] = {0};
    float p_k[3] = {0};



    for(int i = 0; i < SENSOR_DIMENSION; i++)
    {
//        FTSensorDataProcess::calibrationMessurement_[0][i] = pose1_sensordata[i];
//        FTSensorDataProcess::calibrationMessurement_[1][i] = pose2_sensordata[i];
//        FTSensorDataProcess::calibrationMessurement_[2][i] = pose3_sensordata[i];

        pose_jointangle[0][i] = pose1_jointangle[i]*M_PI/180;
        pose_jointangle[1][i] = pose2_jointangle[i]*M_PI/180;
        pose_jointangle[2][i] = pose3_jointangle[i]*M_PI/180;
    }


    RMatrix pose_f(9,6);
    RMatrix pose_md(POSE_NUM*3,1);
    RMatrix pose_fd(POSE_NUM*3,1);

    RMatrix pose_ft(6,POSE_NUM*3);
    RMatrix p_center(6,1);

    for(int i = 0; i < 3; i++)
    {
        pose_f.value[3*i][0] = 0;
        pose_f.value[3*i][1] = FTSensorDataProcess::calibrationMessurement_[i][2];
        pose_f.value[3*i][2] = -FTSensorDataProcess::calibrationMessurement_[i][1];
        pose_f.value[3*i][3] = 1;

        pose_f.value[3*i+1][0] = -FTSensorDataProcess::calibrationMessurement_[i][2];
        pose_f.value[3*i+1][1] = 0;
        pose_f.value[3*i+1][2] = FTSensorDataProcess::calibrationMessurement_[i][0];
        pose_f.value[3*i+1][4] = 1;

        pose_f.value[3*i+2][0] = FTSensorDataProcess::calibrationMessurement_[i][1];
        pose_f.value[3*i+2][1] = -FTSensorDataProcess::calibrationMessurement_[i][0];
        pose_f.value[3*i+2][2] = 0;
        pose_f.value[3*i+2][5] = 1;

        pose_md.value[3*i][0] = FTSensorDataProcess::calibrationMessurement_[i][3];
        pose_md.value[3*i+1][0] = FTSensorDataProcess::calibrationMessurement_[i][4];
        pose_md.value[3*i+2][0] = FTSensorDataProcess::calibrationMessurement_[i][5];

        pose_fd.value[3*i][0] = FTSensorDataProcess::calibrationMessurement_[i][0];
        pose_fd.value[3*i+1][0] = FTSensorDataProcess::calibrationMessurement_[i][1];
        pose_fd.value[3*i+2][0] = FTSensorDataProcess::calibrationMessurement_[i][2];
    }


    pose_ft = RMatrix::RTranspose(pose_f);

    RMatrix ftf_inv(6,6);

    RMatrix ftf = pose_ft * pose_f;

    RMatrix::RMatrixInv(ftf, ftf_inv);

    p_center = ftf_inv*pose_ft*pose_md;

    for(int i = 0; i < 3; i++)
    {
        center_mass[i] = p_center.value[i][0];
        p_k[i] = p_center.value[i+3][0];
    }


    RMatrix pose_RR(9,6);
    RMatrix pose_RR_trans(6,9);
    RMatrix l_offset(6,1);

    for(int k = 0; k < 3; k++)
    {
        double pose_jointangle1[6] = {0};
        double flangetobase[9] = {0};
        aubo_robot_namespace::wayPoint_S pose_way_point;

        for(int m = 0; m < 6; m++)
        {
            pose_jointangle1[m] = pose_jointangle[k][m];
        }
        int ret = robotServiceReceive.robotServiceRobotFk(pose_jointangle1, 6, pose_way_point);
        Util::quaternionToOriMatrix(pose_way_point.orientation, flangetobase);

        RMatrix flg_base(3,3);
        RMatrix sensor_flg(3,3);

        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                flg_base.value[i][j] = flangetobase[3*i+j];
                sensor_flg.value[i][j] = m_toolOrientation[3*i+j];
            }
        }

        RMatrix pose_sensortobase(3,3);
        pose_sensortobase = flg_base * sensor_flg;


        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                pose_RR.value[i+3*k][j] = pose_sensortobase.value[j][i];
            }
        }

        pose_RR.value[0+3*k][3] = 1;
        pose_RR.value[1+3*k][4] = 1;
        pose_RR.value[2+3*k][5] = 1;

    }

    pose_RR_trans = RMatrix::RTranspose(pose_RR);

    RMatrix RR_inv(6,6);

    RMatrix::RMatrixInv(pose_RR_trans*pose_RR, RR_inv);

    l_offset = RR_inv*pose_RR_trans*pose_fd;

    //FTSensorDataProcess::s_sensor_offset[6] = [fx0,fy0,fz0,mx0,my0,mz0];
    //center_mass[3] = [cx,cy,cz]

//    float FTSensorDataProcess::s_sensor_offset[6] = {0};
    float l_l[3] = {0};
    float tool_mass;
    float base_angle_offset[2];

    for(int i = 0; i < 3; i++)
    {
        FTSensorDataProcess::s_sensor_offset[i] = l_offset.value[i+3][0];
        l_l[i] = l_offset.value[i][0];
    }

    FTSensorDataProcess::s_sensor_offset[3] = p_k[0] - FTSensorDataProcess::s_sensor_offset[1]*center_mass[2] + FTSensorDataProcess::s_sensor_offset[2]*center_mass[1];
    FTSensorDataProcess::s_sensor_offset[4] = p_k[1] - FTSensorDataProcess::s_sensor_offset[2]*center_mass[0] + FTSensorDataProcess::s_sensor_offset[0]*center_mass[2];
    FTSensorDataProcess::s_sensor_offset[5] = p_k[2] - FTSensorDataProcess::s_sensor_offset[0]*center_mass[1] + FTSensorDataProcess::s_sensor_offset[1]*center_mass[0];

    tool_mass = sqrt(l_l[0]*l_l[0] + l_l[1]*l_l[1] + l_l[2]*l_l[2]);

    base_angle_offset[0] = asin(-l_l[1]/tool_mass);//U
    base_angle_offset[1] = atan(-l_l[0]/l_l[2]);//V

    FTSensorDataProcess::sensor_data_calibrated_ = true;

    std::cout<<"center of mass"<<center_mass[0]<<","<<center_mass[1]<<","<<center_mass[2];
    std::cout<<"tool_gravity"<<tool_mass;
    std::cout<<"sensor_offset"<<FTSensorDataProcess::s_sensor_offset[0]<<","<<FTSensorDataProcess::s_sensor_offset[1]<<","<<FTSensorDataProcess::s_sensor_offset[2]
            <<","<<FTSensorDataProcess::s_sensor_offset[3]<<","<<FTSensorDataProcess::s_sensor_offset[4]<<","<<FTSensorDataProcess::s_sensor_offset[5];
    return true;
}

int RobotControl::moveToTargetPose(int index)
{
    //move to target pose
    robotServiceSend.robotServiceInitGlobalMoveProfile();
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc, jointMaxVelc;
    for(int i = 0; i < aubo_robot_namespace::ARM_DOF; i++)
    {
        jointMaxAcc.jointPara[i] = 100.0/180.0*M_PI;
        jointMaxVelc.jointPara[0] = 50.0/180.0*M_PI;

    }
    robotServiceSend.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
    robotServiceSend.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);
    double jointAngle[6];
    if(index == 1)
        memcpy(jointAngle, pose1_jointangle, sizeof(double)*aubo_robot_namespace::ARM_DOF);
    else if(index == 2)
        memcpy(jointAngle, pose2_jointangle, sizeof(double)*aubo_robot_namespace::ARM_DOF);
    else
        memcpy(jointAngle, pose3_jointangle, sizeof(double)*aubo_robot_namespace::ARM_DOF);
    for(int i = 0; i < aubo_robot_namespace::ARM_DOF; i++)
        jointAngle[i] = jointAngle[i] / 180 * M_PI;
    return robotServiceSend.robotServiceJointMove(jointAngle, true);
}
