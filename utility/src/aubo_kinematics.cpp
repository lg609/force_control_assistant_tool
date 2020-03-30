#include "aubo_kinematics.h"

double Kinematics::s_w_th[2] = {0.02, 0.03};
double Kinematics::s_w_cr[2] = {0.005, 0.005};
double Kinematics::s_w_lambda[2] = {10, 10};

Kinematics::Kinematics(RobotModel& rm):robot_kin_(rm),robot_model(&rm),ik_ana_(rm),fk_ana_(rm)
{
}

void Kinematics::setToolProperty(const Frame& frame, const RigidBodyInertia& I)
{
//    robot_model->setEndEffector(frame, I);
}

void Kinematics::getExternalForceOnTool(const Wrench& sensor_data, const JntArray& cur_joints, Wrench &force_of_end_)
{
    Wrench raw_sensor_data_ = sensor_data;
    //subtract gravity component(forcr and torque) from sensor data ;
    //center_mass tool_mass
    Vector g_base(0,0,-1);
    Frame fr;
    robot_kin_.fk_ana_.JntToCart(cur_joints, fr);
    Rotation sensortobase = fr.M;
    Vector g_component = sensortobase.Inverse()*g_base;
    for(int i = 0; i < 3; i++)
        gravity_component[i] = g_component[i]*robot_model->getRobotTool().getInertia().getMass();       //GX GY GZ MGx MGy MGz IN SENSOR
    gravity_component[3] = gravity_component[2]*robot_model->getRobotTool().getInertia().getCOG()[1] - gravity_component[1]*robot_model->getRobotTool().getInertia().getCOG()[2];
    gravity_component[4] = gravity_component[0]*robot_model->getRobotTool().getInertia().getCOG()[2] - gravity_component[2]*robot_model->getRobotTool().getInertia().getCOG()[0];
    gravity_component[5] = gravity_component[1]*robot_model->getRobotTool().getInertia().getCOG()[0] - gravity_component[0]*robot_model->getRobotTool().getInertia().getCOG()[1];


    //translate to end of tool
    raw_sensor_data_ -= gravity_component;
    for(int i = 0; i < 3; i++)
        force_of_end_[i] = raw_sensor_data_[i];
    force_of_end_[3] = raw_sensor_data_[2]*robot_model->getRobotTool().getFrameToTip().p[1] - raw_sensor_data_[1]*robot_model->getRobotTool().getFrameToTip().p[2] + raw_sensor_data_[3];
    force_of_end_[4] = raw_sensor_data_[0]*robot_model->getRobotTool().getFrameToTip().p[2] - raw_sensor_data_[2]*robot_model->getRobotTool().getFrameToTip().p[0] + raw_sensor_data_[4];
    force_of_end_[5] = raw_sensor_data_[1]*robot_model->getRobotTool().getFrameToTip().p[0] - raw_sensor_data_[0]*robot_model->getRobotTool().getFrameToTip().p[1] + raw_sensor_data_[5];
}


bool Kinematics::calibrateToolAndSensor(const JntArray poses[CALIBRATE_POSE_NUM], const Wrench mesurement[CALIBRATE_POSE_NUM], double result[])
{
    double p_k[3] = {0};
    double center_mass[3] = {0};
    double l_l[3] = {0};
    double base_angle_offset[2];
//    RigidBodyInertia I = robot_model->getRobotTool().getInertia();

    Jacobian F = Jacobian(CALIBRATE_POSE_NUM*3,6);
    Jacobian tmp;
    JntArray M = JntArray(CALIBRATE_POSE_NUM*3);
    JntArray f = JntArray(CALIBRATE_POSE_NUM*3);
    JntArray p = JntArray(6);  //[X Y Z K1 K2 K3]

    for(int i = 0; i < CALIBRATE_POSE_NUM; i++)
    {
        F(3*i,1) = mesurement[i][2];
        F(3*i,2) = -mesurement[i][1];
        F(3*i,3) = 1;

        F(3*i+1,0) = -mesurement[i][2];
        F(3*i+1,2) = mesurement[i][0];
        F(3*i+1,4) = 1;

        F(3*i+2,0) = mesurement[i][1];
        F(3*i+2,1) = -mesurement[i][0];
        F(3*i+2,5) = 1;

        M(3*i) = mesurement[i][3];
        M(3*i+1) = mesurement[i][4];
        M(3*i+2) = mesurement[i][5];

        f(3*i) = mesurement[i][0];
        f(3*i+1) = mesurement[i][1];
        f(3*i+2) = mesurement[i][2];
    }

    M.print();
    F.print();
    tmp = F.inverse();
    tmp.print();
    p.data = tmp.data * M.data;
    p.print();
//    std::cout<<mesurement[0]<<std::endl;
    JntArray jt = poses[0];
    jt.print();

    for(int i = 0; i < CALIBRATE_POSE_NUM; i++)
    {
        center_mass[i] = p(i);
        p_k[i] = p(i+3);
    }

    Jacobian poseR(CALIBRATE_POSE_NUM*3,6);
    JntArray l_offset(6);
    for(int k = 0; k < CALIBRATE_POSE_NUM; k++)
    {
        Frame fr;
        robot_kin_.fk_ana_.JntToCart(poses[k], fr);
        Rotation R = fr.M;

        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                poseR(i+3*k,j) = R(j,i);//transpose
            }
        }
        poseR(0+3*k,3) = 1;
        poseR(1+3*k,4) = 1;
        poseR(2+3*k,5) = 1;
    }
    tmp = poseR.inverse();
    l_offset.data = tmp.data * f.data;

    for(int i = 0; i < 3; i++)
    {
        l_l[i] = l_offset(i);
        result[i] = l_offset(i+3);
    }

    result[3] = p_k[0] - result[1]*center_mass[2] + result[2]*center_mass[1];
    result[4] = p_k[1] - result[2]*center_mass[0] + result[0]*center_mass[2];
    result[5] = p_k[2] - result[0]*center_mass[1] + result[1]*center_mass[0];

    double tool_mass = sqrt(l_l[0]*l_l[0] + l_l[1]*l_l[1] + l_l[2]*l_l[2]);//the gravity of tool in base;

    base_angle_offset[0] = asin(-l_l[1] / tool_mass);       //U
    base_angle_offset[1] = atan(-l_l[0] / l_l[2]);          //V

    memcpy(&result[6], center_mass, sizeof(double)*3);
    result[9] = tool_mass;
    return true;
}


void Kinematics::obtainConstraintForce(const JntArray&q0, Wrench& wrench_constraint)
{
    Jacobian J_inv(6,3);
    double wq_0[2] = {0}, wq_next[2] = {0}, kw[2] = {0};

    //iterate for translation and rotation
    for(int k = 0; k < 2; k++)
    {
        wq_0[k] = getPerformanceIndex(q0, J_inv, (JACOBIAN_TYPE)k);
        if(wq_0[k] > s_w_th[k])
            kw[k] = 0;
        else
        {
            kw[k] = s_w_lambda[k] * (1.0 / (wq_0[k] - s_w_cr[k]) - 1.0 / (s_w_th[k] - s_w_cr[k]));
            for(int i = 0; i < 3; i++)
            {
                double v_unit = 1, dt = CONTROL_PERIOD, dw = 0;
                double direc[] = {1.0,-1.0}, delta_w[2];
                for(int j = 0; j < 2; j++)
                {
                    JntArray q_next(6);
                    RVector x_dot(3);
                    x_dot.value[i]= v_unit*direc[j];
                    Add(q0, JntArray(J_inv.data * x_dot * dt),q_next);

                    wq_next[k] = getPerformanceIndex(q_next, J_inv, (JACOBIAN_TYPE)k);
                    if(wq_next[k] > wq_0[k])
                        delta_w[j] =  wq_next[k] - wq_0[k];
                    else
                        delta_w[j]=0;
                }
                int index = (delta_w[0] > delta_w[1])?0:1;

                if(k == 0)
                    wrench_constraint.force(i) = delta_w[index] * direc[index] * kw[k];
                else
                    wrench_constraint.torque(i) = delta_w[index] * direc[index] * kw[k];
            }
        }
    }
}

//complete with different performance index
double Kinematics::getPerformanceIndex(const JntArray& q_in, Jacobian& J_inv, JACOBIAN_TYPE type)
{
    RMatrix U(3,3), V(3,3), S(3,3);  //to do
    Jacobian Jn(6, robot_model->getChain().getNrOfJoints());
    robot_kin_.getDifferentialJacobian(Jn, q_in, true);
    Jacobian Jt = Jn.getTranslationPart();
    Jacobian Jr = Jn.getRotationPart();

    if(type == TRAN_J)
    {
        J_inv = Jt.inverse();
        Jacobian Jt_norm = (Jt.data*(RMatrix::eye(Jt.columns()) - Jr.inverse().data*Jr.data)*Jt.data.transpose());
        Jt_norm.data.svdSim(U, S, V, 100);
    }
    else
    {
        J_inv = Jr.inverse();
        Jacobian Jr_norm = Jr.data*(RMatrix::eye(Jr.columns()) - Jt.inverse().data*Jt.data)*Jr.data.transpose();
        Jr_norm.data.svdSim(U, S, V, 100);
    }

    return S.value[2][2];
}

bool Kinematics::getAngleVelocity(const Twist& twist, const JntArray& q_in, JntArray& dq)
{
    bool flag;
    Jacobian Jn(6, ARM_DOF);
    Frame toolToBase = robot_kin_.getDifferentialJacobian(Jn, q_in, true);
    //    Jn.changeRefFrame(toolToBase);          //ds_b = A * ds_t; J*q_dot = A * ds_t;

    MultiplyJacobian(Jn.inverse(), twist, dq);

    //    T0_7 = T6*T_tool;

    //    Rn = RMatrix::subRMatrix(T0_7,0,2,0,2);
    //    on = RMatrix::subRVector(T0_7,0,2,3,"Column");
    //    sk =  RVector::Skew(on);
    //    Rn0 = RMatrix::RTranspose(Rn);

    //    C = sk*Rn;
    //    RMatrix::catRMatrix(A,0,2,0,2,Rn);
    //    RMatrix::catRMatrix(A,0,2,3,5,C);
    //    RMatrix::catRMatrix(A,3,5,0,2,m_zeros);
    //    RMatrix::catRMatrix(A,3,5,3,5,Rn);

    //    flag = RMatrix::RMatrixInv(J, T);
    //    RMatrix J_A = T * A;
    //    dq = J_A * dDis; //the increment of joint angle

    if(flag/*!isnan(dDelta.value[0]) && !isnan(dDelta.value[1]) && !isnan(dDelta.value[2]) && !isnan(dDelta.value[3]) && !isnan(dDelta.value[4]) && !isnan(dDelta.value[5])*/)
    {
        //        std::cout<<"dq"<<dq[0]<<","<<dq[1]<<","<<dq[2]<<","<<dq[3]<<","<<dq[4]<<","<<dq[5]<<std::endl;
        return true;
    }
    else
    {
        dq.setToZero();
        return false;
    }
}

bool Kinematics::checkForSafe(const JntArray& lhs, const JntArray& rhs)
{
    assert(lhs.rows() == rhs.rows());
    for(unsigned int i =0; i < lhs.rows(); i++)
    {
        if(fabs(lhs.data.value[i] -  rhs.data.value[i]) > SAFETY_COEFFICIENT)
            return false;
    }
    return true;
}

void Kinematics::jointSpaceControl(const JntArray& q_in, const Wrench& forceofEnd, JntArray& tau)
{
    Jacobian Jn(6, ARM_DOF);
    robot_kin_.getGeometryJacobian(Jn, q_in, true);
//    tau = Jn.transport()*forceofEnd;
}

bool Kinematics::calculateTheoreticalWaypoint(const int controlType[], Twist& ds, const JntArray& q_in, JntArray& q_out)
{
    if(controlType[0] == CALCULATE_METHOD::JACOBIAN)
    {
        JntArray dq(6);
        if(controlType[1] == DRAG_MODE::POSITION)
        {
            ds.rot.data[0] = 0;
            ds.rot.data[1] = 0;
            ds.rot.data[2] = 0;
        }
        else if(controlType[1] == DRAG_MODE::ORI)
        {
            ds.vel.data[0] = 0;
            ds.vel.data[1] = 0;
            ds.vel.data[2] = 0;
        }
        bool flag = getAngleVelocity(ds, q_in, dq);
        if(!flag)
        {
//            std::cout<<"nan"<<q_in[0]<<","<<q_in[1]<<","<<q_in[2]<<","<<q_in[3]<<","<<q_in[4]<<","<<q_in[5]<<","
//                    <<ds[0]<<","<<ds[1]<<","<<ds[2]<<","<<ds[3]<<","<<ds[4]<<","<<ds[5];
            return false;
        }
        Add(q_in, dq, q_out);
    }
    else
    {
        double angle_norm = sqrt(ds.rot[0] * ds.rot[0] + ds.rot[1] * ds.rot[1] + ds.rot[2] * ds.rot[2]);
        Rotation relTran;
        if(angle_norm != 0 && controlType[1] > DRAG_MODE::POSITION)
        {
            if(controlType[1] == DRAG_MODE::ORI)
                 ds.vel.setToZero();

            ds.rot = ds.rot / angle_norm;  //normalize w
            relTran = Rotation(ds.rot, angle_norm);
        }

        Frame curPose;
        fk_ana_.robotFK(q_in, curPose);
        Frame updatePose = curPose * Frame(relTran, ds.vel);
        int ret = ik_ana_.CartToJnt(q_in, updatePose, q_out);
        if(ret != ik_ana_.E_NOERROR)
        {
            std::cout<<"robot IK error;"<<ret;
            return false;
        }
    }
    return true;
}
