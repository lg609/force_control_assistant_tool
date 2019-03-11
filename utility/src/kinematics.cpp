#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::Matrix4d;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::Vector3d;

#include "../include/rmatrix.h"
#include "../include/rvector.h"
#include "../include/robotcontrol.h"


#define i5_PARAMS
#define MIN -175    //随机数产生的范围
#define MAX 175
#define MAXIUM_ITERATOR 10


namespace
{
const double ZERO_THRESH = 1e-5;
const double PI = M_PI;
#ifdef i5_PARAMS
const double d1 =  0.122;
const double d2 =  0.1215;
const double d5 =  0.1025;
const double d6 =  0.094;
const double a2 =  0.408;
const double a3 =  0.376;
#endif

int SIGN(double x)
{
    return (x > 0) - (x < 0);
}
}

// input radian
void forward(MatrixXd& T, const VectorXd q)
{
    double  q1 = q(0), q2 = q(1), q3 = q(2), q4 = q(3), q5 = q(4), q6 = q(5);
    double  C1 = cos(q1), C2 = cos(q2), C4 = cos(q4), C5 = cos(q5), C6 = cos(q6);
    double  C23 = cos(q2 - q3), C234 = cos(q2 - q3 + q4), C2345 = cos(q2 - q3 + q4 - q5), C2345p = cos(q2 - q3 + q4 + q5);
    double  S1 = sin(q1), S2 = sin(q2), S4 = sin(q4), S5 = sin(q5), S6 = sin(q6);
    double  S23 = sin(q2 - q3), S234 = sin(q2 - q3 + q4);

    T(0,0) = -C6 * S1 * S5 + C1 * (C234 * C5 * C6 - S234 * S6);
    T(0,1) = S1 * S5 * S6 - C1 * (C4 * C6 * S23 + C23 * C6 * S4 + C234 * C5 * S6);
    T(0,2) = C5 * S1 + C1 * C234 * S5;
    T(0,3) = (d2 + C5 * d6) * S1 - C1 * (a2 * S2 + (a3 + C4 * d5) * S23 + C23 * d5 * S4 - C234 * d6 * S5);

    T(1,0) = C234 * C5 * C6 * S1 + C1 * C6 * S5 - S1 * S234 * S6;
    T(1,1) = -C6 * S1 * S234 - (C234 * C5 * S1 + C1 * S5) * S6;
    T(1,2) = -C1 * C5 + C234 * S1 * S5;
    T(1,3) = -C1 * (d2 + C5 * d6) - S1 * (a2 * S2 + (a3 + C4 * d5) * S23 + C23 * d5 * S4 - C234 * d6 * S5);

    T(2,0) = C5 * C6 * S234 + C234 * S6;
    T(2,1) = C234 * C6 - C5 * S234 * S6;
    T(2,2) = S234 * S5;
    T(2,3) = d1 + a2 * C2 + a3 * C23 + d5 * C234 + d6 * C2345/2 - d6 * C2345p / 2;
    T.row(3) << 0, 0, 0, 1;
}

// output radian//MatrixXd q_sols(6,8);
int inverse(MatrixXd& q_sols, const MatrixXd T, bool& singularity)
{
    int num_sols = 0;
    double nx = T(0,0); double ox = T(0,1); double ax = T(0,2); double px = T(0,3);
    double ny = T(1,0); double oy = T(1,1); double ay = T(1,2); double py = T(1,3);
    double nz = T(2,0); double oz = T(2,1); double az = T(2,2); double pz = T(2,3);

    //////////////////////// shoulder rotate joint (q1) //////////////////////////////
    VectorXd q1(2);

    double A1 = d6 * ay - py;
    double B1 = d6 * ax - px;
    double R1 = A1 * A1 + B1 * B1;
    if((fabs(A1) < ZERO_THRESH) && (fabs(B1) < ZERO_THRESH))
        return num_sols;
    else if(R1 < d2*d2)
        return num_sols;
    else
    {
        q1(0) =  atan2(A1, B1) -  atan2(d2, sqrt(R1- d2 * d2));
        q1(1) =  atan2(A1, B1) -  atan2(d2, -sqrt(R1 - d2 * d2));
        for(int i = 0; i < 2; i++)
        {
            while(q1(i) > M_PI)
                q1(i) -= 2 * M_PI;
            while(q1(i) < -M_PI)
                q1(i) += 2 * M_PI;
        }
    }

    ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
    MatrixXd q5(2,2);

    for(int i = 0; i < 2; i++)
    {

        double C1 = cos(q1(i)), S1 = sin(q1(i));
        double B5 = -ay * C1 + ax * S1;
        double R5 = (-ny * C1 + nx * S1) * (-ny * C1 + nx * S1) + (-oy * C1 + ox * S1) * (-oy * C1 + ox * S1);

        if(fabs(B5) < ZERO_THRESH && fabs(R5) < ZERO_THRESH)
        {
            singularity = true;
            continue;
        }
        else
        {
            q5(i,0) = atan2(sqrt(R5), B5);
            q5(i,1) = atan2(-sqrt(R5), B5);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
    double q6;
    VectorXd q3(2), q2(2), q4(2);

    for(int i = 0; i < 2; i++)
    {
        for(int j = 0; j < 2; j++)
        {
            // wrist 3 joint (q6) //
            double C1 = cos(q1(i)), S1 = sin(q1(i));
            double S5 = sin(q5(i,j));

            double A6 = (-oy * C1 + ox * S1);
            double B6 = (ny * C1 - nx * S1);

            if((fabs(A6) < ZERO_THRESH) && (fabs(B6) < ZERO_THRESH))
            {
                singularity = true;
                break;
            }
            //            //                  return num_sols;
            //            else if(fabs(S5) < ZERO_THRESH)
            //                q6= atan2(A6*S5,B6*S5);
            //                  return num_sols;
            else
                q6 = atan2(A6 * S5, B6 * S5);

            /////// joints (q3,q2,q4) //////
            double C6 = cos(q6);
            double S6 = sin(q6);

            double pp1 = C1 * (ax * d6 - px + d5 * ox * C6 + d5 * nx * S6) + S1 * (ay * d6 - py + d5 * oy * C6 + d5 * ny * S6);
            double pp2 = -d1 - az * d6 + pz - d5 * oz * C6 - d5 * nz * S6;
            double B3 = (pp1 * pp1 + pp2 * pp2 - a2 * a2 - a3 * a3) / (2 * a2 * a3);


            if((fabs(B3)-1) > ZERO_THRESH)
            {
                singularity = true;
                continue;
            }
            //                  return num_sols;
            else
            {
                double Sin3 = sqrt(fabs(1 - B3 * B3));
                q3(0) = atan2(Sin3, B3);
                q3(1) = atan2(-Sin3, B3);
            }

            for(int k = 0; k < 2; k++)
            {

                double C3 = cos(q3(k)), S3 = sin(q3(k));
                double A2 = pp1 * (a2 + a3 * C3) + pp2 * (a3 * S3);
                double B2 = pp2 * (a2 + a3 * C3) - pp1 * (a3 * S3);
                if((fabs(A2) < ZERO_THRESH) && (fabs(B2) < ZERO_THRESH))
                {
                    singularity = true;
                    continue;
                }
                else
                    q2(k) = atan2(A2, B2);

                double C2 = cos(q2(k)), S2 = sin(q2(k));

                double A4 = -C1 * (ox * C6 + nx * S6) - S1 * (oy * C6 + ny * S6);
                double B4 = oz * C6 + nz * S6;
                double A41 = pp1 - a2 * S2;
                double B41 = pp2 - a2 * C2;
                q4(k) = atan2(A4, B4) - atan2(A41, B41);
                while(q4(k) > M_PI)
                    q4(k) -= 2 * M_PI;
                while(q4(k) < -M_PI)
                    q4(k) += 2 * M_PI;

                q_sols(0,num_sols) = q1(i);    q_sols(1,num_sols) = q2(k);
                q_sols(2,num_sols) = q3(k);    q_sols(3,num_sols) = q4(k);
                q_sols(4,num_sols) = q5(i,j);  q_sols(5,num_sols) = q6;
                num_sols++;
            }
        }
    }
    return num_sols;
}

void transfer(RMatrix& T, double alpha, double a, double d, double theta)
{
    //modified DH
    T.value[0][0] = cos(theta);
    T.value[0][1] = -sin(theta);
    T.value[0][2] = 0;
    T.value[0][3] = a;

    T.value[1][0] = sin(theta)*cos(alpha);
    T.value[1][1] = cos(theta)*cos(alpha);
    T.value[1][2] = -sin(alpha);
    T.value[1][3] = -sin(alpha)*d;

    T.value[2][0] = sin(theta)*sin(alpha);
    T.value[2][1] = cos(theta)*sin(alpha);
    T.value[2][2] = cos(alpha);
    T.value[2][3] = cos(alpha)*d;

    T.value[3][0] = 0;
    T.value[3][1] = 0;
    T.value[3][2] = 0;
    T.value[3][3] = 1;
    //            T <<    cos(theta),            -sin(theta),           0,           a,
    //            sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d,
    //            sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha),  cos(alpha)*d,
    //            0,                     0,                     0,           1;
}



VectorXd Vex(MatrixXd A)
{
    VectorXd S(3);
    S(0) = 0.5 * (A(2,1) - A(1,2));
    S(1) = 0.5 * (A(0,2) - A(2,0));
    S(2) = 0.5 * (A(1,0) - A(0,1));
    return S;
}

void GetJacobian(RMatrix& J, RMatrix& A, RVector q)
{
    RMatrix JJ2(3,6);
    RVector z1(3),z2(3),z3(3),z4(3),z5(3),z6(3),z7(3),o1(3),o2(3),o3(3),o4(3),o5(3),o6(3),o7(3),J1(3),J2(3),J3(3),J4(3),J5(3),J6(3),J7(3);
    RMatrix T01(4,4),T12(4,4),T23(4,4),T34(4,4),T45(4,4),T56(4,4), T67(4,4);
    RMatrix T1(4,4),T2(4,4),T3(4,4),T4(4,4),T5(4,4),T6(4,4),T7(4,4);


    transfer(T01, 0, 0, d1, q.value[0] + M_PI);
    transfer(T12, -M_PI / 2, 0, d2, q.value[1] - M_PI / 2);
    transfer(T23, M_PI, a2, 0, q.value[2]);
    transfer(T34, M_PI, a3, 0, q.value[3] - M_PI / 2);
    transfer(T45, -M_PI / 2, 0, d5, q.value[4]);
    transfer(T56, M_PI / 2, 0, d6, q.value[5]);


    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            T67.value[i][j] = RobotControl::m_toolOrientation[3*i+j];
        }
        T67.value[i][3] = RobotControl::m_toolPosition[i];
    }
    T67.value[3][3] = 1;

    //    R = RMatrix.RotZ(M_PI/4);
    //    T67 << 0, -1, 0, 0,
    //           1, 0, 0, 0,
    //           0, 0, 1, 0.07,
    //           0, 0, 0, 1;

    T1 = T01;
    T2 = T1 * T12;
    T3 = T2 * T23;
    T4 = T3 * T34;
    T5 = T4 * T45;
    T6 = T5 * T56;
    T7 = T6 * T67;

    z1 = RMatrix::subRVector(T1,0,2,2,"Column");
    z2 = RMatrix::subRVector(T2,0,2,2,"Column");
    z3 = RMatrix::subRVector(T3,0,2,2,"Column");
    z4 = RMatrix::subRVector(T4,0,2,2,"Column");
    z5 = RMatrix::subRVector(T5,0,2,2,"Column");
    z6 = RMatrix::subRVector(T6,0,2,2,"Column");

    o1 = RMatrix::subRVector(T1,0,2,3,"Column");
    o2 = RMatrix::subRVector(T2,0,2,3,"Column");
    o3 = RMatrix::subRVector(T3,0,2,3,"Column");
    o4 = RMatrix::subRVector(T4,0,2,3,"Column");
    o5 = RMatrix::subRVector(T5,0,2,3,"Column");
    o6 = RMatrix::subRVector(T6,0,2,3,"Column");
    o7 = RMatrix::subRVector(T6,0,2,3,"Column");



    J1 = RVector::cross3(z1, o7 - o1);
    J2 = RVector::cross3(z2, o7 - o2);
    J3 = RVector::cross3(z3, o7 - o3);
    J4 = RVector::cross3(z4, o7 - o4);
    J5 = RVector::cross3(z5, o7 - o5);
    J6 = RVector::cross3(z6, o7 - o6);
    //    J1 = z1.cross(o7 - o1);
    //    J2 = z2.cross(o7 - o2);
    //    J3 = z3.cross(o7 - o3);
    //    J4 = z4.cross(o7 - o4);
    //    J5 = z5.cross(o7 - o5);
    //    J6 = z6.cross(o7 - o6);

    //MatrixXd C(A.rows(), A.cols()+B.cols());
    //C << A, B;
    RMatrix::catRMatrix(J,0,2,0,J1);
    RMatrix::catRMatrix(J,0,2,1,J2);
    RMatrix::catRMatrix(J,0,2,2,J3);
    RMatrix::catRMatrix(J,0,2,3,J4);
    RMatrix::catRMatrix(J,0,2,4,J5);
    RMatrix::catRMatrix(J,0,2,5,J6);

    RMatrix::catRMatrix(JJ2,0,2,0,z1);
    RMatrix::catRMatrix(JJ2,0,2,1,z2);
    RMatrix::catRMatrix(JJ2,0,2,2,z3);
    RMatrix::catRMatrix(JJ2,0,2,3,z4);
    RMatrix::catRMatrix(JJ2,0,2,4,z5);
    RMatrix::catRMatrix(JJ2,0,2,5,z6);

    RMatrix::catRMatrix(J,3,5,0,5,JJ2);

    RMatrix RT(3,3), R(3,3),S(3,3), B(3,3);
    R = RMatrix::subRMatrix(T7,0,2,0,2);
    RT = RMatrix::RTranspose(R);
    S =  RVector::Skew(o7);

    RMatrix C = B-RT*S;
    RMatrix::catRMatrix(A,0,2,0,2,RT);
    RMatrix::catRMatrix(A,0,2,3,5,C);
    RMatrix::catRMatrix(A,3,5,0,2,B);
    RMatrix::catRMatrix(A,3,5,3,5,RT);


    //    for(int i = 0; i < 6; i++)
    //        for(int j = 0; j < 6; j++)
    //        {
    //            JJ.value[i][j] = J(i,j);
    //            AA.value[i][j] = A(i,j);
    //        }


    //    std::cout << J0 << std::endl<< J1 << std::endl;
}

VectorXd JacobianTransposeIK(VectorXd e, VectorXd q)
{
    double alpha = 0;
    MatrixXd J(6,6);
    VectorXd Je(6), JJ(6), dq(6);
    dq << 0,0,0,0,0,0;
    //GetJacobian(J, q);
    Je = J.transpose() * e;
    JJ = J * Je;
    double JJ_Norm = JJ.dot(JJ);
    if(JJ_Norm < 1e-5)
        return dq;
    alpha = e.dot(JJ) / JJ_Norm;
    dq = Je * alpha;
    return dq;
}

VectorXd DLS_IK(VectorXd e, VectorXd q)
{
    MatrixXd J(6,6), Jt(6,6), JJt(6,6), J1(6,6), One;
    VectorXd dq(6);
    dq << 0,0,0,0,0,0;
    double wt = 1, lambda_0 = 1;
    double w, Damp_Factor;

    //GetJacobian(J, q);
    Jt = J.transpose();
    JJt = J * Jt;
    w = sqrt(JJt.determinant());
    if(w < wt)
    {
        double a = 1 - w / wt;
        Damp_Factor = lambda_0 * (a * a);
    }
    else
        Damp_Factor = 0;
    J1 = JJt  - Damp_Factor * One.setIdentity(6,6);
    dq = Jt * J1.inverse() * e;
    return dq;
}

VectorXd IncreOfpose(MatrixXd Td, MatrixXd Tc)
{
    MatrixXd Rd,Rc,dR(3,3),One; //desire ,current end effector
    Vector3d Pd,Pc,dtheta;
    VectorXd e(6);

    Rd = Td.block(0,0,3,3);
    Pd = Td.block(0,3,3,1);

    Rc = Tc.block(0,0,3,3);
    Pc = Tc.block(0,3,3,1);

    dR = Rd * Rc.transpose()-One.setIdentity(3,3);
    dtheta = Vex(dR);
    e << (Pd - Pc), dtheta ;

    return e;
}

MatrixXd processIK(const MatrixXd T, VectorXd& q_old)
{
    MatrixXd q_sols(6,8), q_temp(6,8), q_realsols;
    int num_sols = 0,real_sols = 0;
    bool IsSingularity = false;

    num_sols = inverse(q_sols, T, IsSingularity);
    if(IsSingularity || num_sols == 0)
    {
        for(int i = 0; i < num_sols; i++)
        {
            MatrixXd TT(4,4);
            VectorXd temp(6);
            double err = 0;

            temp = q_sols.col(i);
            forward(TT, temp);

            for(int j = 0; j < 4; j++)
            {
                for(int k = 0; k < 4; k++)
                    err += fabs(TT(j,k) - T(j,k));
            }
            if(err < 1e-5)
            {
                real_sols++;
                q_temp.col(real_sols - 1) = q_sols.col(i);
            }
        }
        q_realsols = q_temp.block(0,0,6,real_sols);

        if(real_sols == 0)
        {
            double differ = 2;
            int n = 0;

            std::cout <<"sols == 0 "<< std::endl;
            VectorXd e(6),dq,ds(3);
            MatrixXd Te(4,4);
            while(differ > 1e-1 && n < MAXIUM_ITERATOR)
            {

                forward(Te, q_old);
                e = IncreOfpose(T,Te);
                ds << e(0), e(1), e(2);
                differ = ds.norm();

                dq = JacobianTransposeIK(e, q_old);
                q_realsols = q_old + dq;

                q_old = q_realsols.col(0);
                n++;
            }
            if(n == MAXIUM_ITERATOR)
            {
                // todo: DLS algorithm
                n = 0;
                while(differ > 1e-1 && n < MAXIUM_ITERATOR)
                {

                    forward(Te, q_old);
                    e = IncreOfpose(T,Te);
                    ds << e(0), e(1), e(2);
                    differ = ds.norm();
                    dq = DLS_IK(e, q_old);
                    q_realsols = q_old + dq;
                    q_old = q_realsols.col(0);
                    n++;
                }
            }
            if(n == MAXIUM_ITERATOR)
                std::cout <<"DLS NO SOLU"<< std::endl ;
        }
        return q_realsols;
    }
    else
    {
#ifdef TEST
        for(int i = 0; i < num_sols; i++)
        {
            MatrixXd TT(4,4);
            VectorXd temp(6);
            double err = 0;

            temp = q_sols.col(i);
            foreward(TT, temp);

            for(int j = 0; j < 4; j++)
            {
                for(int k = 0; k < 4; k++)
                    err += fabs(TT(j,k) - T(j,k));
            }
            if(err > 1e-5)
            {
                std::cout <<"fatal error!!!"<< std::endl;
            }
        }
#endif
        return q_sols;
    }
}


bool decisionIK(MatrixXd q_sols, VectorXd& q_old)
{
    //TODO LIST: configuration flag
    int nn = q_sols.cols();
    if(nn == 0)
        return false;
    double sum = (q_old - q_sols.col(0)).norm();
    double err;
    int index = 0;
    for(int i = 0; i < nn; i++)
    {
        err = (q_old - q_sols.col(i)).norm();
        if(err < sum)
        {
            index = i;
        }

    }
    q_old = q_sols.col(index);
    return true;
}

int test2()
{
    unsigned int NUM = 10000;

    VectorXd  q_solution(6);
    q_solution<<0,0,0,0,0,0;

    while(NUM)
    {
        VectorXd q(6);
        MatrixXd T(4,4);

        srand(NUM); //初始化随机数种子
        for (int i = 0; i < 6; i++)
        {
            q(i) = rand() % (MAX - MIN + 1) + MIN; //random angle degree
        }

        for(int i = 0; i < 6; i++)
        {
            q(i) = q(i) * M_PI / 180;      // degree to radian
        }

        //        q<<1,1,0,1,1,1;
        if(NUM == 6172)
            printf("stop!!!\n");
        forward(T, q);

        MatrixXd q_sols = processIK(T, q_solution);
        bool res = decisionIK(q_sols,q_solution);
        if(res == false)
        {
            std::cout <<"No solution"<< NUM << std::endl;
        }
        NUM--;
    }
    return 0;
}
