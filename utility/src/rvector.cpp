#include "../include/rvector.h"

RVector::RVector()
{

}

RVector::RVector(int m){
    size = m;
    for(int i = 0;i< m;i++)
        value[i] = 0;
}


RVector::RVector(double m[],int n){
    size = n;
    for(int i = 0;i< n;i++)
        value[i] = m[i];
}

RVector::~RVector(void)
{
}

double RVector::norm(RVector d){
    double sum = 0;
    for(int i = 0;i< d.size;i++)
        sum += d.value[i]*d.value[i];
    return sqrt(sum);
}

RVector RVector::vex3(RMatrix A)
{
    RVector c(3);
    c.value[0] = A.value[2][1];
    c.value[1] = A.value[0][2];
    c.value[2] = A.value[1][0];
    return c;
}

RMatrix RVector::Skew(RVector S)
{
    RMatrix A(3,3);
    A.value[0][0] = 0;
    A.value[0][1] = -S.value[2];
    A.value[0][2] = S.value[1];
    A.value[1][0] = S.value[2];
    A.value[1][1] = 0;
    A.value[1][2] = -S.value[0];
    A.value[2][0] = -S.value[1];
    A.value[2][1] = S.value[0];
    A.value[2][2] = 0;

    return A;
}

RVector RVector::cross3(RVector a, RVector b)
{
    RVector c(3);
    c.value[0] =  a.value[1] * b.value[2] - a.value[2] * b.value[1];
    c.value[1] = -(a.value[0] * b.value[2] - a.value[2] * b.value[0]);
    c.value[2] = a.value[0] * b.value[1] - a.value[1] * b.value[0];
    return c;
}

RVector RVector::catRVector(RVector &a, RVector b)
{
    RVector c(a.size + b.size);
    for(int i = 0;i< a.size + b.size;i++)
    {
        if (i < a.size)
        {
            c.value[i] = a.value[i];
        }
        else
        {
            c.value[i] = b.value[i - a.size];
        }
    }
    return c;
}

RVector RVector::diagRVector(RMatrix &A)
{
    RVector c(A.iRow);
    for(int i = 0;i< A.iRow;i++)
        c.value[i] = A.value[i][i];
    return c;
}

void RVector::processAngle(RVector &a)
{
    for(int i = 0;i<a.size;i++)
    {
        //[-PI,PI)
        while (a.value[i] >= M_PI) a.value[i] -= 2*M_PI;
        while (a.value[i] < -M_PI) a.value[i] += 2*M_PI;
        //        if (a.value[i] > JOINT_MAX_POS) a.value[i] = JOINT_MAX_POS;
        //        if (a.value[i] < -JOINT_MAX_POS) a.value[i] = -JOINT_MAX_POS;
        //        int n = (int)(a.value[i] / JOINT_MAX_POS);
        //        a.value[i] = a.value[i] - n * JOINT_MAX_POS;
    }

}

double RVector::dotRVector(RVector &a, RVector &b)
{
    double sum = 0;
    for(int i = 0;i< a.size;i++)
        sum += a.value[i] * b.value[i];
    return sum;
}

RVector RVector::TR2RPY(RMatrix RotN)
{
    RVector c(3);
    double eps = 1e-16;
    // old ZYX order (as per Paul book)
    if (fabs(RotN.value[0][0]) < eps && fabs(RotN.value[1][0]) < eps)
    {
        //singularity
        c.value[0] = 0.0;
        c.value[1] = atan2(-RotN.value[2][0], RotN.value[0][0]);
        c.value[2] = atan2(-RotN.value[1][2], RotN.value[1][1]);
    }
    else
    {
        c.value[0] = atan2(RotN.value[1][0], RotN.value[0][0]);
        double sp = sin(c.value[0]);
        double cp = cos(c.value[0]);
        c.value[1] = atan2(-RotN.value[2][0], cp * RotN.value[0][0] + sp * RotN.value[1][0]);
        c.value[2] = atan2(sp * RotN.value[0][2] - cp * RotN.value[1][2], cp * RotN.value[1][1] - sp * RotN.value[0][1]);
    }

    //    c.value[0] = atan2(RotN.value[1][0], RotN.value[0][0]);
    //    c.value[1] = atan2(-RotN.value[2][0], sqrt(RotN.value[0][0] * RotN.value[0][0] + RotN.value[1][0] * RotN.value[1][0]));
    //    c.value[2] = atan2(RotN.value[2][1], RotN.value[2][2]);
    return c;
}

bool RVector::Ori2Quaternion(RMatrix rot, RVector &c)
{
    //    RVector c(4);
    c.value[0] = ( rot.value[0][0] + rot.value[1][1] + rot.value[2][2] + 1.0f) / 4.0f;
    c.value[1] = ( rot.value[0][0] - rot.value[1][1] - rot.value[2][2] + 1.0f) / 4.0f;
    c.value[2] = (-rot.value[0][0] + rot.value[1][1] - rot.value[2][2] + 1.0f) / 4.0f;
    c.value[3] = (-rot.value[0][0] - rot.value[1][1] + rot.value[2][2] + 1.0f) / 4.0f;
    if(c.value[0] < 0.0f) c.value[0] = 0.0f;
    if(c.value[1] < 0.0f) c.value[1] = 0.0f;
    if(c.value[2] < 0.0f) c.value[2] = 0.0f;
    if(c.value[3] < 0.0f) c.value[3] = 0.0f;

    c = sqrtRVector(c);

    if(c.value[0] >= c.value[1] && c.value[0] >= c.value[2] && c.value[0] >= c.value[3])
    {
        c.value[1] *= (rot.value[2][1] - rot.value[1][2])>=0.0f?1:-1;
        c.value[2] *= (rot.value[0][2] - rot.value[2][0])>=0.0f?1:-1;
        c.value[3] *= (rot.value[1][0] - rot.value[0][1])>=0.0f?1:-1;
    }
    else if(c.value[1] >= c.value[0] && c.value[1] >= c.value[2] && c.value[1] >= c.value[3])
    {
        c.value[0] *= (rot.value[2][1] - rot.value[1][2])>=0.0f?1:-1;
        c.value[2] *= (rot.value[1][0] + rot.value[0][1])>=0.0f?1:-1;
        c.value[3] *= (rot.value[0][2] + rot.value[2][0])>=0.0f?1:-1;
    }
    else if(c.value[2] >= c.value[0] && c.value[2] >= c.value[1] && c.value[2] >= c.value[3])
    {
        c.value[0] *= (rot.value[0][2] - rot.value[2][0])>=0.0f?1:-1;
        c.value[1] *= (rot.value[1][0] + rot.value[0][1])>=0.0f?1:-1;
        c.value[3] *= (rot.value[2][1] + rot.value[1][2])>=0.0f?1:-1;
    }
    else if(c.value[3] >= c.value[0] && c.value[3] >= c.value[1] && c.value[3] >= c.value[2])
    {
        c.value[0] *= (rot.value[1][0] - rot.value[0][1])>=0.0f?1:-1;
        c.value[1] *= (rot.value[2][0] + rot.value[0][2])>=0.0f?1:-1;
        c.value[2] *= (rot.value[2][1] + rot.value[1][2])>=0.0f?1:-1;
    }
    else return false;

    c = c / norm(c);

    //    c.value[0] = sqrt(rot.value[0][0] + rot.value[1][1] + rot.value[2][2] + 1.0f) / 2;
    //    c.value[1] = ((rot.value[2][1] - rot.value[1][2]) >= 0.0f ? 1: -1) * sqrt(rot.value[0][0] - rot.value[1][1] - rot.value[2][2] + 1.0f) / 2;
    //    c.value[2] = ((rot.value[0][2] - rot.value[2][0]) >= 0.0f ? 1: -1) * sqrt(rot.value[1][1] - rot.value[2][2] - rot.value[0][0] + 1.0f) / 2;
    //    c.value[3] = ((rot.value[1][0] - rot.value[0][1]) >= 0.0f ? 1: -1) * sqrt(rot.value[2][2] - rot.value[0][0] - rot.value[1][1] + 1.0f) / 2;
    //    return c;
    return true;
}

RVector RVector::sqrtRVector(RVector a)
{
    RVector c(a.size);
    for(int i = 0; i< c.size; i++)
        c.value[i] = sqrt(a.value[i]);
    return c;
}

RVector operator- (const RVector &a, const RVector &b)
{
    RVector c(a.size);
    for(int i = 0;i< a.size;i++)
        c.value[i] = a.value[i] - b.value[i];
    return c;
}

RVector operator-(const RVector &a)
{
    RVector c(a.size);
    for(int i = 0; i< c.size;i++)
        c.value[i] = -a.value[i];
    return c;
}

RVector operator+ (const RVector &a, const RVector &b)
{
    RVector c(a.size);
    for(int i = 0;i< a.size;i++)
        c.value[i] = a.value[i] + b.value[i];
    return c;
}

RVector operator* (const RVector &a, const double b)
{
    RVector c(a.size);
    for(int i = 0;i< a.size;i++)
        c.value[i] = a.value[i] * b;
    return c;
}

RVector operator/ (const RVector &a, const double b)
{
    RVector c(a.size);
    for(int i = 0;i< a.size;i++)
        c.value[i] = a.value[i] / b;
    return c;
}

RVector operator/ (const double b, const RVector &a)
{
    RVector c(a.size);
    for(int i = 0;i< a.size;i++)
        c.value[i] = b / a.value[i];
    return c;
}

RVector operator* (double b, RVector &a)
{
    RVector c(a.size);
    for(int i = 0;i< a.size;i++)
        c.value[i] = a.value[i] * b;
    return c;
}
