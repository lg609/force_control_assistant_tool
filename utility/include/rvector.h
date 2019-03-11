#ifndef RVECTOR_H
#define RVECTOR_H

#include<math.h>
#include<iomanip>
#include "rmatrix.h"

class RVector
{
public:
    RVector();
    RVector(int m);
    RVector(double m[],int n);

    ~RVector(void);

    friend RVector operator*(const RMatrix &A, const RVector &b);
    friend RVector operator-(const RVector &a, const RVector &b);
    friend RVector operator-(const RVector &a);
    friend RVector operator+(const RVector &a, const RVector &b);
    friend RVector operator*(const RVector &a, const double b);
    friend RVector operator*(double b, const RVector &a);
    friend RVector operator/(const RVector &a, double b);
    friend RVector operator/(double b, const RVector &a);
    //成员函数
    static double norm(RVector d);
    static RVector vex3(RMatrix A);
    static RMatrix Skew(RVector S);
    static RVector cross3(RVector a, RVector b);

    static RVector catRVector(RVector &a, RVector b);
    static void processAngle(RVector &a);
    static RVector diagRVector(RMatrix &A);
    static double dotRVector(RVector &a, RVector &b);
    static RVector TR2RPY(RMatrix RotN);
    static bool Ori2Quaternion(RMatrix rot, RVector &c);

    static RVector sqrtRVector(RVector a);
    //成员变量
    int size;
    bool IsAmpty;
    //vector<double> value;
    double value[7];
    string type;
};

#endif // RVECTOR_H
