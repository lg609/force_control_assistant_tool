#ifndef RMATRIX_H
#define RMATRIX_H

#include<vector>
#include<iomanip>
class RVector;
using namespace std;
#define MATRIX_SIZE 9

class RMatrix
{
public:
    RMatrix();
    RMatrix(int m);
    RMatrix(const int m, const int n);
    RMatrix(double m[9][9]);
    RMatrix(double a[4][4],int m,int n);
    RMatrix(double *eerot, double *eetrans);
    ~RMatrix(void);

    RMatrix(const RMatrix &other);
    RMatrix operator=(const RMatrix &other);

    friend RMatrix operator+(const RMatrix &A,const RMatrix &B);
//    friend RMatrix operator*(const RMatrix &A,const RMatrix &B);
    friend RMatrix operator-(const RMatrix &A,const RMatrix &B);

    RMatrix operator*(const RMatrix &other);
//    RMatrix operator*(const RMatrix &A, const RMatrix &B);


//    static RMatrix RTranspose(RMatrix A);
    static RMatrix RTranspose(const RMatrix other);
    static bool RMatrixInv(RMatrix A, RMatrix &B);
    static RMatrix subRMatrix(RMatrix A,int m0,int n0,int m1,int n1);
    static RVector subRVector(RMatrix &A,int m0,int m1,int n,string type);
    static void catRMatrix(RMatrix &A,int m0,int n0,int m1,int n1,RMatrix &B);
    static void catRMatrix(RMatrix &A,int m0,int m1,int n0,RVector &b);
    static void replaceRMatrix(RMatrix &A,int m0,string type,RVector &B);

    static RMatrix RIdentity(int n);
    static void clearRMatrix(RMatrix &A);

    static void svdSim(RMatrix &A,RMatrix &U,RMatrix &S,RMatrix &V);
    static void qrFullRMatrix(RMatrix &A,RMatrix &Q);
    static RMatrix triuRMatrix(RMatrix &A);
    static double normRMatrix(RMatrix &A,int n);

    static RMatrix Quaternion2RotMatrix(RVector &a);
    static void replaceOriOfTMatrix(RMatrix &A, RMatrix B);
    static void replaceDisOfTMatrix(RMatrix &A, RVector &b);
    static RMatrix RPY2TR(RVector rpy);

    static RMatrix RotZ(double t);
    static RMatrix RotY(double t);
    static RMatrix RotX(double t);


    double value[MATRIX_SIZE][MATRIX_SIZE];
    int iRow;
    int iCol;
    bool IsAmpty;
    //vector<vector<double> > value;
};

#endif // RMATRIX_H
