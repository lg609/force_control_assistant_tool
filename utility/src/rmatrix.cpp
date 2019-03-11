#include "../include/rmatrix.h"
#include "../include/rvector.h"

#define rep(i,n) for(int i=1;i<=n;i++) //宏定义for循环，精简代码

RMatrix::RMatrix(void)
{
    IsAmpty = true;
}

RMatrix::RMatrix(int m)
{
    iRow = m;
    iCol = m;
    if (iRow)
    {
        IsAmpty = false;
    }
    else
    {
        IsAmpty = true;
    }

    for(std::size_t i = 0;i < iRow;i++)
        for(std::size_t j = 0;j < iCol;j++)
        {
            if(i == j)
                value[i][i] = 1;
            else
                value[i][j] = 0;
        }

    for(std::size_t i = iRow;i < MATRIX_SIZE;i++)
        for(std::size_t j = iCol;j < MATRIX_SIZE;j++)
        {
            value[i][j] = 0;
        }
}

//一般矩阵初始化size(m,n),zeros
RMatrix::RMatrix(const int m,const int n)
{
    iRow = m;
    iCol = n;
    if (iRow && iCol )
    {
        IsAmpty = false;
    }
    else
    {
        IsAmpty = true;
    }

    for(std::size_t i = 0;i < MATRIX_SIZE;i++)
        for(std::size_t j = 0;j < MATRIX_SIZE;j++)
            this->value[i][j] = 0;
}

// A = B,size(4*4)
RMatrix::RMatrix(double a[4][4],int m,int n)
{
    iRow = m;
    iCol = n;
    for(std::size_t i = 0;i < iRow;i++)
        for(std::size_t j = 0;j < iCol;j++)
            value[i][j] = a[i][j];

    for(std::size_t i = iRow;i < MATRIX_SIZE;i++)
        for(std::size_t j = iCol;j < MATRIX_SIZE;j++)
        {
            value[i][j] = 0;
        }
}
//T = [R,P]
RMatrix::RMatrix(double *eerot, double *eetrans)
{
    //creat the manipulation transformation matrix
    iRow = 4;
    iCol = 4;
    for(std::size_t i = 0;i < iRow;i++)
    {
        for(std::size_t j = 0;j < iCol;j++)
            value[i][j] = eerot[i * 3 + j];
        value[i][3] = eetrans[i];
    }

    for(std::size_t i = iRow;i < MATRIX_SIZE;i++)
        for(std::size_t j = iCol;j < MATRIX_SIZE;j++)
        {
            value[i][j] = 0;
        }
}

RMatrix::~RMatrix(void)
{
}

RMatrix::RMatrix(const RMatrix &other)
{
    this->iRow = other.iRow;
    this->iCol = other.iCol;
    this->IsAmpty = other.IsAmpty;

    for (int i = 0; i < MATRIX_SIZE; i++)
        for (int j = 0; j < MATRIX_SIZE; j++)
            this->value[i][j] = other.value[i][j];
}

RMatrix RMatrix::operator=(const RMatrix &other)
{
    {
        this->iRow = other.iRow;
        this->iCol = other.iCol;
        this->IsAmpty = other.IsAmpty;

        for (int i = 0; i < MATRIX_SIZE; i++)
            for (int j = 0; j < MATRIX_SIZE; j++)
                this->value[i][j] = other.value[i][j];
    }
    return *this;
}

//RMatrix RMatrix::operator*(const RMatrix &A, const RMatrix &B)
//{
//    RMatrix C(A.iRow,B.iCol);
//    for (int i = 0; i < C.iRow; i++)
//    {
//        for (int j = 0; j < C.iCol; j++)
//        {
//            double sum=0;
//            for (int k = 0; k < A.iCol; k++)
//                sum += A.value[i][k]*B.value[k][j];
//            C.value[i][j] = sum;
//        }
//    }
//    return C;
//}

RMatrix RMatrix::operator*(const RMatrix &other)
{
    RMatrix C(this->iRow,other.iCol);
    for (int i = 0; i < this->iRow; i++)
    {
        for (int j = 0; j < other.iCol; j++)
        {
            double sum=0;
            for (int k = 0; k < this->iCol; k++)
                sum += this->value[i][k]*other.value[k][j];
            C.value[i][j] = sum;
        }
    }
    return C;
}

//eyes(n);
RMatrix RMatrix::RIdentity(int n)
{
    RMatrix c(n, n);
    for (std::size_t i = 0; i < n; i++)
        c.value[i][i] = 1;
    return c;
}

//RMatrix RMatrix::RTranspose(const RMatrix &other)
//{
//    RMatrix C(other.iCol,other.iCol);
//    for (int i = 0; i < MATRIX_SIZE; i++)
//        for (int j = 0; j < MATRIX_SIZE; j++)
//            C.value[i][j] = other.value[j][i];
//    return C;

//}

RMatrix RMatrix::RTranspose(const RMatrix A)
{
    int m = A.iCol;
    int n = A.iRow;
    RMatrix C(m, n);
    for(std::size_t i=0;i<m;i++)
        for(std::size_t j=0;j<n;j++)
            C.value[i][j] = A.value[j][i];
    return C;
}

//row mo to m1;column no to n1;
RMatrix RMatrix::subRMatrix(RMatrix A,int m0,int m1,int n0,int n1)
{
    int m = m1 - m0 + 1;
    int n = n1 - n0 + 1;
    RMatrix rm(m, n);
    for(std::size_t i=0;i<m;i++)
        for(std::size_t j=0;j<n;j++)
            rm.value[i][j] = A.value[m0 + i][n0 + j];
    return rm;
}

RMatrix RMatrix::Quaternion2RotMatrix(RVector &a)
{
    // the 3D rotation matrix dimension: 3X3
    RMatrix rm(3, 3);
    rm.value[0][0] = 2 * (a.value[0]*a.value[0] + a.value[1]*a.value[1]) - 1;
    rm.value[0][1] = 2 * (a.value[1]*a.value[2] - a.value[0]*a.value[3]);
    rm.value[0][2] = 2 * (a.value[1]*a.value[3] + a.value[0]*a.value[2]);
    rm.value[1][0] = 2 * (a.value[1]*a.value[2] + a.value[0]*a.value[3]);
    rm.value[1][1] = 2 * (a.value[0]*a.value[0] + a.value[2]*a.value[2]) - 1;
    rm.value[1][2] = 2 * (a.value[2]*a.value[3] - a.value[0]*a.value[1]);
    rm.value[2][0] = 2 * (a.value[1]*a.value[3] - a.value[0]*a.value[2]);
    rm.value[2][1] = 2 * (a.value[2]*a.value[3] + a.value[0]*a.value[1]);
    rm.value[2][2] = 2 * (a.value[0]*a.value[0] + a.value[3]*a.value[3]) - 1;

    return rm;
}

RMatrix RMatrix::RPY2TR(RVector rpy)
{
    RMatrix rm(3, 3);

    rm = RotZ(rpy.value[0]) *  RotY(rpy.value[1]) * RotX(rpy.value[2]);
//    double cosR = cos(rpy.value[2]);        //rot(X)
//    double sinR = sin(rpy.value[2]);
//    double cosP = cos(rpy.value[1]);        //rot(Y)
//    double sinP = sin(rpy.value[1]);
//    double cosY = cos(rpy.value[0]);        //rot(Z)
//    double sinY = sin(rpy.value[0]);

//    rm.value[0][0] = cosY * cosP;
//    rm.value[0][1] = cosY * sinP * sinR - sinY * cosR;
//    rm.value[0][2] = cosY * sinP * cosR + sinY * sinR;
//    rm.value[1][0] = sinY * cosP;
//    rm.value[1][1] = sinY * sinP * sinR + cosY * cosR;
//    rm.value[1][2] = sinY * sinP * cosR - cosY * sinR;
//    rm.value[2][0] = -sinP;
//    rm.value[2][1] = cosP * sinR;
//    rm.value[2][2] = cosP * cosR;
    return rm;
}

RMatrix RMatrix::RotZ(double t)
{
    double   ct = cos(t);
    double   st = sin(t);
    RMatrix rz(3);
    rz.value[0][0] = ct;
    rz.value[0][1] = -st;
    rz.value[1][0] = st;
    rz.value[1][1] = ct;

    return rz;
}

RMatrix RMatrix::RotY(double t)
{
    double   ct = cos(t);
    double   st = sin(t);
    RMatrix ry(3);
    ry.value[0][0] = ct;
    ry.value[0][2] = st;
    ry.value[2][0] = -st;
    ry.value[2][2] = ct;

    return ry;
}

RMatrix RMatrix::RotX(double t)
{
    double   ct = cos(t);
    double   st = sin(t);
    RMatrix rx(3);
    rx.value[1][1] = ct;
    rx.value[1][2] = -st;
    rx.value[2][1] = st;
    rx.value[2][2] = ct;

    return rx;
}
//A = [B,D;C]
void RMatrix::catRMatrix(RMatrix &A,int m0,int m1,int n0,int n1,RMatrix &B)
{
    for (std::size_t i = m0;i <= m1;i++)
    {
        for (std::size_t j = n0;j <= n1;j++)
        {
            A.value[i][j] = B.value[i-m0][j-n0];
        }
    }
}

void RMatrix::catRMatrix(RMatrix &A,int m0,int m1,int n0,RVector &B)
{
    for (std::size_t i = m0;i <= m1;i++)
    {
        A.value[i][n0] = B.value[i];
    }
}

void RMatrix::replaceRMatrix(RMatrix &A,int m0,string type,RVector &b)
{
    for (std::size_t i = 0;i < b.size;i++)
    {
        A.value[i][m0] = b.value[i];
    }
}

void RMatrix::replaceOriOfTMatrix(RMatrix &A, RMatrix B)
{
    //replace the rotation matrix of the homogenous matrix
    for (std::size_t i = 0;i < 3;i++)
        for(std::size_t j = 0;j < 3;j++)
            A.value[i][j] = B.value[i][j];
}

void RMatrix::replaceDisOfTMatrix(RMatrix &A, RVector &b)
{
    //replace the displacement of the homogenous matrix
    for (std::size_t i = 0;i < 3;i++)
            A.value[i][3] = b.value[i];
}


RVector RMatrix::subRVector(RMatrix &A,int m0,int m1,int n,string type)
{
    std::size_t size = m1 - m0 + 1;
    RVector rm(size);
    if(type == "Column")
    {
        for(std::size_t i=0;i < size;i++)
            rm.value[i] = A.value[m0 + i][n];
    }
    else if(type == "Row")
    {
        for(std::size_t i=0;i < size;i++)
            rm.value[i] = A.value[n][m0 + i];
    }
    else
    {

    }
    return rm;
}

RMatrix RMatrix::triuRMatrix(RMatrix &A)
{
    RMatrix C = A;
    for(std::size_t i=0;i<C.iRow;i++)
        C.value[i][i] = 0;
    return C;

}

double RMatrix::normRMatrix(RMatrix &A,int n)
{
    double res = 0;
    for(std::size_t i=0;i<A.iCol;i++)
        for(std::size_t j=0;j<A.iRow;j++)
            res += A.value[i][j] * A.value[i][j];
    return sqrt(res);

}

void RMatrix::clearRMatrix(RMatrix &A)
{
    for(std::size_t i=0;i<A.iCol;i++)
        for(std::size_t j=0;j<A.iRow;j++)
            A.value[i][j] = 0;
}

void RMatrix::svdSim(RMatrix &A,RMatrix &U,RMatrix &S,RMatrix &V)
{
    double tol = 1e-13;			//精度控制
    int loopmax = 300;			//迭代次数

    int loopcount = 0;			//计数器
    S = RMatrix::RTranspose(A);

    double Err =  10000000;  //1.7977e+308
    while (Err > tol && loopcount < loopmax)
    {
        // log10([Err tol loopcount loopmax]); pause
        RMatrix Q(S.iRow,S.iRow);
        RMatrix::qrFullRMatrix(S,Q);
        U = U * Q;
        RMatrix::qrFullRMatrix(S,Q);
        V =V * Q;

        /*RMatrix E(S.iRow,S.iCol);*/
        RMatrix E = RMatrix::triuRMatrix(S);
        double e = RMatrix::normRMatrix(E,1);
        double f = 0;
        for (int i = 0;i<S.iRow;i++) f += S.value[i][i] * S.value[i][i];
        f = sqrt(f);
        if (f == 0) f = 1;
        Err = e / f;
        loopcount = loopcount + 1;
    }

    //调整输出U S 矩阵
    RVector SS = RVector::diagRVector(S);			//获得S矩阵的对角元素
    RMatrix::clearRMatrix(S);
    double ssn = 0;
    for(int i=0;i<SS.size;i++)
    {
        ssn = SS.value[i];
        S.value[i][i] = fabs(ssn);
        if (ssn < 0)
        {
            for(int j=0;j<U.iRow;j++)
                U.value[j][i] = -U.value[j][i];
        }
    }

}

void RMatrix::qrFullRMatrix(RMatrix &A,RMatrix &Q)
{
    double a[36];
    double q[36];
    int m = 6;
    int n = 6;
    int i, j, k, l, nn, p, jj;
    double u, alpha, w, t;

    //转置矩阵的雅克比
    for (int i = 0;i < A.iCol;i++)
    {
        for (int j = 0;j < A.iRow;j++)
        {
            a[i*6+j] = A.value[j][i];
        }
    }

    for (i = 0; i <= m - 1; i++)
        for (j = 0; j <= m - 1; j++)
        {
            l = i * m + j; q[l] = 0.0;
            if (i == j) q[l] = 1.0;
        }
        nn = n;
        if (m == n) nn = m - 1;
        for (k = 0; k <= nn - 1; k++)
        {
            u = 0.0; l = k * n + k;
            for (i = k; i <= m - 1; i++)
            {
                w =fabs(a[i * n + k]);
                if (w > u) u = w;
            }
            alpha = 0.0;
            for (i = k; i <= m - 1; i++)
            {
                t = a[i * n + k] / u;
                alpha = alpha + t * t;
            }
            if (a[l] > 0.0) u = -u;
            alpha = u * sqrt(alpha);
            /*if (fabs(alpha) + 1.0 == 1.0)
            {
            return (0);
            }*/
            u = sqrt(2.0 * alpha * (alpha - a[l]));
            if ((u + 1.0) != 1.0)
            {
                a[l] = (a[l] - alpha) / u;
                for (i = k + 1; i <= m - 1; i++)
                {
                    p = i * n + k; a[p] = a[p] / u;
                }
                for (j = 0; j <= m - 1; j++)
                {
                    t = 0.0;
                    for (jj = k; jj <= m - 1; jj++)
                        t = t + a[jj * n + k] * q[jj * m + j];
                    for (i = k; i <= m - 1; i++)
                    {
                        p = i * m + j; q[p] = q[p] - 2.0 * t * a[i * n + k];
                    }
                }
                for (j = k + 1; j <= n - 1; j++)
                {
                    t = 0.0;
                    for (jj = k; jj <= m - 1; jj++)
                        t = t + a[jj * n + k] * a[jj * n + j];
                    for (i = k; i <= m - 1; i++)
                    {
                        p = i * n + j; a[p] = a[p] - 2.0 * t * a[i * n + k];
                    }
                }
                a[l] = alpha;
                for (i = k + 1; i <= m - 1; i++)
                    a[i * n + k] = 0.0;
            }
        }
        for (i = 0; i <= m - 2; i++)
            for (j = i + 1; j <= m - 1; j++)
            {
                p = i * m + j; l = j * m + i;
                t = q[p]; q[p] = q[l]; q[l] = t;
            }

        for (int i = 0;i < A.iRow;i++)
        {
            for (int j = 0;j < A.iCol;j++)
            {
                A.value[i][j] = a[i*6+j];
            }
        }

        for (int i = 0;i < A.iRow;i++)
        {
            for (int j = 0;j < A.iCol;j++)
            {
                Q.value[i][j] = q[i*6+j];
            }
        }
}

// 矩阵求逆
bool RMatrix::RMatrixInv(RMatrix A, RMatrix &Mc)
{
    int m = A.iRow;
    int n = A.iCol;
    if (m != n)
    {
        return false;
    }

    double a[6][6];
    for (int i = 0;i<m;i++)
        for (int j = 0;j<n;j++)
            a[i][j] = A.value[i][j];
    double b[6][6] = {0};

    int i, j, row, k;
    double max, temp;

    //单位矩阵
    for (i = 0; i < n; i++)
        b[i][i] = 1;
    for (k = 0; k < n; k++)
    {
        max = 0; row = k;
        //找最大元，其所在行为row
        for (i = k; i < n; i++)
        {
            temp = fabs(a[i][k]);
            if (max < temp)
            {
                max = temp;
                row = i;
            }
        }
        if (max == 0)
        {
            return false;
            //("没有逆矩阵");
        }
        //交换k与row行
        if (row != k)
        {
            for (j = 0; j < n; j++)
            {
                temp = a[row][j];
                a[row][j] = a[k][j];
                a[k][j] = temp;

                temp = b[row][j];
                b[row][j] = b[k][j];
                b[k][j] = temp;
            }
        }

        //首元化为1
        for (j = k + 1; j < n; j++)
            a[k][j] /= a[k][k];
        for (j = 0; j < n; j++)
            b[k][j] /= a[k][k];

        a[k][k] = 1;

        //k列化为0
        //对a
        for (j = k + 1; j < n; j++)
        {
            for (i = 0; i < k; i++) a[i][j] -= a[i][k] * a[k][j];
            for (i = k + 1; i < n; i++) a[i][j] -= a[i][k] * a[k][j];
        }
        //对b
        for (j = 0; j < n; j++)
        {
            for (i = 0; i < k; i++) b[i][j] -= a[i][k] * b[k][j];
            for (i = k + 1; i < n; i++) b[i][j] -= a[i][k] * b[k][j];
        }
        for (i = 0; i < n; i++) a[i][k] = 0;
        a[k][k] = 1;
    }
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            Mc.value[i][j] = b[i][j];

    return true;
}


//RMatrix operator*(const RMatrix &A,const RMatrix &B) //
//{
//    RMatrix C(A.iRow,B.iCol);
//    for (int i = 0; i < C.iRow; i++)
//    {
//        for (int j = 0; j < C.iCol; j++)
//        {
//            double sum=0;
//            for (int k = 0; k < A.iCol; k++)
//                sum += A.value[i][k]*B.value[k][j];
//            C.value[i][j] = sum;
//        }
//    }
//    return C;
//}

RVector operator* (const RMatrix &A,const RVector &b) //乘法定义
{
    if (A.iCol != b.size)
    {

    }
    RVector c(b.size);
    //RVector c(b.size);
    for (int i = 0; i < A.iCol; i++)
        for (int j = 0; j < A.iRow; j++)
            c.value[i] += A.value[i][j]*b.value[j];
    return c;
}


RMatrix operator+ (const RMatrix &A,const RMatrix &B) {
    int m = A.iRow;
    int n = A.iCol;
    RMatrix C(m, n);
    for (int i = 0; i < m; i++)
        for (int j = 0; j <n; j++)
            C.value[i][j] = A.value[i][j] + B.value[i][j];
    return C;
}

//RMatrix operator- (RMatrix &A,RMatrix &B) {
//    int m = A.iRow;
//    int n = A.iCol;
//    RMatrix C(m, n);
//    for (int i = 0; i < m; i++)
//        for (int j = 0; j <n; j++)
//            C.value[i][j] = A.value[i][j] - B.value[i][j];
//    return C;
//}

RMatrix operator- (const RMatrix &A, const RMatrix &B) {
    int m = A.iRow;
    int n = A.iCol;
    RMatrix C(m, n);
    for (int i = 0; i < m; i++)
        for (int j = 0; j <n; j++)
            C.value[i][j] = A.value[i][j] - B.value[i][j];
    return C;
}



