/*------------------------------------------------------------------------------
* lambda.c : integer ambiguity resolution
*
*          Copyright (C) 2007-2008 by T.TAKASU, All rights reserved.
*
* reference :
*     [1] P.J.G.Teunissen, The least-square ambiguity decorrelation adjustment:
*         a method for fast GPS ambiguity estimation, J.Geodesy, Vol.70, 65-82,
*         1995
*     [2] X.-W.Chang, X.Yang, T.Zhou, MLAMBDA: A modified LAMBDA method for
*         integer least-squares estimation, J.Geodesy, Vol.79, 552-565, 2005
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2007/01/13 1.0 new
*           2015/05/31 1.1 add api lambda_reduction(), lambda_search()
*-----------------------------------------------------------------------------*/
#include <cmath>
#include <iostream>
#include <memory.h>

// 矩阵乘法
void MatrixMulti(const int m, const int n, const int k, const double *a, const double *b, double *multi)
{
    // 矩阵相乘，m×n的矩阵a，n×k的矩阵b
    int i, j, t;
    for (i = 0; i < m * k; ++i)
        multi[i] = 0;
    for (t = 0; t < m; t++)  // multi所有元素
        for (j = 0; j < k; j++)  // multi第一行所有的元素
            for (i = 0; i < n; ++i)  // multi第一行第一列的元素
                multi[j + t * k] += a[i + t * n] * b[i * k + j];
}


/****************************************************************************
  MatrixInv
  
  目的：矩阵求逆,采用全选主元高斯-约当法

  参数:
  n      M1的行数和列数
  a      输入矩阵
  b      输出矩阵   b=inv(a)
  返回值：1=正常，0=无法求逆

****************************************************************************/

int MatrixInv(int n/*矩阵维数*/, const double a[], double b[])
{
    int i, j, k, l, u, v, is[250], js[250];   /* matrix dimension <= 10 */
    double d, p;
    
    /* 将输入矩阵赋值给输出矩阵b，下面对b矩阵求逆，a矩阵不变 */
    memcpy(b, a, n * n * sizeof(double));
    for (k = 0; k < n; k++)
    {
        d = 0.0;
        for (i = k; i < n; i++)   /* 查找右下角方阵中主元素的位置 */
        {
            for (j = k; j < n; j++)
            {
                l = n * i + j;
                p = fabs(b[l]);
                if (p > d)
                {
                    d = p;
                    is[k] = i;
                    js[k] = j;
                }
            }
        }
        
        if (d < 1.0E-12)
        {
            printf("Divided by 0 in MatrixInv!\n");
            return 0;
        }
        
        if (is[k] != k)  /* 对主元素所在的行与右下角方阵的首行进行调换 */
        {
            for (j = 0; j < n; j++)
            {
                u = k * n + j;
                v = is[k] * n + j;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
        
        if (js[k] != k)  /* 对主元素所在的列与右下角方阵的首列进行调换 */
        {
            for (i = 0; i < n; i++)
            {
                u = i * n + k;
                v = i * n + js[k];
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
        
        l = k * n + k;
        b[l] = 1.0 / b[l];  /* 初等行变换 */
        for (j = 0; j < n; j++)
        {
            if (j != k)
            {
                u = k * n + j;
                b[u] = b[u] * b[l];
            }
        }
        for (i = 0; i < n; i++)
        {
            if (i != k)
            {
                for (j = 0; j < n; j++)
                {
                    if (j != k)
                    {
                        u = i * n + j;
                        b[u] = b[u] - b[i * n + k] * b[k * n + j];
                    }
                }
            }
        }
        for (i = 0; i < n; i++)
        {
            if (i != k)
            {
                u = i * n + k;
                b[u] = -b[u] * b[l];
            }
        }
    }
    
    for (k = n - 1; k >= 0; k--)  /* 将上面的行列调换重新恢复 */
    {
        if (js[k] != k)
        {
            for (j = 0; j < n; j++)
            {
                u = k * n + j;
                v = js[k] * n + j;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
        if (is[k] != k)
        {
            for (i = 0; i < n; i++)
            {
                u = i * n + k;
                v = is[k] + i * n;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
    }
    
    return (1);
}

/* constants/macros ----------------------------------------------------------*/

#define LOOPMAX     10000           /* maximum count of search loop */

#define SGN(x)      ((x)<=0.0?-1.0:1.0)
#define ROUND(x)    (floor((x)+0.5))
#define SWAP(x, y)   do {double tmp_; tmp_=x; x=y; y=tmp_;} while (0)

/* LD factorization (Q=L'*diag(D)*L) -----------------------------------------*/
static int LD(int n, const double *Q, double *L, double *D)
{
    int i, j, k, info = 0;
    double a, *A;
    
    A = new double[n * n];
    memcpy(A, Q, sizeof(double) * n * n);
    for (i = n - 1; i >= 0; i--)
    {
        if ((D[i] = A[i + i * n]) <= 0.0)
        {
            info = -1;
            break;
        }
        a = sqrt(D[i]);
        for (j = 0; j <= i; j++) L[i + j * n] = A[i + j * n] / a;
        for (j = 0; j <= i - 1; j++) for (k = 0; k <= j; k++) A[j + k * n] -= L[i + k * n] * L[i + j * n];
        for (j = 0; j <= i; j++) L[i + j * n] /= L[i + i * n];
    }
    
    delete[]A;
    return info;
}

/* integer gauss transformation ----------------------------------------------*/
void gauss(int n, double *L, double *Z, int i, int j)
{
    int k, mu;
    
    if ((mu = (int) ROUND(L[i + j * n])) != 0)
    {
        for (k = i; k < n; k++) L[k + n * j] -= (double) mu * L[k + i * n];
        for (k = 0; k < n; k++) Z[k + n * j] -= (double) mu * Z[k + i * n];
    }
}

/* permutations --------------------------------------------------------------*/
void perm(int n, double *L, double *D, int j, double del, double *Z)
{
    int k;
    double eta, lam, a0, a1;
    
    eta = D[j] / del;
    lam = D[j + 1] * L[j + 1 + j * n] / del;
    D[j] = eta * D[j + 1];
    D[j + 1] = del;
    for (k = 0; k <= j - 1; k++)
    {
        a0 = L[j + k * n];
        a1 = L[j + 1 + k * n];
        L[j + k * n] = -L[j + 1 + j * n] * a0 + a1;
        L[j + 1 + k * n] = eta * a0 + lam * a1;
    }
    L[j + 1 + j * n] = lam;
    for (k = j + 2; k < n; k++) SWAP(L[k + j * n], L[k + (j + 1) * n]);
    for (k = 0; k < n; k++) SWAP(Z[k + j * n], Z[k + (j + 1) * n]);
}

/* lambda reduction (z=Z'*a, Qz=Z'*Q*Z=L'*diag(D)*L) (ref.[1]) ---------------*/
void reduction(int n, double *L, double *D, double *Z)
{
    int i, j, k;
    double del;
    
    j = n - 2;
    k = n - 2;
    while (j >= 0)
    {
        if (j <= k) for (i = j + 1; i < n; ++i) gauss(n, L, Z, i, j);
        del = D[j] + L[j + 1 + j * n] * L[j + 1 + j * n] * D[j + 1];
        if (del + 1E-6 < D[j + 1])
        { /* compared considering numerical error */
            perm(n, L, D, j, del, Z);
            k = j;
            j = n - 2;
        } else j--;
    }
}

/* modified lambda (mlambda) search (ref. [2]) -------------------------------*/
int search(int n, int m, const double *L, const double *D,
           const double *zs, double *zn, double *s)
{
    int i, j, k, c, nn = 0, imax = 0;
    double newdist, maxdist = 1E99, y;
    double *S, *dist, *zb, *z, *step;
    
    S = new double[n * n];
    dist = new double[n];
    zb = new double[n];
    z = new double[n];
    step = new double[n];
    memset(S, 0, n * n * sizeof(double));
    
    k = n - 1;
    dist[k] = 0.0;
    zb[k] = zs[k];
    z[k] = ROUND(zb[k]);
    y = zb[k] - z[k];
    step[k] = SGN(y);
    for (c = 0; c < LOOPMAX; c++)
    {
        newdist = dist[k] + y * y / D[k];
        if (newdist < maxdist)
        {
            if (k != 0)
            {
                dist[--k] = newdist;
                for (i = 0; i <= k; ++i)
                    S[k + i * n] = S[k + 1 + i * n] + (z[k + 1] - zb[k + 1]) * L[k + 1 + i * n];
                zb[k] = zs[k] + S[k + k * n];
                z[k] = ROUND(zb[k]);
                y = zb[k] - z[k];
                step[k] = SGN(y);
            } else
            {
                if (nn < m)
                {
                    if (nn == 0 || newdist > s[imax]) imax = nn;
                    for (i = 0; i < n; ++i) zn[i + nn * n] = z[i];
                    s[nn++] = newdist;
                } else
                {
                    if (newdist < s[imax])
                    {
                        for (i = 0; i < n; ++i) zn[i + imax * n] = z[i];
                        s[imax] = newdist;
                        for (i = imax = 0; i < m; ++i) if (s[imax] < s[i]) imax = i;
                    }
                    maxdist = s[imax];
                }
                z[0] += step[0];
                y = zb[0] - z[0];
                step[0] = -step[0] - SGN(step[0]);
            }
        } else
        {
            if (k == n - 1) break;
            else
            {
                k++;
                z[k] += step[k];
                y = zb[k] - z[k];
                step[k] = -step[k] - SGN(step[k]);
            }
        }
    }
    for (i = 0; i < m - 1; ++i)
    { /* sort by s */
        for (j = i + 1; j < m; j++)
        {
            if (s[i] < s[j]) continue;
            SWAP(s[i], s[j]);
            for (k = 0; k < n; k++) SWAP(zn[k + i * n], zn[k + j * n]);
        }
    }
    
    if (c >= LOOPMAX)
    {
        delete[]S;
        delete[]dist;
        delete[]zb;
        delete[]z;
        delete[]step;
        return -1;
    }
    
    delete[]S;
    delete[]dist;
    delete[]zb;
    delete[]z;
    delete[]step;
    return 0;
}

/* lambda/mlambda integer least-square estimation ------------------------------
* integer least-square estimation. reduction is performed by lambda (ref.[1]),
* and search by mlambda (ref.[2]).
* args   : int    n      I  number of float parameters
*          int    m      I  number of fixed solutions
*          double *a     I  float parameters (n x 1)
*          double *Q     I  covariance matrix of float parameters (n x n)
*          double *F     O  fixed solutions (n x m)
*          double *s     O  sum of squared residulas of fixed solutions (1 x m)
* return : status (0:ok,other:error)
* notes  : matrix stored by column-major order (fortran convension)
*-----------------------------------------------------------------------------*/
int lambda(int n, int m, const double *a, const double *Q, double *F, double *s)
{
    int i, info;
    double *L, *D, *Z, *ZT, *z, *E;
    
    if (n <= 0 || m <= 0) return -1;
    L = new double[n * n];
    D = new double[n];
    Z = new double[n * n];
    ZT = new double[n * n];
    z = new double[n];
    E = new double[n * 2];
    
    memset(L, 0, n * n * sizeof(double));
    memset(Z, 0, n * n * sizeof(double));
    for (i = 0; i < n; ++i) Z[i * (n + 1)] = 1.0;
    
    /* LD factorization */
    if (!(info = LD(n, Q, L, D)))
    {
        /* lambda reduction */
        reduction(n, L, D, Z);
        MatrixMulti(n, n, 1, Z, a, z);
        
        /* mlambda search */
        if (!(info = search(n, m, L, D, z, E, s)))
        {
            MatrixInv(n, Z, ZT);
            MatrixMulti(n, n, 1, ZT, E, F);
            MatrixMulti(n, n, 1, ZT, E + n, F + n);
        }
    }
    
    delete[] L;
    delete[] D;
    delete[] Z;
    delete[] ZT;
    delete[] z;
    delete[] E;
    return info;
}


