/*******************************************************************************
 * Copyright (c) 2011 IBM Corporation and others.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *     IBM Corporation - initial API and implementation
 *******************************************************************************/
#ifndef _NR3_H_
#define _NR3_H_

//#define _CHECKBOUNDS_ 1
//#define _USESTDVECTOR_ 1
//#define _USENRERRORCLASS_ 1
//#define _TURNONFPES_ 1

// all the system #include's we'll ever need
#include <fstream>
#include <cmath>
#include <complex>
#include <iostream>
#include <iomanip>
#include <vector>
#include <limits>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <fcntl.h>
#include <string.h>
#include <ctype.h>

using namespace std;

// macro-like inline functions

template<class T>
inline T SQR(const T a) {return a*a;}

template<class T>
inline const T &MAX(const T &a, const T &b)
        {return b > a ? (b) : (a);}

inline float MAX(const double &a, const float &b)
        {return b > a ? (b) : float(a);}

inline float MAX(const float &a, const double &b)
        {return b > a ? float(b) : (a);}

template<class T>
inline const T &MIN(const T &a, const T &b)
        {return b < a ? (b) : (a);}

inline float MIN(const double &a, const float &b)
        {return b < a ? (b) : float(a);}

inline float MIN(const float &a, const double &b)
        {return b < a ? float(b) : (a);}

template<class T>
inline T SIGN(const T &a, const T &b)
    {return b >= 0 ? (a >= 0 ? a : -a) : (a >= 0 ? -a : a);}

inline float SIGN(const float &a, const double &b)
    {return b >= 0 ? (a >= 0 ? a : -a) : (a >= 0 ? -a : a);}

inline float SIGN(const double &a, const float &b)
    {return (float)(b >= 0 ? (a >= 0 ? a : -a) : (a >= 0 ? -a : a));}

template<class T>
inline void SWAP(T &a, T &b)
    {T dum=a; a=b; b=dum;}

// exception handling

#ifndef _USENRERRORCLASS_
#define throw(message) \
{printf("ERROR: %s\n     in file %s at line %d\n", message,__FILE__,__LINE__); throw(1);}
#else
struct NRerror {
    char *message;
    char *file;
    int line;
    NRerror(char *m, char *f, int l) : message(m), file(f), line(l) {}
};
#define throw(message) throw(NRerror(message,__FILE__,__LINE__));
void NRcatch(NRerror err) {
    printf("ERROR: %s\n     in file %s at line %d\n",
        err.message, err.file, err.line);
    exit(1);
}
#endif

// usage example:
//
//  try {
//      somebadroutine();
//  }
//  catch(NRerror s) {NRcatch(s);}
//
// (You can of course substitute any other catch body for NRcatch(s).)


// Vector and Matrix Classes

#ifdef _USESTDVECTOR_
#define NRvector vector
#else

template <class T>
class NRvector {
private:
    int nn; // size of array. upper index is nn-1
    T *v;
public:
    NRvector();
    explicit NRvector(int n);       // Zero-based array
    NRvector(int n, const T &a);    //initialize to constant value
    NRvector(int n, const T *a);    // Initialize to array
    NRvector(const NRvector &rhs);  // Copy constructor
    NRvector & operator=(const NRvector &rhs);  //assignment
    typedef T value_type; // make T available externally
    inline T & operator[](const int i); //i'th element
    inline const T & operator[](const int i) const;
    inline int size() const;
    void resize(int newn); // resize (contents not preserved)
    void assign(int newn, const T &a); // resize and assign a constant value
    ~NRvector();
};

// NRvector definitions

template <class T>
NRvector<T>::NRvector() : nn(0), v(NULL) {}

template <class T>
NRvector<T>::NRvector(int n) : nn(n), v(n>0 ? new T[n] : NULL) {}

template <class T>
NRvector<T>::NRvector(int n, const T& a) : nn(n), v(n>0 ? new T[n] : NULL)
{
    for(int i=0; i<n; i++) v[i] = a;
}

template <class T>
NRvector<T>::NRvector(int n, const T *a) : nn(n), v(n>0 ? new T[n] : NULL)
{
    for(int i=0; i<n; i++) v[i] = *a++;
}

template <class T>
NRvector<T>::NRvector(const NRvector<T> &rhs) : nn(rhs.nn), v(nn>0 ? new T[nn] : NULL)
{
    for(int i=0; i<nn; i++) v[i] = rhs[i];
}

template <class T>
NRvector<T> & NRvector<T>::operator=(const NRvector<T> &rhs)
// postcondition: normal assignment via copying has been performed;
//      if vector and rhs were different sizes, vector
//      has been resized to match the size of rhs
{
    if (this != &rhs)
    {
        if (nn != rhs.nn) {
            if (v != NULL) delete [] (v);
            nn=rhs.nn;
            v= nn>0 ? new T[nn] : NULL;
        }
        for (int i=0; i<nn; i++)
            v[i]=rhs[i];
    }
    return *this;
}

template <class T>
inline T & NRvector<T>::operator[](const int i) //subscripting
{
#ifdef _CHECKBOUNDS_
if (i<0 || i>=nn) {
    throw("NRvector subscript out of bounds");
}
#endif
    return v[i];
}

template <class T>
inline const T & NRvector<T>::operator[](const int i) const //subscripting
{
#ifdef _CHECKBOUNDS_
if (i<0 || i>=nn) {
    throw("NRvector subscript out of bounds");
}
#endif
    return v[i];
}

template <class T>
inline int NRvector<T>::size() const
{
    return nn;
}

template <class T>
void NRvector<T>::resize(int newn)
{
    if (newn != nn) {
        if (v != NULL) delete[] (v);
        nn = newn;
        v = nn > 0 ? new T[nn] : NULL;
    }
}

template <class T>
void NRvector<T>::assign(int newn, const T& a)
{
    if (newn != nn) {
        if (v != NULL) delete[] (v);
        nn = newn;
        v = nn > 0 ? new T[nn] : NULL;
    }
    for (int i=0;i<nn;i++) v[i] = a;
}

template <class T>
NRvector<T>::~NRvector()
{
    if (v != NULL) delete[] (v);
}

// end of NRvector definitions

#endif //ifdef _USESTDVECTOR_

template <class T>
class NRmatrix {
private:
    int nn;
    int mm;
    T **v;
public:
    NRmatrix();
    NRmatrix(int n, int m);         // Zero-based array
    NRmatrix(int n, int m, const T &a); //Initialize to constant
    NRmatrix(int n, int m, const T *a); // Initialize to array
    NRmatrix(const NRmatrix &rhs);      // Copy constructor
    NRmatrix & operator=(const NRmatrix &rhs);  //assignment
    typedef T value_type; // make T available externally
    inline T* operator[](const int i);  //subscripting: pointer to row i
    inline const T* operator[](const int i) const;
    inline int nrows() const;
    inline int ncols() const;
    void resize(int newn, int newm); // resize (contents not preserved)
    void assign(int newn, int newm, const T &a); // resize and assign a constant value
    ~NRmatrix();
};

template <class T>
NRmatrix<T>::NRmatrix() : nn(0), mm(0), v(NULL) {}

template <class T>
NRmatrix<T>::NRmatrix(int n, int m) : nn(n), mm(m), v(n>0 ? new T*[n] : NULL)
{
    int i,nel=m*n;
    if (v) v[0] = nel>0 ? new T[nel] : NULL;
    for (i=1;i<n;i++) v[i] = v[i-1] + m;
}

template <class T>
NRmatrix<T>::NRmatrix(int n, int m, const T &a) : nn(n), mm(m), v(n>0 ? new T*[n] : NULL)
{
    int i,j,nel=m*n;
    if (v) v[0] = nel>0 ? new T[nel] : NULL;
    for (i=1; i< n; i++) v[i] = v[i-1] + m;
    for (i=0; i< n; i++) for (j=0; j<m; j++) v[i][j] = a;
}

template <class T>
NRmatrix<T>::NRmatrix(int n, int m, const T *a) : nn(n), mm(m), v(n>0 ? new T*[n] : NULL)
{
    int i,j,nel=m*n;
    if (v) v[0] = nel>0 ? new T[nel] : NULL;
    for (i=1; i< n; i++) v[i] = v[i-1] + m;
    for (i=0; i< n; i++) for (j=0; j<m; j++) v[i][j] = *a++;
}

template <class T>
NRmatrix<T>::NRmatrix(const NRmatrix &rhs) : nn(rhs.nn), mm(rhs.mm), v(nn>0 ? new T*[nn] : NULL)
{
    int i,j,nel=mm*nn;
    if (v) v[0] = nel>0 ? new T[nel] : NULL;
    for (i=1; i< nn; i++) v[i] = v[i-1] + mm;
    for (i=0; i< nn; i++) for (j=0; j<mm; j++) v[i][j] = rhs[i][j];
}

template <class T>
NRmatrix<T> & NRmatrix<T>::operator=(const NRmatrix<T> &rhs)
// postcondition: normal assignment via copying has been performed;
//      if matrix and rhs were different sizes, matrix
//      has been resized to match the size of rhs
{
    if (this != &rhs) {
        int i,j,nel;
        if (nn != rhs.nn || mm != rhs.mm) {
            if (v != NULL) {
                delete[] (v[0]);
                delete[] (v);
            }
            nn=rhs.nn;
            mm=rhs.mm;
            v = nn>0 ? new T*[nn] : NULL;
            nel = mm*nn;
            if (v) v[0] = nel>0 ? new T[nel] : NULL;
            for (i=1; i< nn; i++) v[i] = v[i-1] + mm;
        }
        for (i=0; i< nn; i++) for (j=0; j<mm; j++) v[i][j] = rhs[i][j];
    }
    return *this;
}

template <class T>
inline T* NRmatrix<T>::operator[](const int i)  //subscripting: pointer to row i
{
#ifdef _CHECKBOUNDS_
if (i<0 || i>=nn) {
    throw("NRmatrix subscript out of bounds");
}
#endif
    return v[i];
}

template <class T>
inline const T* NRmatrix<T>::operator[](const int i) const
{
#ifdef _CHECKBOUNDS_
if (i<0 || i>=nn) {
    throw("NRmatrix subscript out of bounds");
}
#endif
    return v[i];
}

template <class T>
inline int NRmatrix<T>::nrows() const
{
    return nn;
}

template <class T>
inline int NRmatrix<T>::ncols() const
{
    return mm;
}

template <class T>
void NRmatrix<T>::resize(int newn, int newm)
{
    int i,nel;
    if (newn != nn || newm != mm) {
        if (v != NULL) {
            delete[] (v[0]);
            delete[] (v);
        }
        nn = newn;
        mm = newm;
        v = nn>0 ? new T*[nn] : NULL;
        nel = mm*nn;
        if (v) v[0] = nel>0 ? new T[nel] : NULL;
        for (i=1; i< nn; i++) v[i] = v[i-1] + mm;
    }
}

template <class T>
void NRmatrix<T>::assign(int newn, int newm, const T& a)
{
    int i,j,nel;
    if (newn != nn || newm != mm) {
        if (v != NULL) {
            delete[] (v[0]);
            delete[] (v);
        }
        nn = newn;
        mm = newm;
        v = nn>0 ? new T*[nn] : NULL;
        nel = mm*nn;
        if (v) v[0] = nel>0 ? new T[nel] : NULL;
        for (i=1; i< nn; i++) v[i] = v[i-1] + mm;
    }
    for (i=0; i< nn; i++) for (j=0; j<mm; j++) v[i][j] = a;
}

template <class T>
NRmatrix<T>::~NRmatrix()
{
    if (v != NULL) {
        delete[] (v[0]);
        delete[] (v);
    }
}

template <class T>
class NRMat3d {
private:
    int nn;
    int mm;
    int kk;
    T ***v;
public:
    NRMat3d();
    NRMat3d(int n, int m, int k);
    inline T** operator[](const int i); //subscripting: pointer to row i
    inline const T* const * operator[](const int i) const;
    inline int dim1() const;
    inline int dim2() const;
    inline int dim3() const;
    ~NRMat3d();
};

template <class T>
NRMat3d<T>::NRMat3d(): nn(0), mm(0), kk(0), v(NULL) {}

template <class T>
NRMat3d<T>::NRMat3d(int n, int m, int k) : nn(n), mm(m), kk(k), v(new T**[n])
{
    int i,j;
    v[0] = new T*[n*m];
    v[0][0] = new T[n*m*k];
    for(j=1; j<m; j++) v[0][j] = v[0][j-1] + k;
    for(i=1; i<n; i++) {
        v[i] = v[i-1] + m;
        v[i][0] = v[i-1][0] + m*k;
        for(j=1; j<m; j++) v[i][j] = v[i][j-1] + k;
    }
}

template <class T>
inline T** NRMat3d<T>::operator[](const int i) //subscripting: pointer to row i
{
    return v[i];
}

template <class T>
inline const T* const * NRMat3d<T>::operator[](const int i) const
{
    return v[i];
}

template <class T>
inline int NRMat3d<T>::dim1() const
{
    return nn;
}

template <class T>
inline int NRMat3d<T>::dim2() const
{
    return mm;
}

template <class T>
inline int NRMat3d<T>::dim3() const
{
    return kk;
}

template <class T>
NRMat3d<T>::~NRMat3d()
{
    if (v != NULL) {
        delete[] (v[0][0]);
        delete[] (v[0]);
        delete[] (v);
    }
}


// basic type names (redefine if your bit lengths don't match)

typedef int Int; // 32 bit integer
typedef unsigned int Uint;

#ifdef _MSC_VER
typedef __int64 Llong; // 64 bit integer
typedef unsigned __int64 Ullong;
#else
typedef long long int Llong; // 64 bit integer
typedef unsigned long long int Ullong;
#endif

typedef char Char; // 8 bit integer
typedef unsigned char Uchar;

typedef double Doub; // default floating type
typedef long double Ldoub;

typedef complex<double> Complex; // default complex type

typedef bool Bool;

// NaN: uncomment one of the following 3 methods of defining a global NaN
// you can test by verifying that (NaN != NaN) is true

static const Doub NaN = numeric_limits<Doub>::quiet_NaN();

//Uint proto_nan[2]={0xffffffff, 0x7fffffff};
//double NaN = *( double* )proto_nan;

//Doub NaN = sqrt(-1.);

// vector types

typedef const NRvector<Int> VecInt_I;
typedef NRvector<Int> VecInt, VecInt_O, VecInt_IO;

typedef const NRvector<Uint> VecUint_I;
typedef NRvector<Uint> VecUint, VecUint_O, VecUint_IO;

typedef const NRvector<Llong> VecLlong_I;
typedef NRvector<Llong> VecLlong, VecLlong_O, VecLlong_IO;

typedef const NRvector<Ullong> VecUllong_I;
typedef NRvector<Ullong> VecUllong, VecUllong_O, VecUllong_IO;

typedef const NRvector<Char> VecChar_I;
typedef NRvector<Char> VecChar, VecChar_O, VecChar_IO;

typedef const NRvector<Char*> VecCharp_I;
typedef NRvector<Char*> VecCharp, VecCharp_O, VecCharp_IO;

typedef const NRvector<Uchar> VecUchar_I;
typedef NRvector<Uchar> VecUchar, VecUchar_O, VecUchar_IO;

typedef const NRvector<Doub> VecDoub_I;
typedef NRvector<Doub> VecDoub, VecDoub_O, VecDoub_IO;

typedef const NRvector<Doub*> VecDoubp_I;
typedef NRvector<Doub*> VecDoubp, VecDoubp_O, VecDoubp_IO;

typedef const NRvector<Complex> VecComplex_I;
typedef NRvector<Complex> VecComplex, VecComplex_O, VecComplex_IO;

typedef const NRvector<Bool> VecBool_I;
typedef NRvector<Bool> VecBool, VecBool_O, VecBool_IO;

// matrix types

typedef const NRmatrix<Int> MatInt_I;
typedef NRmatrix<Int> MatInt, MatInt_O, MatInt_IO;

typedef const NRmatrix<Uint> MatUint_I;
typedef NRmatrix<Uint> MatUint, MatUint_O, MatUint_IO;

typedef const NRmatrix<Llong> MatLlong_I;
typedef NRmatrix<Llong> MatLlong, MatLlong_O, MatLlong_IO;

typedef const NRmatrix<Ullong> MatUllong_I;
typedef NRmatrix<Ullong> MatUllong, MatUllong_O, MatUllong_IO;

typedef const NRmatrix<Char> MatChar_I;
typedef NRmatrix<Char> MatChar, MatChar_O, MatChar_IO;

typedef const NRmatrix<Uchar> MatUchar_I;
typedef NRmatrix<Uchar> MatUchar, MatUchar_O, MatUchar_IO;

typedef const NRmatrix<Doub> MatDoub_I;
typedef NRmatrix<Doub> MatDoub, MatDoub_O, MatDoub_IO;

typedef const NRmatrix<Bool> MatBool_I;
typedef NRmatrix<Bool> MatBool, MatBool_O, MatBool_IO;

// 3D matrix types

typedef const NRMat3d<Doub> Mat3DDoub_I;
typedef NRMat3d<Doub> Mat3DDoub, Mat3DDoub_O, Mat3DDoub_IO;

// Floating Point Exceptions for Microsoft compilers

#ifdef _TURNONFPES_
#ifdef _MSC_VER
struct turn_on_floating_exceptions {
    turn_on_floating_exceptions() {
        int cw = _controlfp( 0, 0 );
        cw &=~(EM_INVALID | EM_OVERFLOW | EM_ZERODIVIDE );
        _controlfp( cw, MCW_EM );
    }
};
turn_on_floating_exceptions yes_turn_on_floating_exceptions;
#endif /* _MSC_VER */
#endif /* _TURNONFPES */

#endif /* _NR3_H_ */

// TODO
/*******************************************************************************
 * Copyright (c) 2011 IBM Corporation and others.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *     IBM Corporation - initial API and implementation
 *******************************************************************************/
struct LUdcmp
{
  MatDoub_I &aref;
    Int n;
    MatDoub lu;
    VecInt indx;
    Doub d;
    LUdcmp(MatDoub_I &a);
    void solve(VecDoub_I &b, VecDoub_O &x);
    void solve(MatDoub_I &b, MatDoub_O &x);
    void inverse(MatDoub_O &ainv);
    Doub det();
    void mprove(VecDoub_I &b, VecDoub_IO &x);
};
LUdcmp::LUdcmp(MatDoub_I &a) : aref(a), n(a.nrows()), lu(a), indx(n) {
    const Doub TINY=1.0e-40;
    Int i,imax,j,k;
    Doub big,temp;
    VecDoub vv(n);
    d=1.0;
    for (i=0;i<n;i++) {
        big=0.0;
        for (j=0;j<n;j++)
            if ((temp=abs(lu[i][j])) > big) big=temp;
        if (big == 0.0) throw("Singular matrix in LUdcmp");
        vv[i]=1.0/big;
    }
    for (k=0;k<n;k++) {
        big=0.0;
        for (i=k;i<n;i++) {
            temp=vv[i]*abs(lu[i][k]);
            if (temp > big) {
                big=temp;
                imax=i;
            }
        }
        if (k != imax) {
            for (j=0;j<n;j++) {
                temp=lu[imax][j];
                lu[imax][j]=lu[k][j];
                lu[k][j]=temp;
            }
            d = -d;
            vv[imax]=vv[k];
        }
        indx[k]=imax;
        if (lu[k][k] == 0.0) lu[k][k]=TINY;
        for (i=k+1;i<n;i++) {
            temp=lu[i][k] /= lu[k][k];
            for (j=k+1;j<n;j++)
                lu[i][j] -= temp*lu[k][j];
        }
    }
}
void LUdcmp::solve(VecDoub_I &b, VecDoub_O &x)
{
    Int i,ii=0,ip,j;
    Doub sum;
    if (b.size() != n || x.size() != n)
        throw("LUdcmp::solve bad sizes");
    for (i=0;i<n;i++) x[i] = b[i];
    for (i=0;i<n;i++) {
        ip=indx[i];
        sum=x[ip];
        x[ip]=x[i];
        if (ii != 0)
            for (j=ii-1;j<i;j++) sum -= lu[i][j]*x[j];
        else if (sum != 0.0)
            ii=i+1;
        x[i]=sum;
    }
    for (i=n-1;i>=0;i--) {
        sum=x[i];
        for (j=i+1;j<n;j++) sum -= lu[i][j]*x[j];
        x[i]=sum/lu[i][i];
    }
}

void LUdcmp::solve(MatDoub_I &b, MatDoub_O &x)
{
    int i,j,m=b.ncols();
    if (b.nrows() != n || x.nrows() != n || b.ncols() != x.ncols())
        throw("LUdcmp::solve bad sizes");
    VecDoub xx(n);
    for (j=0;j<m;j++) {
        for (i=0;i<n;i++) xx[i] = b[i][j];
        solve(xx,xx);
        for (i=0;i<n;i++) x[i][j] = xx[i];
    }
}
void LUdcmp::inverse(MatDoub_O &ainv)
{
    Int i,j;
    ainv.resize(n,n);
    for (i=0;i<n;i++) {
        for (j=0;j<n;j++) ainv[i][j] = 0.;
        ainv[i][i] = 1.;
    }
    solve(ainv,ainv);
}
Doub LUdcmp::det()
{
    Doub dd = d;
    for (Int i=0;i<n;i++) dd *= lu[i][i];
    return dd;
}
void LUdcmp::mprove(VecDoub_I &b, VecDoub_IO &x)
{
    Int i,j;
    VecDoub r(n);
    for (i=0;i<n;i++) {
        Ldoub sdp = -b[i];
        for (j=0;j<n;j++)
            sdp += (Ldoub)aref[i][j] * (Ldoub)x[j];
        r[i]=sdp;
    }
    solve(r,r);
    for (i=0;i<n;i++) x[i] -= r[i];
}


// TODO
struct QRdcmp {
    Int n;
    MatDoub qt, r;
    Bool sing;
    QRdcmp(MatDoub_I &a);
    void solve(VecDoub_I &b, VecDoub_O &x);
    void qtmult(VecDoub_I &b, VecDoub_O &x);
    void rsolve(VecDoub_I &b, VecDoub_O &x);
    void update(VecDoub_I &u, VecDoub_I &v);
    void rotate(const Int i, const Doub a, const Doub b);
};
QRdcmp::QRdcmp(MatDoub_I &a)
    : n(a.nrows()), qt(n,n), r(a), sing(false) {
    Int i,j,k;
    VecDoub c(n), d(n);
    Doub scale,sigma,sum,tau;
    for (k=0;k<n-1;k++) {
        scale=0.0;
        for (i=k;i<n;i++) scale=MAX(scale,abs(r[i][k]));
        if (scale == 0.0) {
            sing=true;
            c[k]=d[k]=0.0;
        } else {
            for (i=k;i<n;i++) r[i][k] /= scale;
            for (sum=0.0,i=k;i<n;i++) sum += SQR(r[i][k]);
            sigma=SIGN(sqrt(sum),r[k][k]);
            r[k][k] += sigma;
            c[k]=sigma*r[k][k];
            d[k] = -scale*sigma;
            for (j=k+1;j<n;j++) {
                for (sum=0.0,i=k;i<n;i++) sum += r[i][k]*r[i][j];
                tau=sum/c[k];
                for (i=k;i<n;i++) r[i][j] -= tau*r[i][k];
            }
        }
    }
    d[n-1]=r[n-1][n-1];
    if (d[n-1] == 0.0) sing=true;
    for (i=0;i<n;i++) {
        for (j=0;j<n;j++) qt[i][j]=0.0;
        qt[i][i]=1.0;
    }
    for (k=0;k<n-1;k++) {
        if (c[k] != 0.0) {
            for (j=0;j<n;j++) {
                sum=0.0;
                for (i=k;i<n;i++)
                    sum += r[i][k]*qt[i][j];
                sum /= c[k];
                for (i=k;i<n;i++)
                    qt[i][j] -= sum*r[i][k];
            }
        }
    }
    for (i=0;i<n;i++) {
        r[i][i]=d[i];
        for (j=0;j<i;j++) r[i][j]=0.0;
    }
}
void QRdcmp::solve(VecDoub_I &b, VecDoub_O &x) {
    qtmult(b,x);
    rsolve(x,x);
}

void QRdcmp::qtmult(VecDoub_I &b, VecDoub_O &x) {
    Int i,j;
    Doub sum;
    for (i=0;i<n;i++) {
        sum = 0.;
        for (j=0;j<n;j++) sum += qt[i][j]*b[j];
        x[i] = sum;
    }
}

void QRdcmp::rsolve(VecDoub_I &b, VecDoub_O &x) {
    Int i,j;
    Doub sum;
    if (sing) throw("attempting solve in a singular QR");
    for (i=n-1;i>=0;i--) {
        sum=b[i];
        for (j=i+1;j<n;j++) sum -= r[i][j]*x[j];
        x[i]=sum/r[i][i];
    }
}
void QRdcmp::update(VecDoub_I &u, VecDoub_I &v) {
    Int i,k;
    VecDoub w(u);
    for (k=n-1;k>=0;k--)
        if (w[k] != 0.0) break;
    if (k < 0) k=0;
    for (i=k-1;i>=0;i--) {
        rotate(i,w[i],-w[i+1]);
        if (w[i] == 0.0)
            w[i]=abs(w[i+1]);
        else if (abs(w[i]) > abs(w[i+1]))
            w[i]=abs(w[i])*sqrt(1.0+SQR(w[i+1]/w[i]));
        else w[i]=abs(w[i+1])*sqrt(1.0+SQR(w[i]/w[i+1]));
    }
    for (i=0;i<n;i++) r[0][i] += w[0]*v[i];
    for (i=0;i<k;i++)
        rotate(i,r[i][i],-r[i+1][i]);
    for (i=0;i<n;i++)
        if (r[i][i] == 0.0) sing=true;
}

void QRdcmp::rotate(const Int i, const Doub a, const Doub b)
{
    Int j;
    Doub c,fact,s,w,y;
    if (a == 0.0) {
        c=0.0;
        s=(b >= 0.0 ? 1.0 : -1.0);
    } else if (abs(a) > abs(b)) {
        fact=b/a;
        c=SIGN(1.0/sqrt(1.0+(fact*fact)),a);
        s=fact*c;
    } else {
        fact=a/b;
        s=SIGN(1.0/sqrt(1.0+(fact*fact)),b);
        c=fact*s;
    }
    for (j=i;j<n;j++) {
        y=r[i][j];
        w=r[i+1][j];
        r[i][j]=c*y-s*w;
        r[i+1][j]=s*y+c*w;
    }
    for (j=0;j<n;j++) {
        y=qt[i][j];
        w=qt[i+1][j];
        qt[i][j]=c*y-s*w;
        qt[i+1][j]=s*y+c*w;
    }
}


// TODO
template <class T>
void lnsrch(VecDoub_I &xold, const Doub fold, VecDoub_I &g, VecDoub_IO &p,
VecDoub_O &x, Doub &f, const Doub stpmax, Bool &check, T &func) {
    const Doub ALF=1.0e-4, TOLX=numeric_limits<Doub>::epsilon();
    Doub a,alam,alam2=0.0,alamin,b,disc,f2=0.0;
    Doub rhs1,rhs2,slope=0.0,sum=0.0,temp,test,tmplam;
    Int i,n=xold.size();
    check=false;
    for (i=0;i<n;i++) sum += p[i]*p[i];
    sum=sqrt(sum);
    if (sum > stpmax)
        for (i=0;i<n;i++)
            p[i] *= stpmax/sum;
    for (i=0;i<n;i++)
        slope += g[i]*p[i];
    if (slope >= 0.0) throw("Roundoff problem in lnsrch.");
    test=0.0;
    for (i=0;i<n;i++) {
        temp=abs(p[i])/MAX(abs(xold[i]),1.0);
        if (temp > test) test=temp;
    }
    alamin=TOLX/test;
    alam=1.0;
    for (;;) {
        for (i=0;i<n;i++) x[i]=xold[i]+alam*p[i];
        f=func(x);
        if (alam < alamin) {
            for (i=0;i<n;i++) x[i]=xold[i];
            check=true;
            return;
        } else if (f <= fold+ALF*alam*slope) return;
        else {
            if (alam == 1.0)
                tmplam = -slope/(2.0*(f-fold-slope));
            else {
                rhs1=f-fold-alam*slope;
                rhs2=f2-fold-alam2*slope;
                a=(rhs1/(alam*alam)-rhs2/(alam2*alam2))/(alam-alam2);
                b=(-alam2*rhs1/(alam*alam)+alam*rhs2/(alam2*alam2))/(alam-alam2);
                if (a == 0.0) tmplam = -slope/(2.0*b);
                else {
                    disc=b*b-3.0*a*slope;
                    if (disc < 0.0) tmplam=0.5*alam;
                    else if (b <= 0.0) tmplam=(-b+sqrt(disc))/(3.0*a);
                    else tmplam=-slope/(b+sqrt(disc));
                }
                if (tmplam>0.5*alam)
                    tmplam=0.5*alam;
            }
        }
        alam2=alam;
        f2 = f;
        alam=MAX(tmplam,0.1*alam);
    }
}
template <class T>
struct NRfdjac {
    const Doub EPS;
    T &func;
    NRfdjac(T &funcc) : EPS(1.0e-8),func(funcc) {}
    MatDoub operator() (VecDoub_I &x, VecDoub_I &fvec) {
        Int n=x.size();
        MatDoub df(n,n);
        VecDoub xh=x;
        for (Int j=0;j<n;j++) {
            Doub temp=xh[j];
            Doub h=EPS*abs(temp);
            if (h == 0.0) h=EPS;
            xh[j]=temp+h;
            h=xh[j]-temp;
            VecDoub f=func(xh);
            xh[j]=temp;
            for (Int i=0;i<n;i++)
                df[i][j]=(f[i]-fvec[i])/h;
        }
        return df;
    }
};
template <class T>
struct NRfmin {
    VecDoub fvec;
    T &func;
    Int n;
    NRfmin(T &funcc) : func(funcc){}
    Doub operator() (VecDoub_I &x) {
        n=x.size();
        Doub sum=0;
        fvec=func(x);
        for (Int i=0;i<n;i++) sum += SQR(fvec[i]);
        return 0.5*sum;
    }
};
template <class T>
void newt(VecDoub_IO &x, Bool &check, T &vecfunc) {
    const Int MAXITS=200;
    const Doub TOLF=1.0e-8,TOLMIN=1.0e-12,STPMX=100.0;
    const Doub TOLX=numeric_limits<Doub>::epsilon();
    Int i,j,its,n=x.size();
    Doub den,f,fold,stpmax,sum,temp,test;
    VecDoub g(n),p(n),xold(n);
    MatDoub fjac(n,n);
    NRfmin<T> fmin(vecfunc);
    NRfdjac<T> fdjac(vecfunc);
    VecDoub &fvec=fmin.fvec;
    f=fmin(x);
    test=0.0;
    for (i=0;i<n;i++)
        if (abs(fvec[i]) > test) test=abs(fvec[i]);
    if (test < 0.01*TOLF) {
        check=false;
        return;
    }
    sum=0.0;
    for (i=0;i<n;i++) sum += SQR(x[i]);
    stpmax=STPMX*MAX(sqrt(sum),Doub(n));
    for (its=0;its<MAXITS;its++) {
        fjac=fdjac(x,fvec);
        for (i=0;i<n;i++) {
            sum=0.0;
            for (j=0;j<n;j++) sum += fjac[j][i]*fvec[j];
            g[i]=sum;
        }
        for (i=0;i<n;i++) xold[i]=x[i];
        fold=f;
        for (i=0;i<n;i++) p[i] = -fvec[i];
        LUdcmp alu(fjac);
        alu.solve(p,p);
        lnsrch(xold,fold,g,p,x,f,stpmax,check,fmin);
        test=0.0;
        for (i=0;i<n;i++)
            if (abs(fvec[i]) > test) test=abs(fvec[i]);
        if (test < TOLF) {
            check=false;
            return;
        }
        if (check) {
            test=0.0;
            den=MAX(f,0.5*n);
            for (i=0;i<n;i++) {
                temp=abs(g[i])*MAX(abs(x[i]),1.0)/den;
                if (temp > test) test=temp;
            }
            check=(test < TOLMIN);
            return;
        }
        test=0.0;
        for (i=0;i<n;i++) {
            temp=(abs(x[i]-xold[i]))/MAX(abs(x[i]),1.0);
            if (temp > test) test=temp;
        }
        if (test < TOLX)
            return;
    }
    throw("MAXITS exceeded in newt");
}
template <class T>
void broydn(VecDoub_IO &x, Bool &check, T &vecfunc) {
    const Int MAXITS=200;
    const Doub EPS=numeric_limits<Doub>::epsilon();
    const Doub TOLF=1.0e-8, TOLX=EPS, STPMX=100.0, TOLMIN=1.0e-12;
    Bool restrt,skip;
    Int i,its,j,n=x.size();
    Doub den,f,fold,stpmax,sum,temp,test;
    VecDoub fvcold(n),g(n),p(n),s(n),t(n),w(n),xold(n);
    QRdcmp *qr;
    NRfmin<T> fmin(vecfunc);
    NRfdjac<T> fdjac(vecfunc);
    VecDoub &fvec=fmin.fvec;
    f=fmin(x);
    test=0.0;
    for (i=0;i<n;i++)
        if (abs(fvec[i]) > test) test=abs(fvec[i]);
    if (test < 0.01*TOLF) {
        check=false;
        return;
    }
    for (sum=0.0,i=0;i<n;i++) sum += SQR(x[i]);
    stpmax=STPMX*MAX(sqrt(sum),Doub(n));
    restrt=true;
    for (its=1;its<=MAXITS;its++) {
        if (restrt) {
            qr=new QRdcmp(fdjac(x,fvec));
            if (qr->sing) throw("singular Jacobian in broydn");
        } else {
            for (i=0;i<n;i++) s[i]=x[i]-xold[i];
            for (i=0;i<n;i++) {
                for (sum=0.0,j=i;j<n;j++) sum += qr->r[i][j]*s[j];
                t[i]=sum;
            }
            skip=true;
            for (i=0;i<n;i++) {
                for (sum=0.0,j=0;j<n;j++) sum += qr->qt[j][i]*t[j];
                w[i]=fvec[i]-fvcold[i]-sum;
                if (abs(w[i]) >= EPS*(abs(fvec[i])+abs(fvcold[i]))) skip=false;
                else w[i]=0.0;
            }
            if (!skip) {
                qr->qtmult(w,t);
                for (den=0.0,i=0;i<n;i++) den += SQR(s[i]);
                for (i=0;i<n;i++) s[i] /= den;
                qr->update(t,s);
                if (qr->sing) throw("singular update in broydn");
            }
        }
        qr->qtmult(fvec,p);
        for (i=0;i<n;i++)
            p[i] = -p[i];
        for (i=n-1;i>=0;i--) {
            for (sum=0.0,j=0;j<=i;j++) sum -= qr->r[j][i]*p[j];
            g[i]=sum;
        }
        for (i=0;i<n;i++) {
            xold[i]=x[i];
            fvcold[i]=fvec[i];
        }
        fold=f;
        qr->rsolve(p,p);
        lnsrch(xold,fold,g,p,x,f,stpmax,check,fmin);
        test=0.0;
        for (i=0;i<n;i++)
            if (abs(fvec[i]) > test) test=abs(fvec[i]);
        if (test < TOLF) {
            check=false;
            delete qr;
            return;
        }
        if (check) {
            if (restrt) {
                delete qr;
                return;
            } else {
                test=0.0;
                den=MAX(f,0.5*n);
                for (i=0;i<n;i++) {
                    temp=abs(g[i])*MAX(abs(x[i]),1.0)/den;
                    if (temp > test) test=temp;
                }
                if (test < TOLMIN) {
                    delete qr;
                    return;
                }
                else restrt=true;
            }
        } else {
            restrt=false;
            test=0.0;
            for (i=0;i<n;i++) {
                temp=(abs(x[i]-xold[i]))/MAX(abs(x[i]),1.0);
                if (temp > test) test=temp;
            }
            if (test < TOLX) {
                delete qr;
                return;
            }
        }
    }
    throw("MAXITS exceeded in broydn");
}


// TODO
struct Quadrature{
    Int n;
    virtual Doub next() = 0;
};
template<class T>
struct Trapzd : Quadrature {
    Doub a,b,s;
    T &func;
    Trapzd() {};
    Trapzd(T &funcc, const Doub aa, const Doub bb) :
        func(funcc), a(aa), b(bb) {n=0;}
    Doub next() {
        Doub x,tnm,sum,del;
        Int it,j;
        n++;
        if (n == 1) {
            return (s=0.5*(b-a)*(func(a)+func(b)));
        } else {
            for (it=1,j=1;j<n-1;j++) it <<= 1;
            tnm=it;
            del=(b-a)/tnm;
            x=a+0.5*del;
            for (sum=0.0,j=0;j<it;j++,x+=del) sum += func(x);
            s=0.5*(s+(b-a)*sum/tnm);
            return s;
        }
    }
};
template<class T>
Doub qtrap(T &func, const Doub a, const Doub b, const Doub eps=1.0e-10) {
    const Int JMAX=20;
    Doub s,olds=0.0;
    Trapzd<T> t(func,a,b);
    for (Int j=0;j<JMAX;j++) {
        s=t.next();
        if (j > 5)
            if (abs(s-olds) < eps*abs(olds) ||
                (s == 0.0 && olds == 0.0)) return s;
        olds=s;
    }
    throw("Too many steps in routine qtrap");
}
template<class T>
Doub qsimp(T &func, const Doub a, const Doub b, const Doub eps=1.0e-10) {
    const Int JMAX=20;
    Doub s,st,ost=0.0,os=0.0;
    Trapzd<T> t(func,a,b);
    for (Int j=0;j<JMAX;j++) {
        st=t.next();
        s=(4.0*st-ost)/3.0;
        if (j > 5)
            if (abs(s-os) < eps*abs(os) ||
                (s == 0.0 && os == 0.0)) return s;
        os=s;
        ost=st;
    }
    throw("Too many steps in routine qsimp");
}
template <class T>
struct Midpnt : Quadrature {
    Doub a,b,s;
    T &funk;
    Midpnt(T &funcc, const Doub aa, const Doub bb) :
        funk(funcc), a(aa), b(bb) {n=0;}
    Doub next(){
        Int it,j;
        Doub x,tnm,sum,del,ddel;
        n++;
        if (n == 1) {
            return (s=(b-a)*func(0.5*(a+b)));
        } else {
            for(it=1,j=1;j<n-1;j++) it *= 3;
            tnm=it;
            del=(b-a)/(3.0*tnm);
            ddel=del+del;
            x=a+0.5*del;
            sum=0.0;
            for (j=0;j<it;j++) {
                sum += func(x);
                x += ddel;
                sum += func(x);
                x += del;
            }
            s=(s+(b-a)*sum/tnm)/3.0;
            return s;
        }
    }
    virtual Doub func(const Doub x) {return funk(x);}
};
template <class T>
struct Midinf : Midpnt<T>{
    Doub func(const Doub x) {
        return Midpnt<T>::funk(1.0/x)/(x*x);
    }
    Midinf(T &funcc, const Doub aa, const Doub bb) :
        Midpnt<T>(funcc, aa, bb) {
        Midpnt<T>::a=1.0/bb;
        Midpnt<T>::b=1.0/aa;
    }
};
template <class T>
struct Midsql : Midpnt<T>{
    Doub aorig;
    Doub func(const Doub x) {
        return 2.0*x*Midpnt<T>::funk(aorig+x*x);
    }
    Midsql(T &funcc, const Doub aa, const Doub bb) :
        Midpnt<T>(funcc, aa, bb), aorig(aa) {
        Midpnt<T>::a=0;
        Midpnt<T>::b=sqrt(bb-aa);
    }
};
template <class T>
struct Midsqu : Midpnt<T>{
    Doub borig;
    Doub func(const Doub x) {
        return 2.0*x*Midpnt<T>::funk(borig-x*x);
    }
    Midsqu(T &funcc, const Doub aa, const Doub bb) :
        Midpnt<T>(funcc, aa, bb), borig(bb) {
        Midpnt<T>::a=0;
        Midpnt<T>::b=sqrt(bb-aa);
    }
};
template <class T>
struct Midexp : Midpnt<T>{
    Doub func(const Doub x) {
        return Midpnt<T>::funk(-log(x))/x;
    }
    Midexp(T &funcc, const Doub aa, const Doub bb) :
        Midpnt<T>(funcc, aa, bb) {
        Midpnt<T>::a=0.0;
        Midpnt<T>::b=exp(-aa);
    }
};
