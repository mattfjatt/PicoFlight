#include "headers/LinAlg.h"

void LinAlg_zeromat(int N, int M,double A[N][M]){
    for(int i = 0; i < N; i++){
        for(int j = 0; j < M; j++){
            A[i][j]= 0;
        }
    }
}

void LinAlg_zerovec(int N, double x[N]){
    for(int i = 0; i < N; i++){
        x[i] = 0;
    }
}

void LinAlg_eye(int N,double A[N][N])
{
    LinAlg_zeromat(N, N, A);
    for(int i = 0; i < N; i++){
        A[i][i]= 1;
    }
}

void LinAlg_matvecmul(int N, int M, double A[N][M], double x[M], double b[N])
{
    if(N == M){
        double temp[N];
        LinAlg_zerovec(N,temp); //Found no cheaper way to do this than to create an entire temp vector
        for(int i = 0; i < N; i++){
            for(int j = 0; j < M; j++){
                temp[i] += A[i][j]*x[j];
            }
        }
        LinAlg_veccopy(N,temp,b);
    }else{
        if(x == b){
            printf("Error: Input and output vector can not be the same address in memory for N != M, aborting\n");
            return;
        }
        LinAlg_zerovec(N,b);
        for(int i = 0; i < N; i++){
            for(int j = 0; j < M; j++){
                b[i] += A[i][j]*x[j];
            }
        }
    }
}

void LinAlg_matmatmul_small(int N, int M, double A[N][M], double B[M][N], double C[N][N]){
    //A e NxM
    //B e MxN
    //C e NxN
    //Creating a temp matrix on the stack is a problem if N is very large, but even the Jacobian matrices of
    //J e 1000x9 and JT e 9x1000 will only yield JTJ e 9x9 when multiplied, this is 648 bytes, and the stack is 8kB 
    //But this is unnecessary as C will not be an alias anyway when N != M
    double temp[N][N];
    LinAlg_zeromat(N, N, temp); //temp set to be all zeroes
    for(int i = 0; i < N; i++){
        for(int j = 0; j < N; j++){
            for(int k = 0; k < M; k++){
                temp[i][j] += A[i][k]*B[k][j]; //Some inefficiency with indexing I need to consider here
            }
        }
    }
    LinAlg_matcopy(N, N, temp, C);
}

void LinAlg_vecvecadd(int N, double a[N], double b[N], double c[N]){
    for(int i = 0; i < N; i++){
        c[i] = a[i] + b[i];
    }
}

void LinAlg_vecvecsub(int N, double a[N], double b[N], double c[N]){
    for(int i = 0; i < N; i++){
        c[i] = a[i] - b[i];
    }
}

void LinAlg_vecscalmult(int N, double x[N], double y[N], double k){
    for(int i = 0; i < N; i++){
        y[i] = k*x[i];
    }
}

double LinAlg_vecdot(int N, double a[N], double b[N]){
    double c = 0;
    for(int i = 0; i < N; i++){
        c += a[i]*b[i];
    }
    return c;
}

void LinAlg_matmatadd(int N, int M, double A[N][M], double B[N][M], double C[N][M]){
    for(int i = 0; i < N; i++){
        for(int j = 0; j < M; j++){
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

void LinAlg_matmatsub(int N, int M, double A[N][M], double B[N][M], double C[N][M]){
    for(int i = 0; i < N; i++){
        for(int j = 0; j < M; j++){
            C[i][j] = A[i][j] - B[i][j];
        }
    }
}

void LinAlg_matscalmult(int N, int M, double A[N][M], double k, double C[N][M])
{
    for(int i = 0; i < N; i++){
        for(int j = 0; j < M; j++){
            C[i][j] = k*A[i][j];
        }
    }
}

void LinAlg_mat2colvecs3x3(double R[][3], double r1[3], double r2[3], double r3[3]){
    for(int i = 0; i < 3; i++){
        r1[i] = R[i][0];
        r2[i] = R[i][1];
        r3[i] = R[i][2];
    }
}

void LinAlg_colvecs2mat3x3(double R[][3], double r1[3], double r2[3], double r3[3]){
    for(int i = 0; i < 3; i++){
        R[i][0] = r1[i];
        R[i][1] = r2[i];
        R[i][2] = r3[i];
    }
}

void LinAlg_matnormalizerotation(double R[][3]){
    //Crude method of ensuring R remains in SO(3)
    int N = 3;
    double r1[3], r2[3], r3[3];
    double u1[3], u2[3], u3[3];
    double k1, k2, k3;
    double k1u1[3], k2u1[3], k3u2[3];
    LinAlg_mat2colvecs3x3(R, r1, r2, r3);
    //u1 = r1;
    LinAlg_veccopy(N,r1,u1);

    //u2 = r2 - k1*u1
    //k1 = (r2*u1)/(u1*u1)
    k1 = LinAlg_vecdot(N,r2,u1)/LinAlg_vecdot(N,u1,u1);
    LinAlg_vecscalmult(N,u1,k1u1, k1);
    LinAlg_vecvecsub(N,r2,k1u1,u2);

    //u3 = r3 - (r3*u1)/(u1*u1)*u1 - (r3*u2)/(u2*u2)*u2
    //u3 = r3 - k2*u1 - k3*u2;
    //k2 = (r3*u1)/(u1*u1)
    //k3 = (r3*u2)/(u2*u2)
    k2 = LinAlg_vecdot(N,r3,u1)/LinAlg_vecdot(N,u1,u1);
    k3 = LinAlg_vecdot(N,r3,u2)/LinAlg_vecdot(N,u2,u2);
    LinAlg_vecscalmult(N,u1,k2u1, k2);
    LinAlg_vecscalmult(N,u2,k3u2, k3);
    LinAlg_veccopy(N,r3,u3);
    LinAlg_vecvecsub(N,u3,k2u1,u3);
    LinAlg_vecvecsub(N,u3,k3u2,u3);

    //Normalize the orthogonal vectors 
    LinAlg_normalize(N,u1,u1);
    LinAlg_normalize(N,u2,u2);
    LinAlg_normalize(N,u3,u3);
    
    LinAlg_colvecs2mat3x3(R,u1,u2,u3);

}

void LinAlg_vec2skew3x3(double x[3], double S[][3]){
    S[0][0] = 0;
    S[1][1] = 0;
    S[2][2] = 0;

    S[0][1] = -x[2];
    S[0][2] =  x[1];
    S[1][2] = -x[0];
    S[1][0] =  x[2];
    S[2][0] = -x[1];
    S[2][1] =  x[0];
}

void LinAlg_matcopy(int N, int M, const double A[N][M], double B[N][M]){
    for(int i = 0; i < N; i++){
        for(int j = 0; j < M; j++){
            B[i][j] = A[i][j];
        }
    }
}

void LinAlg_veccopy(int N, double a[N], double b[N]){
    for(int i = 0; i < N; i++){
        b[i] = a[i];
    }
}

void LinAlg_mattranspose(int N, int M, double A[N][M], double AT[M][N])
{
    if(N == M){
        for(int i = 0; i < N; i++){
            for(int j = i; j < M; j++){
                double tmp = A[i][j]; 
                AT[i][j] = A[j][i];
                AT[j][i] = tmp;
            }
        }
    }else{
        if(A == AT){
            printf("Error: in-place non-square matrix transpose attempted, this is not possible, aborting\n");
            return;
        }else{
            //A and AT are separate variables and a tmp variable is not required
            for(int i = 0; i < N; i++){
                for(int j = 0; j < M; j++){
                    AT[j][i] = A[i][j];
                }
            }
        }
    }
}

double LinAlg_vecnorm(int N, double x[N]){
    double norm = 0;
    for(int i = 0; i < N; i++){
        norm += x[i]*x[i];
    }
    return sqrt(norm);
}

void LinAlg_normalize(int N, double x[N], double x_norm[N]){
    double size = LinAlg_vecnorm(N,x);
    for(int i = 0; i < N; i++){
        x_norm[i] = x[i]/size;
    }
}

int LinAlg_factorial(int k){
    int ret = 1;
    if(k > 0){
        for(int i = 1; i <= k; i++){
            ret *= i;
        }
    }else if (k < 0){
        printf("Input to factorial() below 0\n");
        ret = -1;
    }
    return ret;
}

void LinAlg_expm3x3(double A[][3], double eA[][3], int k){
    int N = 3;
    LinAlg_eye(N,eA);
    double B[3][3];
    LinAlg_matcopy(N,N,A,B);
    double Bs[3][3]; //scaled B
    double scale = 0;
    for(int i = 1; i <= k; i++){
        scale = 1.0/LinAlg_factorial(i);
        LinAlg_matscalmult(N,N,B,scale,Bs); //Bs = scale*B;
        LinAlg_matmatadd(N,N,eA,Bs,eA);
        LinAlg_matmatmul_small(N,N,B,A,B);
    }
}

double LinAlg_det3x3(double A[][3]){
    double ret = A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1]);
    ret = ret - A[0][1]*(A[1][0]*A[2][2] - A[2][0]*A[1][2]);
    ret = ret + A[0][2]*(A[1][0]*A[2][1] - A[2][0]*A[1][1]);
    return ret;
}

void LinAlg_rotX(double t, double R[][3]){
    LinAlg_zeromat(3,3,R);
    R[0][0] = 1;
    R[1][1] = cos(t);
    R[2][2] = cos(t);
    R[2][1] = sin(t);
    R[1][2] = -sin(t);
}

void LinAlg_rotY(double t, double R[][3]){
    LinAlg_zeromat(3,3,R);
    R[1][1] = 1;
    R[0][0] = cos(t);
    R[2][2] = cos(t);
    R[0][2] = sin(t);
    R[2][0] = -sin(t);
}

void LinAlg_rotZ(double t, double R[][3]){
    LinAlg_zeromat(3,3,R);
    R[2][2] = 1;
    R[0][0] = cos(t);
    R[1][1] = cos(t);
    R[1][0] = sin(t);
    R[0][1] = -sin(t);
}

void LinAlg_printscal(double a){
    printf("%.8f\n",a);
}

void LinAlg_printvec(int N, double v[N]){
    for(int i = 0; i < N; i++){
        printf("[%.8f]\n",v[i]);
    }
    printf("\n");
}

void LinAlg_printmat(int N, int M, const double A[N][M]){
    for(int i = 0; i < N; i++){
        printf("[");
        for(int j = 0; j < M; j++){
            if(j < 2){
                printf(" %.3f,",A[i][j]);
            }else{
               printf(" %.3f",A[i][j]); 
            }
        }
        printf("]\n");
    }
    printf("\n");
}

void LinAlg_printvec_comma_separated(double v[3]){
    for(int i = 0; i < 2; i++){
        printf("%.8f,",v[i]);
    }
    printf("%.8f",v[2]);
    printf("\n");
}
