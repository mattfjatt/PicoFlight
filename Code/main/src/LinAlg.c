#include "headers/LinAlg.h"

void LinAlg_zeromat(float A[][3]){
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            A[i][j]= 0;
        }
    }
}

void LinAlg_zerovec(float x[3]){
    for(int i = 0; i < 3; i++){
        x[i] = 0;
    }
}

void LinAlg_eye(float A[][3]){
    LinAlg_zeromat(A);
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            if(i == j){
                A[i][j]= 1;
            }
        }
    }
}

void LinAlg_matvecmul(float A[][3], float x[3], float b[3]){
    //A: double array, x: single array
    float temp[3]; //Just use b instead of temp?
    LinAlg_zerovec(temp); //Set temp = [0 0 0]
    //printvec(temp);
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            temp[i] += A[i][j]*x[j];
        }
    }
    LinAlg_veccopy(temp,b);
}

void LinAlg_matmatmul(float A[][3], float B[][3], float C[][3]){
    float temp[3][3];
    LinAlg_zeromat(temp); //temp set to be all zeroes
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 3; k++){
                temp[i][j] += A[i][k]*B[k][j];
            }
        }
    }
    LinAlg_matcopy(temp,C);
}

void LinAlg_vecvecadd(float a[3], float b[3], float c[3]){
    for(int i = 0; i < 3; i++){
        c[i] = a[i] + b[i];
    }
};

void LinAlg_vecvecsub(float a[3], float b[3],float c[3]){
    for(int i = 0; i < 3; i++){
        c[i] = a[i] - b[i];
    }
};

void LinAlg_vecscalmult(float x[3], float y[3], float k){
    for(int i = 0; i < 3; i++){
        y[i] = k*x[i];
    }
}

float LinAlg_vecdot(float a[3], float b[3]){
    float c = 0;
    for(int i = 0; i < 3; i++){
        c += a[i]*b[i];
    }
    return c;
}

void LinAlg_matmatadd(float A[][3], float B[][3], float C[][3]){
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

void LinAlg_matmatsub(float A[][3], float B[][3], float C[][3]){
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            C[i][j] = A[i][j] - B[i][j];
        }
    }
}

void LinAlg_matscalmult(float A[][3], float k, float C[][3]){
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            C[i][j] = k*A[i][j];
        }
    }
}

void LinAlg_mat2colvecs(float R[][3], float r1[3], float r2[3], float r3[3]){
    for(int i = 0; i < 3; i++){
        r1[i] = R[i][0];
        r2[i] = R[i][1];
        r3[i] = R[i][2];
    }
}

void LinAlg_colvecs2mat(float R[][3], float r1[3], float r2[3], float r3[3]){
    for(int i = 0; i < 3; i++){
        R[i][0] = r1[i];
        R[i][1] = r2[i];
        R[i][2] = r3[i];
    }
}

void LinAlg_matnormalize(float R[][3]){
    //Crude method of ensuring R remains in SO(3)
    float r1[3], r2[3], r3[3];
    float u1[3], u2[3], u3[3];
    float k1, k2, k3;
    float k1u1[3], k2u1[3], k3u2[3];
    LinAlg_mat2colvecs(R, r1, r2, r3);
    //u1 = r1;
    LinAlg_veccopy(r1,u1);

    //u2 = r2 - k1*u1
    //k1 = (r2*u1)/(u1*u1)
    k1 = LinAlg_vecdot(r2,u1)/LinAlg_vecdot(u1,u1);
    LinAlg_vecscalmult(u1,k1u1, k1);
    LinAlg_vecvecsub(r2,k1u1,u2);

    //u3 = r3 - (r3*u1)/(u1*u1)*u1 - (r3*u2)/(u2*u2)*u2
    //u3 = r3 - k2*u1 - k3*u2;
    //k2 = (r3*u1)/(u1*u1)
    //k3 = (r3*u2)/(u2*u2)
    k2 = LinAlg_vecdot(r3,u1)/LinAlg_vecdot(u1,u1);
    k3 = LinAlg_vecdot(r3,u2)/LinAlg_vecdot(u2,u2);
    LinAlg_vecscalmult(u1,k2u1, k2);
    LinAlg_vecscalmult(u2,k3u2, k3);
    LinAlg_veccopy(r3,u3);
    LinAlg_vecvecsub(u3,k2u1,u3);
    LinAlg_vecvecsub(u3,k3u2,u3);

    //Normalize the orthogonal vectors 
    LinAlg_normalize(u1,u1);
    LinAlg_normalize(u2,u2);
    LinAlg_normalize(u3,u3);
    
    LinAlg_colvecs2mat(R,u1,u2,u3);

}

void LinAlg_vec2skew(float x[3], float S[][3]){
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

void LinAlg_matcopy(float A[][3], float B[][3]){
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            B[i][j] = A[i][j];
        }
    }
}

void LinAlg_veccopy(float a[3], float b[3]){
    for(int i = 0; i < 3; i++){
        b[i] = a[i];
    }
}

void LinAlg_transpose(float A[][3], float AT[][3]){
    float temp_mat[3][3]; 
    LinAlg_matcopy(A,temp_mat);
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            AT[i][j] = temp_mat[j][i];
        }
    }
}

float LinAlg_vecnorm(float x[3]){
    float size = sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]);
    return size;
}

void LinAlg_normalize(float x[3], float x_norm[3]){
    float size = LinAlg_vecnorm(x);
    for(int i = 0; i < 3; i++){
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

void LinAlg_expm(float A[][3], float eA[][3], int k){
    LinAlg_eye(eA);
    float B[3][3];
    LinAlg_matcopy(A,B);
    float Bs[3][3]; //scaled B
    float scale = 0;
    for(int i = 1; i <= k; i++){
        scale = 1.0/LinAlg_factorial(i);
        LinAlg_matscalmult(B,scale,Bs); //Bs = scale*B;
        LinAlg_matmatadd(eA,Bs,eA);
        LinAlg_matmatmul(B,A,B);
    }
}

float LinAlg_det(float A[][3]){
    float ret = A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1]);
    ret = ret - A[0][1]*(A[1][0]*A[2][2] - A[2][0]*A[1][2]);
    ret = ret + A[0][2]*(A[1][0]*A[2][1] - A[2][0]*A[1][1]);
    return ret;
}

void LinAlg_rotX(float t, float R[][3]){
    LinAlg_zeromat(R);
    R[0][0] = 1;
    R[1][1] = cos(t);
    R[2][2] = cos(t);
    R[2][1] = sin(t);
    R[1][2] = -sin(t);
}

void LinAlg_rotY(float t, float R[][3]){
    LinAlg_zeromat(R);
    R[1][1] = 1;
    R[0][0] = cos(t);
    R[2][2] = cos(t);
    R[0][2] = sin(t);
    R[2][0] = -sin(t);
}

void LinAlg_rotZ(float t, float R[][3]){
    LinAlg_zeromat(R);
    R[2][2] = 1;
    R[0][0] = cos(t);
    R[1][1] = cos(t);
    R[1][0] = sin(t);
    R[0][1] = -sin(t);
}

void LinAlg_printscal(float a){
    printf("%.8f\n",a);
}

void LinAlg_printvec(float v[3]){
    for(int i = 0; i < 3; i++){
        printf("[%.5f]\n",v[i]);
    }
    printf("\n");
}

void LinAlg_printmat(float A[][3]){
    for(int i = 0; i < 3; i++){
        printf("[");
        for(int j = 0; j < 3; j++){
            if(j < 2){
                printf("%.2f,",A[i][j]);
            }else{
               printf("%.2f",A[i][j]); 
            }
        }
        printf("]\n");
    }
    printf("\n");
}

void LinAlg_printvec_comma_separated(float v[3]){
    for(int i = 0; i < 2; i++){
        printf("%.8f,",v[i]);
    }
    printf("%.8f",v[2]);
    printf("\n");
}
