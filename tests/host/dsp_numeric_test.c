/** @file dsp_numeric_test.c
 * @brief 固定版本DSP的矩阵和三角函数回归；使用双精度参考及Release始终执行的检查。
 */
#include "test_check.h"
#include "dsp_matrix.h"
#include <math.h>
#include <float.h>
#include <string.h>
#include <stdint.h>
static void near(float a,double b,double tolerance)
{ TEST_CHECK(isfinite(a) && fabs((double)a-b)<=tolerance); }
// 独立双精度Gauss-Jordan参考，选绝对值最大的主元，和厂商实现分开保存数据。
static void reference(const float *a,double *out,unsigned n)
{
    double work[8][16]={0};
    for(unsigned i=0;i<n;i++) for(unsigned j=0;j<n;j++) { work[i][j]=a[i*n+j]; work[i][n+j]=(i==j); }
    for(unsigned i=0;i<n;i++) {
        unsigned pivot=i;
        for(unsigned j=i+1;j<n;j++) if(fabs(work[j][i])>fabs(work[pivot][i]))pivot=j;
        TEST_CHECK(fabs(work[pivot][i])>1e-12);
        for(unsigned j=0;j<2*n;j++) { double v=work[i][j];work[i][j]=work[pivot][j];work[pivot][j]=v; }
        double div=work[i][i]; for(unsigned j=0;j<2*n;j++) work[i][j]/=div;
        for(unsigned r=0;r<n;r++) if(r!=i) {
            double factor=work[r][i]; for(unsigned j=0;j<2*n;j++) work[r][j]-=factor*work[i][j];
        }
    }
    for(unsigned i=0;i<n;i++) for(unsigned j=0;j<n;j++) out[i*n+j]=work[i][n+j];
}
static void inverse(void)
{
    float a[64],old[64],out[64],scratch[128]; double expected[64];
    for(unsigned n=1;n<=8;n++) {
        for(unsigned i=0;i<n;i++) for(unsigned j=0;j<n;j++) a[i*n+j]=i==j?10.0f+i:((int)(i+j)%3-1)*.125f;
        memcpy(old,a,n*n*sizeof(float)); reference(a,expected,n);
        arm_matrix_instance_f32 input={n,n,a},output={n,n,out};
        TEST_CHECK(DSPMatrixInverseF32(&input,&output,scratch,2*n*n)==ARM_MATH_SUCCESS);
        TEST_CHECK(memcmp(a,old,n*n*sizeof(float))==0);
        for(unsigned i=0;i<n*n;i++) near(out[i],expected[i],2e-6);
        for(unsigned i=0;i<n;i++) for(unsigned j=0;j<n;j++) {
            double residual=0; for(unsigned k=0;k<n;k++) residual+=(double)a[i*n+k]*out[k*n+j];
            TEST_CHECK(fabs(residual-(i==j))<2e-6);
        }
    }
    float pivot[4]={0,2,1,3}; arm_matrix_instance_f32 input={2,2,pivot},output={2,2,out};
    TEST_CHECK(DSPMatrixInverseF32(&input,&output,scratch,8)==ARM_MATH_SUCCESS);
    near(out[0],-1.5,1e-6); near(out[1],1,1e-6); near(out[2],.5,1e-6); near(out[3],0,1e-6);
    float saved[4];memcpy(saved,out,sizeof(saved));
    arm_matrix_instance_f32 alias=input;
    TEST_CHECK(DSPMatrixInverseF32(&input,&alias,scratch,8)==ARM_MATH_ARGUMENT_ERROR);
    alias=output;alias.numRows=1;
    TEST_CHECK(DSPMatrixInverseF32(&input,&alias,scratch,8)==ARM_MATH_SIZE_MISMATCH);
    for(unsigned fault=0;fault<9;fault++) {
        float bad[4]={1,2,2,4},copy[4]; input=(arm_matrix_instance_f32){2,2,bad};
        if(fault==1)bad[0]=NAN; if(fault==2)bad[0]=INFINITY;
        if(fault==3){bad[0]=FLT_TRUE_MIN;bad[1]=bad[2]=0;bad[3]=1;}
        if(fault==4)input.numCols=1;
        memcpy(copy,bad,sizeof(copy));
        arm_status s;
        if(fault==5)s=DSPMatrixInverseF32(&input,&output,scratch,7);
        else if(fault==6)s=DSPMatrixInverseF32(&input,&output,bad,8);
        else if(fault==7)s=DSPMatrixInverseF32(NULL,&output,scratch,8);
        else if(fault==8)s=DSPMatrixInverseF32(&input,&output,(float *)((uint8_t *)scratch+1),8);
        else s=DSPMatrixInverseF32(&input,&output,scratch,8);
        TEST_CHECK(s!=ARM_MATH_SUCCESS && memcmp(saved,out,sizeof(saved))==0 && memcmp(copy,bad,sizeof(copy))==0);
    }
}
static void matrices(void)
{
    float a[6]={1,2,3,4,5,6},b[6]={-1,0,2,3,4,5},out[9]={0};
    arm_matrix_instance_f32 x,y,z;arm_mat_init_f32(&x,2,3,a);arm_mat_init_f32(&y,2,3,b);arm_mat_init_f32(&z,2,3,out);
    TEST_CHECK(x.numRows==2 && x.numCols==3 && x.pData==a);
    TEST_CHECK(arm_mat_add_f32(&x,&y,&z)==ARM_MATH_SUCCESS);
    for(unsigned i=0;i<6;i++)near(out[i],(double)a[i]+b[i],0);
    TEST_CHECK(arm_mat_sub_f32(&x,&y,&z)==ARM_MATH_SUCCESS);
    for(unsigned i=0;i<6;i++)near(out[i],(double)a[i]-b[i],0);
    z.numRows=3;z.numCols=2; TEST_CHECK(arm_mat_trans_f32(&x,&z)==ARM_MATH_SUCCESS);
    const float transpose[6]={1,4,2,5,3,6};TEST_CHECK(memcmp(out,transpose,sizeof(transpose))==0);
    float product[4];arm_matrix_instance_f32 result={2,2,product};
    TEST_CHECK(arm_mat_mult_f32(&x,&z,&result)==ARM_MATH_SUCCESS);
    near(product[0],14,0);near(product[1],32,0);near(product[2],32,0);near(product[3],77,0);
    TEST_CHECK(arm_mat_mult_f32(&x,&y,&result)==ARM_MATH_SIZE_MISMATCH);
    TEST_CHECK(arm_mat_add_f32(&x,&y,&result)==ARM_MATH_SIZE_MISMATCH);
    TEST_CHECK(arm_mat_sub_f32(&x,&y,&result)==ARM_MATH_SIZE_MISMATCH);
    TEST_CHECK(arm_mat_trans_f32(&x,&result)==ARM_MATH_SIZE_MISMATCH);
}
int main(void)
{
    matrices();inverse();
    const double pi=3.14159265358979323846;
    // 包含零、整周、四分之一周和负角度；比较传入float本身代表的角度。
    for(int i=-8192;i<=8192;i++) {
        float angle=(float)(i*pi/1024.0);
        near(arm_sin_f32(angle),sin((double)angle),3e-5);
        near(arm_cos_f32(angle),cos((double)angle),3e-5);
    }
    return 0;
}
