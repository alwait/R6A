#ifndef KINEMATICS_H
#define KINEMATICS_H

#define matrix_size 3

#include <vector>
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include "math.h"

using namespace std;
using namespace BLA;

class Kinematics{
private:
    float K1_len;
    float K2_len;
    float K3_len;
    float K3_ang;
    vector<float> homePosition;
    Matrix<matrix_size> toolOffset;
public:
    Kinematics();
    Kinematics(float, float, float, float);
    vector<float> inverseKinematics(vector<float>);
    vector<float> forwardKinematics(vector<float>);
    String printInverseK(vector<float>); 
    String printForwardK(vector<float>); 
    Matrix<matrix_size,matrix_size> getR03(vector<float>);
    Matrix<matrix_size,matrix_size> getR06(vector<float>);
    Matrix<matrix_size,matrix_size> RPY(vector<float>);
    Matrix<matrix_size,matrix_size> rotX(float);
    Matrix<matrix_size,matrix_size> rotZ(float);
    float roundToOne(float);
    void roundMatrix(Matrix<matrix_size,matrix_size>&);
    bool isMatrixValueOk(Matrix<matrix_size,matrix_size>);
    vector<float> getHomePosition(){return homePosition;};
};

#endif 