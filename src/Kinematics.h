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
    double K1_len;
    double K2_len;
    double K3_len;
    double K3_ang;
public:
    Kinematics();
    Kinematics(double, double, double, double);
    vector<double> inverseKinematics(vector<double>);
    String printInverseK(vector<double>); 
    Matrix<matrix_size,matrix_size> getR03(vector<double>);
    Matrix<matrix_size,matrix_size> RPY(vector<double>);
    Matrix<matrix_size,matrix_size> rotX(double);
    Matrix<matrix_size,matrix_size> rotZ(double);
    double roundToOne(double);
    void roundMatrix(Matrix<matrix_size,matrix_size>&);
    bool isMatrixValueOk(Matrix<matrix_size,matrix_size>);
};

#endif 