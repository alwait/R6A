#include "Kinematics.h"

using namespace BLA;

Kinematics::Kinematics(){
    K1_len=0;
    K2_len=0;
    K3_len=0;
    K3_ang=0;
    //Matrix<3,3> B;
    //Matrix<3,3> Inverse(B);
}

Kinematics::Kinematics(double k1_l, double k2_l, double k3_l, double k3_a){
    K1_len=k1_l;
    K2_len=k2_l;
    K3_len=k3_l;
    K3_ang=k3_a;
}

vector<double> Kinematics::inverseKinematics(vector<double> position){
    vector<double> angles;

    double sqrtpos_xy=sqrt(pow(position[0],2)+pow(position[1],2));
    
    double w2=pow(position[2],2)+pow(sqrtpos_xy-K1_len,2);
    double cos_sum_K3=(pow(K2_len,2)+pow(K3_len,2)-w2)/(2*K2_len*K3_len);

    double angle_c=atan2(sqrt(1-pow(cos_sum_K3,2)),cos_sum_K3)*180/PI-K3_ang;

    double angle_b=90.-(atan2(position[2],sqrtpos_xy-K1_len)+atan2(K3_len*sin(PI-(angle_c+K3_ang)/180*PI),K2_len+K3_len*cos(PI-(angle_c+K3_ang)/180*PI)))*180/PI;

    double angle_a=atan2(position[1],position[0])*180/PI;

    angle_c=angle_c-90;

    Matrix<matrix_size,matrix_size> R03=this->getR03({angle_a, angle_b, angle_c});
    bool isNonsingular = Invert(R03);
        if(!isNonsingular) return vector<double>();
    Matrix<matrix_size,matrix_size> RPY=this->RPY({position[3], position[4], position[5]});
    double deg=PI/2;
    Matrix<matrix_size,matrix_size> corr=rotX(deg)*rotZ(deg)*rotX(deg);
    Matrix<matrix_size,matrix_size> R36= R03*RPY; 

    R36*=corr;
    roundMatrix(R36);

    Serial.println(R36);

    //if(!isMatrixValueOk(R36)) return angles;

    double angle_e=atan2(sqrt(1-pow(R36(2,2),2)),R36(2,2))*180/PI;

    double angle_d=atan2(R36(1,2),R36(0,2))*180/PI; 

    double angle_f=atan2(R36(2,1),-R36(2,0))*180/PI; 
    
    angles.push_back(angle_a);
    angles.push_back(angle_b);
    angles.push_back(angle_c);
    angles.push_back(angle_d);
    angles.push_back(angle_e);
    angles.push_back(angle_f);

    return angles;
}

String Kinematics::printInverseK(vector<double> position){
    vector<double> pos= this->inverseKinematics(position);

    String build="";

    build+="a:";
    build+=pos[0];
    build+=", b:";
    build+=pos[1];
    build+=", c:";
    build+=pos[2];
    build+=", d:";
    build+=pos[3];
    build+=", e:";
    build+=pos[4];
    build+=", f:";
    build+=pos[5];       
    if(pos.size()==0){
        build+="\n Math Error";
    }

    return build;
}

Matrix<matrix_size,matrix_size> Kinematics::getR03(vector<double> angles){
    for (size_t i = 0; i < angles.size(); ++i) 
        angles[i]=angles[i]/180*PI;
    angles[1]=-angles[1];

    Matrix<matrix_size,matrix_size> R03=
    {
        -cos(angles[0])*sin(angles[1]+angles[2]),
        sin(angles[0]),
        cos(angles[0])*cos(angles[1]+angles[2]),
        -sin(angles[0])*sin(angles[1]+angles[2]),
        -cos(angles[0]),
        sin(angles[0])*cos(angles[1]+angles[2]),
        cos(angles[1]+angles[2]),
        0,
        sin(angles[1]+angles[2])
    };

    return R03;
}

Matrix<matrix_size,matrix_size> Kinematics::RPY(vector<double> angles){
    for (size_t i = 0; i < angles.size(); ++i) 
        angles[i]=angles[i]/180*PI;

    Matrix<matrix_size,matrix_size> RPY=
    {
        cos(angles[0])*cos(angles[1]),
        -sin(angles[0])*cos(angles[2])+cos(angles[0])*sin(angles[1])*sin(angles[2]),
        sin(angles[0])*sin(angles[2])+cos(angles[0])*sin(angles[1])*cos(angles[2]),
        sin(angles[0])*cos(angles[1]),
        cos(angles[0])*cos(angles[2])+sin(angles[0])*sin(angles[1])*sin(angles[2]),
        -cos(angles[0])*sin(angles[2])+sin(angles[0])*sin(angles[1])*cos(angles[2]),
        -sin(angles[1]),
        cos(angles[1])*sin(angles[2]),
        cos(angles[1])*cos(angles[2])
    };

    return RPY;
}

Matrix<matrix_size,matrix_size> Kinematics::rotX(double rad){
    Matrix<matrix_size,matrix_size> rot=
    {
        1,
        0,
        0,
        0,
        cos(rad),
        -sin(rad),
        0,
        sin(rad),
        cos(rad)
    };
    return rot;
}

Matrix<matrix_size,matrix_size> Kinematics::rotZ(double rad){
    Matrix<matrix_size,matrix_size> rot=
    {
        cos(rad),
        -sin(rad),
        0,
        sin(rad),
        cos(rad),
        0,
        0,
        0,
        1
    };
    return rot;
}

double Kinematics::roundToOne(double element){
    double tolerance=0.001;
    if(abs(element)<1.0+tolerance && abs(element)>1){
        if (element - 1.0 > 0) {
            return 1.0;
        } else if (element + 1.0 < 0) {
            return -1.0;
        }
    }
    //else if(abs(element)<tolerance){
    //    return 0;
    //}
    else return 2;
}

void Kinematics::roundMatrix(Matrix<matrix_size,matrix_size> &matrix){
    double roundTry;
    for (size_t i = 0; i < matrix.Rows; ++i) 
    {
        for (size_t j = 0; j < matrix.Cols; ++j) 
        {   
            roundTry=roundToOne(matrix(i,j));
            if(roundTry!=2){
                matrix(i,j)= roundToOne(matrix(i,j));
            }
        }
    }
}

bool Kinematics::isMatrixValueOk(Matrix<matrix_size,matrix_size> matrix){
    for (size_t i = 0; i < matrix.Rows; ++i) 
    {
        for (size_t j = 0; j < matrix.Cols; ++j) 
        {   
            if(abs(matrix(i,j))>1){
                return false;
            }
        }
    }
    return true;
}