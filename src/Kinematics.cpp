#include "Kinematics.h"

using namespace BLA;

Kinematics::Kinematics(){
    K1_len=0;
    K2_len=0;
    K3_len=0;
    K3_ang=0;
}

Kinematics::Kinematics(float k1_l, float k2_l, float k3_l, float k3_a){
    K1_len=k1_l;
    K2_len=k2_l;
    K3_len=k3_l;
    K3_ang=k3_a;
    homePosition=forwardKinematics({0,0,0,0,0,0});
}

vector<float> Kinematics::inverseKinematics(vector<float> position){
    vector<float> angles;

    float sqrtpos_xy=sqrt(pow(position[0],2)+pow(position[1],2));
    
    float w2=pow(position[2],2)+pow(sqrtpos_xy-K1_len,2);
    float cos_sum_K3=(pow(K2_len,2)+pow(K3_len,2)-w2)/(2*K2_len*K3_len);

    float angle_c=atan2(sqrt(1-pow(cos_sum_K3,2)),cos_sum_K3)*180/PI-K3_ang;

    float angle_b=90.-(atan2(position[2],sqrtpos_xy-K1_len)+atan2(K3_len*sin(PI-(angle_c+K3_ang)/180*PI),K2_len+K3_len*cos(PI-(angle_c+K3_ang)/180*PI)))*180/PI;

    float angle_a=atan2(position[1],position[0])*180/PI;

    angle_c=angle_c-90;

    Matrix<matrix_size,matrix_size> R03=this->getR03({angle_a, angle_b, angle_c});
    bool isNonsingular = Invert(R03);
        if(!isNonsingular) return vector<float>();
    Matrix<matrix_size,matrix_size> RPY=this->RPY({position[3], position[4], position[5]});
    float deg=PI/2;
    Matrix<matrix_size,matrix_size> corr=rotX(deg)*rotZ(deg)*rotX(deg);
    Matrix<matrix_size,matrix_size> R36= R03*RPY; 

    R36*=corr;
    roundMatrix(R36);

    //Serial.println(R36);

    //if(!isMatrixValueOk(R36)) return angles;

    float angle_e=atan2(sqrt(1-pow(R36(2,2),2)),R36(2,2))*180/PI;

    float angle_d=atan2(R36(1,2),R36(0,2))*180/PI; 

    float angle_f=atan2(R36(2,1),-R36(2,0))*180/PI; 

    //if(angle_e==0 && angle_d == -angle_f){
    //    angle_d=0;
    //    angle_f=0;
    //}
    
    angles.push_back(angle_a);
    angles.push_back(angle_b);
    angles.push_back(angle_c);
    angles.push_back(angle_d);
    angles.push_back(angle_e);
    angles.push_back(angle_f);

    return angles;
}

vector<float> Kinematics::forwardKinematics(vector<float> angles){
    vector<float> pos;

    for (size_t i = 0; i < angles.size(); ++i) 
        angles[i]=angles[i]/180*PI;

    float angle=angles[2]+(K3_ang/180*PI)-angles[1];
    float xy=K1_len+K2_len*sin(angles[1])+K3_len*cos(angle);
    float x=xy*cos(angles[0]);
    float y=xy*sin(angles[0]);

    float z=K2_len*cos(angles[1])+K3_len*sin(angle);

    pos.push_back(x);
    pos.push_back(y);
    pos.push_back(z);

    for (size_t i = 0; i < angles.size(); ++i) 
        angles[i]=angles[i]*180/PI;

    Matrix<matrix_size,matrix_size> R06=this->getR06(angles);
    
    float deg=PI/2;
    Matrix<matrix_size,matrix_size> corr=rotX(deg)*rotZ(deg)*rotX(deg);
    R06*=corr;

    float roll=atan2(R06(2,1),R06(2,2))*180/PI;
    float pitch=asin(-R06(2,0))*180/PI;
    float yaw=atan2(R06(1,0),R06(0,0))*180/PI;

    pos.push_back(yaw);
    pos.push_back(pitch);
    pos.push_back(roll);

    return pos;
}

String Kinematics::printInverseK(vector<float> position){
    vector<float> pos= this->inverseKinematics(position);

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

String Kinematics::printForwardK(vector<float> position){
    vector<float> pos= this->forwardKinematics(position);

    String build="";

    build+="x:";
    build+=pos[0];
    build+=", y:";
    build+=pos[1];
    build+=", z:";
    build+=pos[2];
    build+=", yaw:";
    build+=pos[3];
    build+=", pitch:";
    build+=pos[4];
    build+=", roll:";
    build+=pos[5];       
    if(pos.size()==0){
        build+="\n Math Error";
    }

    return build;
}

Matrix<matrix_size,matrix_size> Kinematics::getR03(vector<float> angles){
    for (size_t i = 0; i < angles.size(); ++i) 
        angles[i]=angles[i]/180*PI;
    angles[1]=-angles[1]; // schema and real move direction difference

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

Matrix<matrix_size,matrix_size> Kinematics::getR06(vector<float> angles){
    Matrix<matrix_size,matrix_size> R03=this->getR03(angles);

    for (size_t i = 0; i < angles.size(); ++i) 
        angles[i]=angles[i]/180*PI;
    Matrix<matrix_size,matrix_size> R36=
    {
        cos(angles[3])*cos(angles[4])*cos(angles[5])-sin(angles[3])*sin(angles[5]),
        -cos(angles[3])*cos(angles[4])*sin(angles[5])-sin(angles[3])*cos(angles[5]),
        cos(angles[3])*sin(angles[4]),
        sin(angles[3])*cos(angles[4])*cos(angles[5])+cos(angles[3])*sin(angles[5]),
        -sin(angles[3])*cos(angles[4])*sin(angles[5])+cos(angles[3])*cos(angles[5]),
        sin(angles[3])*sin(angles[4]),
        -sin(angles[4])*cos(angles[5]),
        sin(angles[4])*sin(angles[5]),
        cos(angles[4])
    };
    return R03*=R36;
}

Matrix<matrix_size,matrix_size> Kinematics::RPY(vector<float> angles){
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

Matrix<matrix_size,matrix_size> Kinematics::rotX(float rad){
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

Matrix<matrix_size,matrix_size> Kinematics::rotZ(float rad){
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

float Kinematics::roundToOne(float element){
    float tolerance=0.001;
    if(abs(element)<1.0+tolerance && abs(element)>1){
        if (element - 1.0 > 0) {
            return 1.0;
        } else if (element + 1.0 < 0) {
            return -1.0;
        }
    }
    else if(abs(element)<tolerance){
        return 0;
    }
    return 2;
}

void Kinematics::roundMatrix(Matrix<matrix_size,matrix_size> &matrix){
    float roundTry;
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