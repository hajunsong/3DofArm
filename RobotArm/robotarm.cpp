#include "robotarm.h"

// Text file data load
void load_data(char* file_name, vector< vector<double> > *data) {
    FILE *fp_in;
    const int buffer = 1000000;
    char *ptr, basic[buffer];
    fp_in = fopen(file_name, "r");
    while (fgets(basic, buffer, fp_in) != nullptr)
    {
        ptr = strtok(basic, "\t");
        vector<double> data_temp;
        while (ptr != nullptr) {
            data_temp.push_back(static_cast<double>(atof(ptr)));
            ptr = strtok(nullptr, "\t");
        }
        data->push_back(data_temp);
    }
    fclose(fp_in);
}

RobotArm::Body::Body(double psi, double theta, double phi, double sijp_x, double sijp_y, double sijp_z){
    u_vec[0] = 0;
    u_vec[1] = 0;
    u_vec[2] = 1;

    double *Ai_ptr, *ri_ptr, *sijp_ptr;
    Ai_ptr = Ai;
    *(Ai_ptr++) = 1;	*(Ai_ptr++) = 0;	*(Ai_ptr++) = 0;
    *(Ai_ptr++) = 0;	*(Ai_ptr++) = 1;	*(Ai_ptr++) = 0;
    *(Ai_ptr++) = 0;	*(Ai_ptr++) = 0;	*(Ai_ptr) = 1;
    ri_ptr = ri;
    *(ri_ptr++) = 0;	*(ri_ptr++) = 0;	*(ri_ptr++) = 0;
    Euler2Trans(psi, theta, phi, Cij);
    sijp_ptr = sijp;
    *(sijp_ptr++) = sijp_x;	*(sijp_ptr++) = sijp_y;	*(sijp_ptr++) = sijp_z;
}

RobotArm::Body::~Body(){}

RobotArm::RobotArm(uint numbody, uint DOF){
    num_body = numbody;
    dof = DOF;

    start_time = 0;
    end_time = 2;
    h = 0.01;

    L[0] = 0.08205;
    L[1] = 0.135;
    L[2] = 0.147;

    P_offset[0] = 0;
    P_offset[1] = 0.04647;
    P_offset[2] = -0.00497;

    bodys.push_back(Body(0, 0, 0, 0, 0, 0));
    bodys.push_back(Body(M_PI_2, M_PI_2, 0, 0, 0, L[0]));
    bodys.push_back(Body(M_PI, M_PI, 0, L[1], 0, 0));
    bodys.push_back(Body(0, 0, 0, -L[2], 0, 0));
}

RobotArm::~RobotArm(){}

void RobotArm::run_kinematics(){
    sprintf(file_name, "../data/hj_kinematics_result.txt");
    fp = fopen(file_name, "w+");

    vector< vector<double> > input;

    char q_file_name[256];
    sprintf(q_file_name, "../data/kinematics_result.txt");
    load_data(q_file_name, &input);

    for(vector<double> input_data : input){
        double *data = &input_data.front() + 2;
        for (double &thetai : theta){
            thetai = *(data++);
        }
        bodys[1].qi = theta[0];
        bodys[2].qi = theta[1];
        bodys[3].qi = -(M_PI - (theta[2] - theta[1]));

        kinematics();

        save_data();

        printf("Time : %1.3f[s]\n", t_current);

        t_current += h;
    }

    input.clear();
    fclose(fp);
}

void RobotArm::run_kinematics(double *q, double *end){
    bodys[1].qi = q[0];
    bodys[2].qi = q[1];
    bodys[3].qi = M_PI + q[2] + q[1];

    kinematics();

    Body *body_end = &bodys.back();

    end[0] = body_end->re[0];
    end[1] = body_end->re[1];
    end[2] = body_end->re[2];
}

void RobotArm::run_inverse_kinematics(){
    sprintf(file_name, "../data/hj_inverse_kinematics_result.txt");
    fp = fopen(file_name, "w+");

    vector< vector<double> > input;

    char q_file_name[256];
    sprintf(q_file_name, "../data/inverse_kinematics_result.txt");
    load_data(q_file_name, &input);

    double pos_d[3];

    for(vector<double> input_data : input){
        double *data = &input_data.front() + 5;
        pos_d[0] = *(data++);
        pos_d[1] = *(data++);
        pos_d[2] = *(data++);

        inverse_kinematics(pos_d);

        kinematics();

        save_data();

        printf("Time : %1.3f[s]\n", t_current);

        t_current += h;
    }

    input.clear();
    fclose(fp);
}

void RobotArm::run_inverse_kinematics(double *des_pos, double *q){
    inverse_kinematics(des_pos);

    q[0] = bodys[1].qi;
    q[1] = bodys[2].qi;
    q[2] = -M_PI + bodys[3].qi - q[1];
}

void RobotArm::kinematics(){
    for(vector<Body>::iterator body = bodys.begin() + 1, body_pre = bodys.begin(); body != bodys.end(); ++body, ++body_pre){
        // Orientation
        double *Aijpp_ptr = body->Aijpp;
        *(Aijpp_ptr++) = cos(body->qi);	*(Aijpp_ptr++) = -sin(body->qi);	*(Aijpp_ptr++) = 0;
        *(Aijpp_ptr++) = sin(body->qi);	*(Aijpp_ptr++) = cos(body->qi);     *(Aijpp_ptr++) = 0;
        *(Aijpp_ptr++) = 0;             *(Aijpp_ptr++) = 0;                 *(Aijpp_ptr) = 1;

        mat3333(body_pre->Ai, body_pre->Cij, body->Ai_Cij);
        mat3331(body->Ai_Cij, body->u_vec, body->Hi);
        mat3333(body->Ai_Cij, body->Aijpp, body->Ai);

        // Position
        mat3331(body_pre->Ai, body_pre->sijp, body_pre->sij);

        for (uint i = 0; i < 3; i++) {
            body->ri[i] = body_pre->ri[i] + body_pre->sij[i];
        }
    }

    // End point
    Body *body_end = &bodys.back();
    mat3331(body_end->Ai, body_end->sijp, body_end->sij);

    double temp[3];
    mat3331(bodys[1].Ai, P_offset, temp);

    for (uint i = 0; i < 3; i++) {
        body_end->re[i] = body_end->ri[i] + body_end->sij[i] + temp[i];
    }

//    mat3333(body_end->Ai, body_end->Cij, body_end->Ae);
    memcpy(bodys.back().Ae, bodys[1].Ai, sizeof(double)*9);

    body_end->roll = atan2(body_end->Ae[2 * 3 + 1], body_end->Ae[2 * 3 + 2]);
    body_end->pitch = atan2(-body_end->Ae[2 * 3 + 0], sqrt(pow(body_end->Ae[2 * 3 + 1], 2.0) + pow(body_end->Ae[2 * 3 + 2], 2.0)));
    body_end->yaw = atan2(body_end->Ae[1 * 3 + 0], body_end->Ae[0 * 3 + 0]);
}

void RobotArm::inverse_kinematics(double *des_pos){
    double Px = des_pos[0];
    double Py = des_pos[1];
    double Pz = des_pos[2] - P_offset[2];
    double d = bodys.front().sijp[0]*0;
    double d2 = d*d;
    double Px2 = Px*Px;
    double Py2 = Py*Py;
    double r = sqrt(Px2 + Py2);
    double phi = atan2(Py, Px);
    double alpha = atan2(d, sqrt(Px2 + Py2 - d2));
    double q1[2];
    q1[0] = phi + alpha;
    alpha = atan2(Py, Px);
    double gamma = atan2(-d, -sqrt(r*r - d2));
    double beta = gamma + M_PI;
    q1[1] = alpha + beta;
//    cout << q1[0] << endl;
//    cout << q1[1] << endl;

    double d1 = L[0];
    double s = Pz - d1;
    r = sqrt(Px2 + Py2 - d2) - sqrt(P_offset[0]*P_offset[0] + P_offset[1]*P_offset[1]);
    double L2 = L[1];
    double L3 = L[2];
    double C3 = (r*r + s*s - L2*L2 - L3*L3)/(2*L2*L3);
    double S3[2] = {sqrt(1-C3*C3), -sqrt(1-C3*C3)};
    double q3[2] = {atan2(S3[0], C3), atan2(S3[1], C3)};
//    cout << q3[0] << endl;
//    cout << q3[1] << endl;
    double q2[2] = {atan2(s, r) - atan2(L3*S3[0], L2 + L3*C3), atan2(s, r) - atan2(L3*S3[1], L2 + L3*C3)};
//    cout << q2[0] << endl;
//    cout << q2[1] << endl;

    bodys[1].qi = q1[1] - M_PI_2;
    bodys[2].qi = q2[1];
    bodys[3].qi = q3[0];
}

void RobotArm::save_data(){
    fprintf(fp, "%.10f\t", t_current);

    theta[0] = bodys[1].qi;
    theta[1] = bodys[2].qi;
    theta[2] = M_PI + bodys[3].qi + theta[1];

    for (double thetai : theta){
        fprintf(fp, "%.10f\t", thetai);
    }

    Body *body_end = &bodys.back();
    double *re_ptr = body_end->re;
    for(uint i = 0; i < 3; i++){
        fprintf(fp, "%.10f\t", *(re_ptr++));
    }
    fprintf(fp, "%.10f\t%.10f\t%.10f", body_end->roll, body_end->pitch, body_end->yaw);
    fprintf(fp, "\n");
}

// Euler Angle to Transformation Matrix(body 3-1-3)
void Euler2Trans(double psi, double theta, double phi, double* Mat)
{
    double Rz1[9], Rx[9], Rz2[9], *Rz1_ptr, *Rx_ptr, *Rz2_ptr;
    Rz1_ptr = Rz1;
    *(Rz1_ptr++) = cos(psi);    *(Rz1_ptr++) = -sin(psi);   *(Rz1_ptr++) = 0;
    *(Rz1_ptr++) = sin(psi);    *(Rz1_ptr++) = cos(psi);    *(Rz1_ptr++) = 0;
    *(Rz1_ptr++) = 0;           *(Rz1_ptr++) = 0;           *(Rz1_ptr) = 1;
    Rx_ptr = Rx;
    *(Rx_ptr++) = 1;            *(Rx_ptr++) = 0;            *(Rx_ptr++) = 0;
    *(Rx_ptr++) = 0;            *(Rx_ptr++) = cos(theta);   *(Rx_ptr++) = -sin(theta);
    *(Rx_ptr++) = 0;            *(Rx_ptr++) = sin(theta);   *(Rx_ptr) = cos(theta);
    Rz2_ptr = Rz2;
    *(Rz2_ptr++) = cos(phi);    *(Rz2_ptr++) = -sin(phi);   *(Rz2_ptr++) = 0;
    *(Rz2_ptr++) = sin(phi);    *(Rz2_ptr++) = cos(phi);    *(Rz2_ptr++) = 0;
    *(Rz2_ptr++) = 0;           *(Rz2_ptr++) = 0;           *(Rz2_ptr) = 1;

    double Rz1_Rx[9] = {0,};
    for(uint i = 0; i < 3; i++){
        for(uint j = 0; j < 3; j++){
            for(uint k = 0; k < 3; k++){
                Rz1_Rx[i*3 + j] += Rz1[i*3 + k]*Rx[k*3 + j];
            }
        }
    }
    for(uint i = 0; i < 3; i++){
        for(uint j = 0; j < 3; j++){
            Mat[i*3+j] = 0;
            for(uint k = 0; k < 3; k++){
                Mat[i*3 + j] += Rz1_Rx[i*3 + k]*Rz2[k*3 + j];
            }
        }
    }
}

// Matrix & Vector Calculation
void mat3333(double *a, double *b, double *c)
{
    for(uint i = 0; i < 3; i++){
        for(uint j = 0; j < 3; j++){
            c[i*3+j] = 0;
            for(uint k = 0; k < 3; k++){
                c[i*3+j] += a[i*3 + k]*b[k*3 + j];
            }
        }
    }
}

void mat3331(double *a, double *b, double *c)
{
    for(uint i = 0; i < 3; i++){
        c[i] = 0;
        for(uint j = 0; j < 3; j++){
            c[i] += a[i*3 + j]*b[j];
        }
    }
}
