#pragma once

#include <iostream>
#include <vector>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>

#include "numerical.h"

using namespace std;
typedef unsigned int uint;

class RobotArm
{
public:
    RobotArm(uint numbody, uint DOF);
    ~RobotArm();
    void run_kinematics();
    void run_inverse_kinematics();
    void run_kinematics(double *q, double *end);
    void run_inverse_kinematics(double *end, double *q);

private:
    class Body
    {
    public:
        Body(double psi, double theta, double phi, double sijp_x, double sijp_y, double sijp_z);
        ~Body();
        // base body information
        double A0[9], C01[9], s01p[3], J0p[9], r0[3], m0, A0_C01[9], C01_A01pp[9];
        // body initial data
        double qi, qi_dot, mi, qi_init;
        double ri[3], ri_dot[3], wi[3], rhoip[3], sijp[3], Jip[9], Cii[9], Cij[9], Ai_Cij[9], Cij_Aijpp[9];
        // Orientation
        double Aijpp[9], Ai[9], Hi[3], u_vec[3];
        // Position
        double sij[3], rhoi[3], ric[3], rit[9];
        // End point
        double re[3], Ae[9], roll, pitch, yaw;
    };

    // robot parameter
    double theta[3], L[3], P[3], P_offset[3];
    uint num_body, dof;

    // system variable
    double start_time, end_time, h, g, t_current;

    // file
    char file_name[256];
    FILE *fp;

    vector<Body> bodys;
    Numerical *numeric;

    void kinematics();
    void inverse_kinematics(double *des_pos);
    void save_data();
};

// Text file data load
void load_data(char* file_name, vector< vector<double> > *data);
// Euler Angle to Transformation Matrix(body 3-1-3)
void Euler2Trans(double psi, double theta, double phi, double* Mat);
// Matrix & Vector Calculation
void mat3333(double *a, double *b, double *c);
void mat3331(double *a, double *b, double *c);
