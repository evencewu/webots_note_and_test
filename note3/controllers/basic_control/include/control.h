#ifndef _CONTROL_
#define _CONTROL_
//装点控制算法的代码


void LEG_solution_pos(int RL,double A,double B){
  if(RL == 0){
    ROBOT.leg[R].Px =       - (l1*cos(A))/2 + (l1*cos(B))/2 + (l1*sin(A))/2 - (l1*sin(B))/2;
    ROBOT.leg[R].Py = - d/2 - (l1*cos(A))/2 - (l1*cos(B))/2 - (l1*sin(A))/2 - (l1*sin(B))/2;
  }else if(RL == 1){
    ROBOT.leg[L].Px =       - (l1*cos(A))/2 + (l1*cos(B))/2 + (l1*sin(A))/2 - (l1*sin(B))/2;
    ROBOT.leg[L].Py = - d/2 - (l1*cos(A))/2 - (l1*cos(B))/2 - (l1*sin(A))/2 - (l1*sin(B))/2;
  }
};

void LEG_solution_pos_opposite(int RL,double X,double Y){
  if(RL == 0){  
    double O1 = -(4*pow(X,2) + 4*pow(Y,2) + 4*Y*d + pow(d,2))*pow((4*pow(X,2) + 4*pow(Y,2) + 4*Y*d + pow(d,2) - 8*pow(l1,2)),(1/2));
    double O2 = (4*pow(X,2) + 4*l1*X + 4*pow(Y,2) + 4*Y*d - 4*l1*Y + pow(d,2) - 2*l1*d);
    double O3 = (4*pow(X,2) - 4*l1*X + 4*pow(Y,2) + 4*Y*d - 4*l1*Y + pow(d,2) - 2*l1*d);

    ROBOT.motor[RB_MOTOR].want_P =  2*atan((4*X*l1 + 4*Y*l1 + 2*d*l1 - O1)/O2) - 2*atan((4*X*l1 + 4*Y*l1 + 2*d*l1 + O1)/O2) -1.570796;
    ROBOT.motor[RF_MOTOR].want_P = -2*atan((4*X*l1 - 4*Y*l1 - 2*d*l1 + O1)/O3) - 2*atan((4*Y*l1 - 4*X*l1 + 2*d*l1 + O1)/O3) -1.570796;
   
  }else if(RL == 1){
    double O1 = -(4*pow(X,2) + 4*pow(Y,2) + 4*Y*d + pow(d,2))*pow((4*pow(X,2) + 4*pow(Y,2) + 4*Y*d + pow(d,2) - 8*pow(l1,2)),(1/2));
    double O2 = (4*pow(X,2) + 4*l1*X + 4*pow(Y,2) + 4*Y*d - 4*l1*Y + pow(d,2) - 2*l1*d);
    double O3 = (4*pow(X,2) - 4*l1*X + 4*pow(Y,2) + 4*Y*d - 4*l1*Y + pow(d,2) - 2*l1*d);

    ROBOT.motor[LB_MOTOR].want_P =  2*atan((4*X*l1 + 4*Y*l1 + 2*d*l1 - O1)/O2) - 2*atan((4*X*l1 + 4*Y*l1 + 2*d*l1 + O1)/O2) -1.570796;
    ROBOT.motor[LF_MOTOR].want_P = -2*atan((4*X*l1 - 4*Y*l1 - 2*d*l1 + O1)/O3) - 2*atan((4*Y*l1 - 4*X*l1 + 2*d*l1 + O1)/O3) -1.570796;
  }
};

void LEG_solution_speed(int RL,double A ,double B ,double w1,double w2){  if(RL == 0){
    ROBOT.leg[R].Vx =  (l1*cos(A)*w1)/2 + (l1*sin(A)*w1)/2 - (l1*cos(B)*w2)/2 - (l1*sin(B)*w2)/2;
    ROBOT.leg[R].Vy = -(l1*cos(A)*w1)/2 + (l1*sin(A)*w1)/2 - (l1*cos(B)*w2)/2 + (l1*sin(B)*w2)/2;
  }else if(RL == 1){
    ROBOT.leg[L].Vx =  (l1*cos(A)*w1)/2 + (l1*sin(A)*w1)/2 - (l1*cos(B)*w2)/2 - (l1*sin(B)*w2)/2;
    ROBOT.leg[L].Vy = -(l1*cos(A)*w1)/2 + (l1*sin(A)*w1)/2 - (l1*cos(B)*w2)/2 + (l1*sin(B)*w2)/2;
  }
};

void LEG_solution_speed_opposite(int RL,double A,double B,double Vx,double Vy){
    if(RL == 0){
        ROBOT.motor[RB_MOTOR].want_W = -(Vy*cos(B) - Vx*cos(B) + Vx*sin(B) + Vy*sin(B))/(l1*(cos(A)*cos(B) - sin(A)*sin(B))); 
        ROBOT.motor[RF_MOTOR].want_W = (Vx*cos(A) + Vy*cos(A) - Vx*sin(A) + Vy*sin(A))/(l1*cos(A)*cos(B) - l1*sin(A)*sin(B)); 
    }else if(RL == 1){
        ROBOT.motor[LB_MOTOR].want_W = -(Vy*cos(B) - Vx*cos(B) + Vx*sin(B) + Vy*sin(B))/(l1*(cos(A)*cos(B) - sin(A)*sin(B)));  
        ROBOT.motor[LF_MOTOR].want_W = (Vx*cos(A) + Vy*cos(A) - Vx*sin(A) + Vy*sin(A))/(l1*cos(A)*cos(B) - l1*sin(A)*sin(B)); 
    }

};

//void stable_leg(int RL,double x,double y,double w_x ,double w_y,double a,double b){
//    double k_1 = 1;
//    double k_e_x= 1;
//    double k_e_y= 1;
//
//    double k_P_2 = 20;
//    double k_D_2 = 0;
//    double k_I_2 = 0;
//
//    double error_x =  w_x - x;
//    double error_y =  w_y - y;
//
//    double w_v_x = (pow(e,fabs(error_x)*k_e_x)-1)* k_1 *error_x/fabs(error_x);
//    double w_v_y = (pow(e,fabs(error_y)*k_e_y)-1)* k_1 *error_y/fabs(error_y);
//
//    printf("%f \n",w_v_x);
//    printf("%f \n",w_v_y);
//    LEG_solution_speed_opposite(RL,a,b,w_v_x,w_v_y);
//    
//    if(RL == 0){
//        double error_v_1 =  1*(ROBOT.motor[RB_MOTOR].want_W - ROBOT.position_sensor[RB_MOTOR].w);
//        double error_v_2 =  -1*(ROBOT.motor[RF_MOTOR].want_W - ROBOT.position_sensor[RF_MOTOR].w);  
//        ROBOT.motor[RB_MOTOR].torque = error_v_1 *k_P_2 - k_D_2 *ROBOT.motor[RB_MOTOR].d_torque + ROBOT.position_sensor[RB_MOTOR].w * k_I_2;
//        ROBOT.motor[RF_MOTOR].torque = error_v_2 *k_P_2 + k_D_2 *ROBOT.motor[RF_MOTOR].d_torque - ROBOT.position_sensor[RF_MOTOR].w * k_I_2;
//    }else if(RL == 1){
//        double error_v_1 =  1*(ROBOT.motor[LB_MOTOR].want_W - ROBOT.position_sensor[LB_MOTOR].w);
//        double error_v_2 =  -1*(ROBOT.motor[LF_MOTOR].want_W - ROBOT.position_sensor[LF_MOTOR].w); 
//        ROBOT.motor[LB_MOTOR].torque = error_v_1 *k_P_2 - k_D_2 *ROBOT.motor[LB_MOTOR].d_torque + ROBOT.position_sensor[LB_MOTOR].w * k_I_2;
//        ROBOT.motor[LF_MOTOR].torque = error_v_2 *k_P_2 + k_D_2 *ROBOT.motor[LF_MOTOR].d_torque - ROBOT.position_sensor[LF_MOTOR].w * k_I_2;
//    }    
//}

void stable_leg(int RL,double x,double y,double w_x ,double w_y,double a,double b){
  double K_P = 24;
  double K_I = 0.04;
  double K_D = 16;

  if(RL == 0){
    LEG_solution_pos_opposite(R,w_x,w_y);

    double error_1 = ROBOT.motor[RB_MOTOR].want_P - a;
    double error_2 = ROBOT.motor[RF_MOTOR].want_P - b;

    ROBOT.motor[RB_MOTOR].I = ROBOT.motor[RB_MOTOR].I + error_1 * K_I;
    ROBOT.motor[RB_MOTOR].torque = error_1*K_P - K_D*ROBOT.position_sensor[RB_MOTOR].w + ROBOT.motor[RB_MOTOR].I;
  
    ROBOT.motor[RF_MOTOR].I = ROBOT.motor[RF_MOTOR].I + error_2 * K_I;
    ROBOT.motor[RF_MOTOR].torque = error_2*K_P + K_D*ROBOT.position_sensor[RF_MOTOR].w + ROBOT.motor[RF_MOTOR].I;
  }else if(RL == 1){
    LEG_solution_pos_opposite(L,w_x,w_y);
  
    double error_1 = ROBOT.motor[LB_MOTOR].want_P - a;
    double error_2 = ROBOT.motor[LF_MOTOR].want_P - b;
    
    ROBOT.motor[LB_MOTOR].I = ROBOT.motor[LB_MOTOR].I + error_1 * K_I;
    ROBOT.motor[LB_MOTOR].torque = error_1*K_P - K_D*ROBOT.position_sensor[LB_MOTOR].w + ROBOT.motor[LB_MOTOR].I;
  
    ROBOT.motor[LF_MOTOR].I = ROBOT.motor[LF_MOTOR].I + error_2 * K_I;
    ROBOT.motor[LF_MOTOR].torque = error_2*K_P + K_D*ROBOT.position_sensor[LF_MOTOR].w + ROBOT.motor[LF_MOTOR].I;
  }
};
//void balance_control(){
//  double 
//}



#endif