#ifndef _CONTROL_
#define _CONTROL_
//装点控制算法的代码


void LEG_solution_pos(double A,double B,double C,double D){
  ROBOT.leg1.Px =       - (l1*cos(A))/2 + (l1*cos(B))/2 + (l1*sin(A))/2 - (l1*sin(B))/2;
  ROBOT.leg1.Py = - d/2 - (l1*cos(A))/2 - (l1*cos(B))/2 - (l1*sin(A))/2 - (l1*sin(B))/2;
  ROBOT.leg2.Px =       - (l1*cos(C))/2 + (l1*cos(D))/2 + (l1*sin(C))/2 - (l1*sin(D))/2;
  ROBOT.leg2.Py = - d/2 - (l1*cos(C))/2 - (l1*cos(D))/2 - (l1*sin(C))/2 - (l1*sin(D))/2;
};

void LEG_solution_speed(double A ,double B ,double C ,double D,
                        double v1,double v2,double v3,double v4){
  ROBOT.leg1.Vx =  (l1*cos(A)*v1)/2 - (l1*sin(A)*v1)/2 + (l1*cos(B)*v2)/2 - (l1*sin(B)*v2)/2; 
  ROBOT.leg1.Vy = -(l1*cos(A)*v1)/2 + (l1*sin(A)*v1)/2 + (l1*cos(B)*v2)/2 - (l1*sin(B)*v2)/2; 
  ROBOT.leg2.Vx =  (l1*cos(C)*v3)/2 - (l1*sin(C)*v3)/2 + (l1*cos(D)*v4)/2 - (l1*sin(D)*v4)/2; 
  ROBOT.leg2.Vy = -(l1*cos(C)*v3)/2 + (l1*sin(C)*v3)/2 + (l1*cos(D)*v4)/2 - (l1*sin(D)*v4)/2; 
};

void LEG_solution_speed_opposite(double A,double B,double vx_1,double vy_1){
  ROBOT.motor[RB_MOTOR].want_V = (vx_1 - vy_1)/(l1*cos(A) - l1*sin(A)); 
  ROBOT.motor[RF_MOTOR].want_V = (vx_1 + vy_1)/(l1*cos(B) - l1*sin(B)); 
};

void stable_leg(double x,double y,double w_x ,double w_y,
                double a,double b,int RL){
    double k_p_1 = 4;
    double k_P_2 = 4;
    double k_I_2 = 0;
    double k_D_2 = 0;

    double error_x =  w_x - x;
    double error_y =  w_y - y;
    
    double w_v_x = error_x * k_p_1;
    double w_v_y = error_y * k_p_1;

    LEG_solution_speed_opposite(a,b,w_v_x,w_v_y);

    if(RL == 0){
        double error_v_1 = ROBOT.position_sensor[RB_POS].w - ROBOT.motor[RB_MOTOR].want_V;
        double error_v_2 = ROBOT.position_sensor[RF_POS].w - ROBOT.motor[RF_MOTOR].want_V;  
        ROBOT.motor[RB_MOTOR].torque = error_v_1 *k_P_2 - k_D_2 *ROBOT.motor[RB_MOTOR].d_torque;
        ROBOT.motor[RF_MOTOR].torque = error_v_2 *k_P_2 - k_D_2 *ROBOT.motor[RF_MOTOR].d_torque;
    }else if(RL == 1){
        double error_v_1 = ROBOT.position_sensor[LB_POS].w - ROBOT.motor[LB_MOTOR].want_V;
        double error_v_2 = ROBOT.position_sensor[LF_POS].w - ROBOT.motor[LF_MOTOR].want_V; 
        ROBOT.motor[LB_MOTOR].torque = error_v_1 *k_P_2 - k_D_2 *ROBOT.motor[LB_MOTOR].d_torque;
        ROBOT.motor[LF_MOTOR].torque = error_v_2 *k_P_2 - k_D_2 *ROBOT.motor[LF_MOTOR].d_torque;
    }
    
}

#endif