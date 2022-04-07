#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h> 
#include <webots/inertial_unit.h>
#include <webots/Gyro.h>
#include <webots/Compass.h>
#include <webots/Accelerometer.h>
#include <webots/position_sensor.h>
#include <stdio.h>
#include <math.h>

#ifndef _BASIC_PACK_
#define _BASIC_PACK_

//eg.一组关节  一号位为左RB   二号位为右RF
/*

*/
//FINISH
//腿部位置正解
void positive_solution_leg(double a,double b,double *xy){
    *xy =           - (l1*cos(a))/2 + (l1*cos(b))/2 + (l1*sin(a))/2 - (l1*sin(b))/2;
    *(xy+1) = - d/2 - (l1*cos(a))/2 - (l1*cos(b))/2 - (l1*sin(a))/2 - (l1*sin(b))/2;
}

//FINISH
//腿部速度正解
void speed_solution_leg(double a,double b,double v1,double v2,double *xy){//v1左 v2右方向时针互逆
    *xy =    (l1*cos(a)*v1)/2 - (l1*sin(a)*v1)/2 + (l1*cos(b)*v2)/2 - (l1*sin(b)*v2)/2; // VX
    *(xy+1) = -(l1*cos(a)*v1)/2 + (l1*sin(a)*v1)/2 + (l1*cos(b)*v2)/2 - (l1*sin(b)*v2)/2; // VY
}

//腿部速度逆解
void speed_Insolution_leg(double a,double b,double vx,double vy,double *xy){//v1左 v2右方向时针互逆
    *xy =     (vx - vy)/(l1*cos(a) - l1*sin(a)); // V1
    *(xy+1) = (vx + vy)/(l1*cos(b) - l1*sin(b)); // V2
}

//todo
//腿部力学正解（忽略质量）
//void force_solution_leg(double f1,double f2,double *xy){
//    double a = -pos[0]; //RF
//    double b = pos[1]; //RB
//    *xy = f2*sin(b) + f1*sin(a);       // FX             
//    *(xy+1) = f1*cos(a) - f2*cos(b); // FY
//}
//todo
//腿部力学逆解（忽略质量）
//void force_Insolution_leg(double fx,double fy,double *xy){
//    double a = -pos[0]; //RF
//    double b = pos[1]; //RB
//    *xy =     l1*(fx*cos(a) - fy*sin(a))/(cos(a)*sin(b) + cos(b)*sin(a));   //F1
//    *(xy+1) = l1*(fx*cos(b) + fy*sin(b))/(cos(a)*sin(b) + cos(b)*sin(a));//F2
//}


//腿部位置稳定
void stable_leg(double a,double b,double w_x ,double w_y,double x ,double y,double v1,double v2,double *F_motor){
    double k_p = 4;
    double error_x =  w_x - x;
    double error_y =  w_y - y;
    double w_v_x = error_x * k_p;
    double w_v_y = error_y * k_p;
    double V_motor[2];
    speed_Insolution_leg(a,b,w_v_x,w_v_y,&V_motor[0]);
    double error_v_1 = v2 - V_motor[0];
    double error_v_2 = v1 - V_motor[1];
    double Kp = 8;
    *F_motor     = error_v_1 *Kp;
    *(F_motor+1) = error_v_2 *Kp;
    printf("----------------------\n");
    printf("xy %f %f\n",x,y);
    printf("wxy %f %f\n",w_x,w_y);
    printf("v %f %f\n",w_v_x,w_v_y);
    
}

void key_input(){
    key = wb_keyboard_get_key();
    if (key == 72){//H
        want_y_R = want_y_R + 0.005;
        want_y_L = want_y_L + 0.005;
    }else if (key == 74){
        want_y_R = want_y_R - 0.005;
        want_y_L = want_y_L - 0.005;
    }
    else{
        want_y_R = want_y_R;
        want_y_L = want_y_L;
    }
}

void master(){
    
    key_input();
    printf("YRL %f %f \n",want_y_R,want_y_L);
    //腿部姿态解算
    double R_xy[2];
    double L_xy[2];
    positive_solution_leg(pos[1],-pos[0],&R_xy[0]);
    positive_solution_leg(pos[3],-pos[2],&L_xy[0]);
    //printf("R_xy %f %f \n",R_xy[0],R_xy[1]);
    //printf("L_xy %f %f \n",L_xy[0],L_xy[1]);
    
    //double V_R[2];
    //double V_L[2];
    //speed_solution_leg(R,pos[1],-pos[0],velocity[1],velocity[0],&V_R[0]);
    //speed_solution_leg(L,pos[3],-pos[2],velocity[3],velocity[2],&V_L[0]);
    //printf("V_R: %f %f \n",V_R[0],V_R[1]);
    //printf("V_L: %f %f \n",V_L[0],V_L[1]);

    //double F_xy[2];
    //force_solution_leg(-3,3,&F_xy[0]);
    //force_Insolution_leg(0,-6,&F_xy[0]);

    //腿部速度解算+执行
    double F_motor[4];
    stable_leg(pos[1], -pos[0], want_x_R, want_y_R, R_xy[0], R_xy[1], velocity[1], velocity[0], &F_motor[0]);
    stable_leg(pos[3], -pos[2], want_x_L, want_y_L, L_xy[0], L_xy[1], velocity[3], velocity[2], &F_motor[2]);
    printf("%f %f %f %f\n",F_motor[0],F_motor[1],F_motor[2],F_motor[3]);

    //wb_motor_set_available_torque(motor_device[0],F_motor[0]);
    //wb_motor_set_available_torque(motor_device[1],F_motor[1]);
    //wb_motor_set_available_torque(motor_device[2],F_motor[2]);
    //wb_motor_set_available_torque(motor_device[3],F_motor[3]);
    
    wb_motor_set_available_torque(motor_device[0],-50);
    wb_motor_set_available_torque(motor_device[1], 50);
    wb_motor_set_available_torque(motor_device[2],-50);
    wb_motor_set_available_torque(motor_device[3], 50);
    
    
    
    //printf("Fxy %f %f \n",F_xy[0],F_xy[1]);


}

#endif