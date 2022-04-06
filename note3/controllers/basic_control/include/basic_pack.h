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

//eg.一组关节  一号位为左   二号位为右
/*

*/
//FINISH
//腿部位置正解
void positive_solution_leg(double *xy){
    double b = -pos[0]; //RF
    double a = pos[1]; //RB
    *xy = (l1*cos(b))/2 - (l1*cos(a))/2 + (l1*sin(a))/2 - (l1*sin(b))/2;
    *(xy+1) = - d/2 - (l1*cos(a))/2 - (l1*cos(b))/2 - (l1*sin(a))/2 - (l1*sin(b))/2;
}

//todo
//腿部力学正解（忽略质量）
void force_solution_leg(double f1,double f2,double *xy){
    double a = -pos[0]; //RF
    double b = pos[1]; //RB
    *xy = f2*sin(b) + f1*sin(a);       // FX
    *(xy+1) = f1*cos(a) - f2*cos(b); // FY
}
//todo
//腿部力学逆解（忽略质量）
void force_Insolution_leg(double fx,double fy,double *xy){
    double a = -pos[0]; //RF
    double b = pos[1]; //RB
    *xy = l1*(fx*cos(a) - fy*sin(a))/(cos(a)*sin(b) + cos(b)*sin(a));   //F1
    *(xy+1) = l1*(fx*cos(b) + fy*sin(b))/(cos(a)*sin(b) + cos(b)*sin(a));//F2
}
//todo
//腿部速度正解
void speed_solution_leg(double v1,double v2,double *xy){
    double b = -pos[0]; //RF
    double a = pos[1]; //RB
    *xy = v2*l1*sin(a) + v1*l1*sin(b);     // VX
    *(xy+1) = v1*l1*cos(b) - v2*l1*cos(a); // VY
}

//腿部位置稳定
//void stable_leg(double w_x ,double w_y,double x ,double y){
//    double error_x =  w_x - x;
//    double error_y =  w_y - y;
//    double w_v_x = error_x * k_p_1;
//    torque = (w_v_x - v_x)*k_p_2;
//    wb_motor_set_torque(motor_device[0],1.5);
//}

void master(){

    double D_xy[2];
    positive_solution_leg(&D_xy[0]);
    
    double V_xy[2];
    speed_solution_leg(velocity[1],velocity[0],&V_xy[0]);
    speed_solution_leg(3,-3,&V_xy[0]);

    double F_xy[2];
    //force_solution_leg(-3,3,&F_xy[0]);
    force_Insolution_leg(0,-6,&F_xy[0]);

    printf("Dxy %f %f \n",D_xy[0],D_xy[1]);
    printf("Vxy %f %f \n",V_xy[0],V_xy[1]);
    printf("Fxy %f %f \n",F_xy[0],F_xy[1]);

    key = wb_keyboard_get_key();
    if (key == 72){//H
        wb_motor_set_torque(motor_device[0],0.5);
        wb_motor_set_torque(motor_device[1],-0.5);
        wb_motor_set_torque(motor_device[2],0.5);
        wb_motor_set_torque(motor_device[3],-0.5);
    }else if (key == 74){
        wb_motor_set_torque(motor_device[0],-0.5);
        wb_motor_set_torque(motor_device[1],0.5);
        wb_motor_set_torque(motor_device[2],-0.5);
        wb_motor_set_torque(motor_device[3],0.5);
    }else{
        wb_motor_set_torque(motor_device[0],0);
        wb_motor_set_torque(motor_device[1],0);
        wb_motor_set_torque(motor_device[2],0);
        wb_motor_set_torque(motor_device[3],0);
    }
}

#endif