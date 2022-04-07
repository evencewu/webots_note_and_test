#ifndef _SENSOR_
#define _SENSOR_

//传感器轮询和处理数据

void MOTOR_data()//轮询所有电机传感器
{
  int i;
  for( i = 0; i < MOTOR_NUM; i++){
    assert(ROBOT.position_sensor[i].ID);//错误检测
    ROBOT.position_sensor[i].position = wb_position_sensor_get_value(ROBOT.position_sensor[i].ID);
    ROBOT.position_sensor[i].w = ROBOT.position_sensor[i].position - ROBOT.position_sensor[i].position_last;
    ROBOT.position_sensor[i].position_last = ROBOT.position_sensor[i].position;
    
    ROBOT.motor[i].torque_fb = wb_motor_get_torque_feedback(ROBOT.motor[i].ID);
  }
  //轮子里程计免了(*^_^*)               
};

void LEG_data(){
  LEG_data_pos( ROBOT.position_sensor[RB_POS].position , -ROBOT.position_sensor[RF_POS].position,
               -ROBOT.position_sensor[LB_POS].position ,  ROBOT.position_sensor[LF_POS].position);
               
}

void LEG_data_pos(double A,double B,double C,double D){
  ROBOT.leg1.Px =       - (l1*cos(A))/2 + (l1*cos(B))/2 + (l1*sin(A))/2 - (l1*sin(B))/2;
  ROBOT.leg1.Py = - d/2 - (l1*cos(A))/2 - (l1*cos(B))/2 - (l1*sin(A))/2 - (l1*sin(B))/2;
  ROBOT.leg2.Px =       - (l1*cos(C))/2 + (l1*cos(D))/2 + (l1*sin(C))/2 - (l1*sin(D))/2;
  ROBOT.leg2.Py = - d/2 - (l1*cos(C))/2 - (l1*cos(D))/2 - (l1*sin(C))/2 - (l1*sin(D))/2;
}

#endif