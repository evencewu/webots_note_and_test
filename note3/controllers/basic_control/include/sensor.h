#ifndef _SENSOR_
#define _SENSOR_

//传感器轮询和处理数据

//电机数据刷新处理
//=====================================================================================
void MOTOR_data()//轮询所有电机传感器
{
  int i;
  for( i = 0; i < MOTOR_NUM; i++){
    assert(ROBOT.position_sensor[i].ID);//错误检测
    ROBOT.position_sensor[i].position = wb_position_sensor_get_value(ROBOT.position_sensor[i].ID);
    ROBOT.position_sensor[i].w = ROBOT.position_sensor[i].position - ROBOT.position_sensor[i].position_last;
    ROBOT.position_sensor[i].position_last = ROBOT.position_sensor[i].position;
    
    ROBOT.motor[i].torque_fb = wb_motor_get_torque_feedback(ROBOT.motor[i].ID);
    ROBOT.motor[i].d_torque = ROBOT.motor[i].torque_fb - ROBOT.motor[i].torque_last;
    ROBOT.motor[i].torque_last = ROBOT.motor[i].torque_fb;
  }
  //轮子里程计免了(*^_^*)               
};

//腿部数据刷新&处理
//=====================================================================================
void LEG_data(){
  LEG_solution_pos(R, ROBOT.position_sensor[RB_MOTOR].position , -ROBOT.position_sensor[RF_MOTOR].position); //反转
  LEG_solution_pos(L, ROBOT.position_sensor[LB_MOTOR].position , -ROBOT.position_sensor[LF_MOTOR].position);

  LEG_solution_speed(R, ROBOT.position_sensor[RB_MOTOR].position , -ROBOT.position_sensor[RF_MOTOR].position,
                        ROBOT.position_sensor[RB_MOTOR].w        , -ROBOT.position_sensor[RF_MOTOR].w);  
  LEG_solution_speed(L, ROBOT.position_sensor[LB_MOTOR].position , -ROBOT.position_sensor[LF_MOTOR].position,
                        ROBOT.position_sensor[LB_MOTOR].w        , -ROBOT.position_sensor[LF_MOTOR].w);      
};
//=====================================================================================
#endif