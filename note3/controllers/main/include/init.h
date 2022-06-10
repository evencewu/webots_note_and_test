#ifndef _INIT_
#define _INIT_

void robot_init(){
    ROBOT.leg[R].want_Py = -0.225;
    ROBOT.leg[L].want_Py = -0.225;
    ROBOT.leg[R].want_Px = 0;
    ROBOT.leg[L].want_Px = 0;

    motor_init(0);
    position_sensor_init();
    key_mouse_init();
}

//电机初始化
void motor_init(double angle_set){
  ROBOT.motor[RB].name = "RB_MOTOR";
  ROBOT.motor[RF].name = "RF_MOTOR";
  ROBOT.motor[LB].name = "LB_MOTOR";
  ROBOT.motor[LF].name = "LF_MOTOR";
  ROBOT.motor[R].name = "R_MOTOR"; //右
  ROBOT.motor[L].name = "L_MOTOR"; //左

  int i;
  for( i = 0; i < MOTOR_NUM; i++){
    //获取电机ID
    ROBOT.motor[i].ID = wb_robot_get_device(ROBOT.motor[i].name);
    assert(ROBOT.motor[i].ID);
    //获取最大扭矩
    ROBOT.motor[i].MAX_TORQUE = wb_motor_get_max_torque(ROBOT.motor[i].ID);
    //printf("max_t%f\n",ROBOT.motor[i].MAX_TORQUE);
    //使能扭矩反馈
    int sampling_period;
    sampling_period = TIME_STEP;// wb_motor_get_torque_feedback_sampling_period(ROBOT.motor[i].ID);
    wb_motor_enable_torque_feedback(ROBOT.motor[i].ID, sampling_period);
    //归零
    ROBOT.motor[i].torque = 0;
    ROBOT.motor[i].pid.I = 0;
    wb_motor_set_torque(ROBOT.motor[i].ID,0);
    //printf("get motor %s succeed: %d\n", ROBOT.motor[i].name, ROBOT.motor[i].ID);
  }

};

void position_sensor_init(){
  ROBOT.position_sensor[RB].name = "RB_POS";
  ROBOT.position_sensor[RF].name = "RF_POS";
  ROBOT.position_sensor[LB].name = "LB_POS";
  ROBOT.position_sensor[LF].name = "LF_POS";
  ROBOT.position_sensor[R].name = "R_POS";
  ROBOT.position_sensor[L].name = "L_POS"; 

  int i;
  for( i = 0; i < MOTOR_NUM; i++){
  ROBOT.position_sensor[i].ID = wb_robot_get_device(ROBOT.position_sensor[i].name);
  assert(ROBOT.position_sensor[i].ID);
  wb_position_sensor_enable(ROBOT.position_sensor[i].ID, (int)TIME_STEP);

  ROBOT.position_sensor[i].position = 0;
  ROBOT.position_sensor[i].position_last = 0;

  ROBOT.position_sensor[i].w = 0;
  ROBOT.position_sensor[i].w_last = 0;
  }
};

int ky;
void key_mouse_init(){
  wb_keyboard_enable(TIME_STEP); 
};

#endif