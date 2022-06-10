#ifndef _SENSOR_
#define _SENSOR_

void flash_sensor_data(){
     MOTOR_data();
     leg_data();
     key_data();
}

void MOTOR_data(){//轮询所有电机传感器
    for(int i = 0; i < (MOTOR_NUM - 2); i++){
        printf("%d",i);
        assert(ROBOT.position_sensor[i].ID);//错误检测
        ROBOT.position_sensor[i].position      = wb_position_sensor_get_value(ROBOT.position_sensor[i].ID);
        ROBOT.position_sensor[i].w             = ROBOT.position_sensor[i].position - ROBOT.position_sensor[i].position_last;
        ROBOT.position_sensor[i].position_last = ROBOT.position_sensor[i].position;
        ROBOT.motor[i].torque_fb               = wb_motor_get_torque_feedback(ROBOT.motor[i].ID);
    }
    //反转方向
    for(int i = MOTOR_NUM - 2; i < MOTOR_NUM; i++){
        printf("%d",i);
        assert(ROBOT.position_sensor[i].ID);//错误检测
        ROBOT.position_sensor[i].position      = -1*(wb_position_sensor_get_value(ROBOT.position_sensor[i].ID));
        ROBOT.position_sensor[i].w             =  1*(ROBOT.position_sensor[i].position - ROBOT.position_sensor[i].position_last);
        ROBOT.position_sensor[i].position_last =  1*(ROBOT.position_sensor[i].position);
        ROBOT.motor[i].torque_fb               = -1*(wb_motor_get_torque_feedback(ROBOT.motor[i].ID));
    }

  //轮子里程计免了(*^_^*)               
};
//================================================================================================
void leg_data(){
    LEG_solution_pos(R, ROBOT.position_sensor[RB].position , ROBOT.position_sensor[RF].position);
    LEG_solution_pos(L, ROBOT.position_sensor[LB].position , ROBOT.position_sensor[LF].position);
}
//足端位置正解
void LEG_solution_pos(int RL,double A,double B){
  if(RL == 0){
    ROBOT.leg[R].Px =       - (l1*cos(A))/2 + (l1*cos(B))/2 + (l1*sin(A))/2 - (l1*sin(B))/2;
    ROBOT.leg[R].Py = - d/2 - (l1*cos(A))/2 - (l1*cos(B))/2 - (l1*sin(A))/2 - (l1*sin(B))/2;
  }else if(RL == 1){
    ROBOT.leg[L].Px =       - (l1*cos(A))/2 + (l1*cos(B))/2 + (l1*sin(A))/2 - (l1*sin(B))/2;
    ROBOT.leg[L].Py = - d/2 - (l1*cos(A))/2 - (l1*cos(B))/2 - (l1*sin(A))/2 - (l1*sin(B))/2;
  }
};

void key_data(){
  ky = wb_keyboard_get_key();
  if(ky == 74){
    if(ROBOT.leg[R].want_Py < -0.1){
      ROBOT.leg[R].want_Py = ROBOT.leg[R].want_Py + 0.001;
    }else{
      ROBOT.leg[R].want_Py = -0.1;
    };

    if(ROBOT.leg[L].want_Py < -0.1){
      ROBOT.leg[L].want_Py = ROBOT.leg[L].want_Py + 0.001;
    }else{
      ROBOT.leg[L].want_Py = -0.1;//0.088
    };

  }else if(ky == 85){
    if(ROBOT.leg[R].want_Py > -0.32){
      ROBOT.leg[R].want_Py = ROBOT.leg[R].want_Py - 0.001;
    }else{
      ROBOT.leg[R].want_Py = -0.32;
    };

    if(ROBOT.leg[L].want_Py > -0.32){
      ROBOT.leg[L].want_Py = ROBOT.leg[L].want_Py - 0.001;
    }else{
      ROBOT.leg[L].want_Py = -0.32;
    };

  }else if(ky == 72){
    if(ROBOT.leg[R].want_Px > -0.1){
      ROBOT.leg[R].want_Px = ROBOT.leg[R].want_Px - 0.001;
    }else{
      ROBOT.leg[R].want_Px = -0.1;
    };

    if(ROBOT.leg[L].want_Px > -0.1){
      ROBOT.leg[L].want_Px = ROBOT.leg[L].want_Px - 0.001;
    }else{
      ROBOT.leg[L].want_Px = -0.1;
    };

  }else if(ky == 75){
    if(ROBOT.leg[R].want_Px < 0.1){
      ROBOT.leg[R].want_Px = ROBOT.leg[R].want_Px + 0.001;
    }else{
      ROBOT.leg[R].want_Px = 0.1;
    };

    if(ROBOT.leg[L].want_Px < 0.1){
      ROBOT.leg[L].want_Px = ROBOT.leg[L].want_Px + 0.001;
    }else{
      ROBOT.leg[L].want_Px = 0.1;
    };
  }
}


#endif