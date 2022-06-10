#ifndef _CONTROL_
#define _CONTROL_
void control_master(){
    LEG_solution_pos_opposite(R,ROBOT.leg[R].want_Px,ROBOT.leg[R].want_Py);
    LEG_solution_pos_opposite(L,ROBOT.leg[L].want_Px,ROBOT.leg[L].want_Py);

    stable_leg(RF,12,0.3,0);
    stable_leg(RB,12,0.3,0);
    stable_leg(LF,12,0.3,0);
    stable_leg(LB,12,0.3,0);
};
                                                    
void LEG_solution_pos_opposite(int RL,double X,double Y){
  if(RL == 0){  
    ROBOT.motor[RF].want_P = pi - acos((pow(Y,2) + pow(l1,2) - pow(l2,2) + pow(d/2 - X,2))/(2*l1*sqrt(pow(Y,2) + pow(d/2 - X,2)))) - atan(Y/(X - d/2));
    ROBOT.motor[RB].want_P = pi - acos((pow(Y,2) + pow(l1,2) - pow(l2,2) + pow(X - d/2,2))/(2*l1*sqrt(pow(Y,2) + pow(X - d/2,2)))) - atan(Y/(X - d/2));  
  }else if(RL == 1){
    ROBOT.motor[LF].want_P = pi - acos((pow(Y,2) + pow(l1,2) - pow(l2,2) + pow(d/2 - X,2))/(2*l1*sqrt(pow(Y,2) + pow(d/2 - X,2)))) - atan(Y/(X - d/2));
    ROBOT.motor[LB].want_P = pi - acos((pow(Y,2) + pow(l1,2) - pow(l2,2) + pow(X - d/2,2))/(2*l1*sqrt(pow(Y,2) + pow(X - d/2,2)))) - atan(Y/(X - d/2));
  }
};

void stable_leg(int i, double p, double I,double D){
    ROBOT.motor[i].pid.K_P = p;
    ROBOT.motor[i].pid.K_I = I;
    ROBOT.motor[i].pid.K_D = D;

    ROBOT.motor[i].pid.error = ROBOT.motor[i].want_P - ROBOT.position_sensor[i].position;
    ROBOT.motor[i].pid.I     = ROBOT.motor[i].pid.I + ROBOT.motor[i].pid.error * ROBOT.motor[i].pid.K_I;
    ROBOT.motor[i].torque    = ROBOT.motor[i].pid.error * ROBOT.motor[i].pid.K_P - ROBOT.motor[i].pid.K_D * ROBOT.position_sensor[i].w + ROBOT.motor[i].pid.I ;
    //printf("%f\n",ROBOT.motor[i].pid.error * ROBOT.motor[i].pid.K_P);
    //printf("%f\n",- ROBOT.motor[i].pid.K_D * ROBOT.position_sensor[i].w);
    //printf("%f\n",ROBOT.motor[i].pid.I);
};

 
#endif