#ifndef _TYPE_DEF_
#define _TYPE_DEF_

#define l1 0.15
#define l2 0.31819805153394638598037996294718
#define d 0.15

#define roll  0
#define pitch 1
#define yaw   2

#define R   0
#define L   1
#define RB  2
#define LB  3
#define RF  4
#define LF  5


#define MASS_BODY 30
#define MASS_COMPONENT 0.5
#define pi  3.1415926

#define MOTOR_NUM 6

typedef struct pid_feature{
    double I;
    double error;

    double K_P;
    double K_I;
    double K_D;
}pid_feature;

typedef struct motor_feature{
  WbDeviceTag ID;
  const char *name;
  double MAX_TORQUE;

  double torque;//力矩
  double torque_fb;//读取

  double want_P;

  pid_feature pid;
}MOTOR;

//编码器参数
typedef struct position_sensor_feature{
  WbDeviceTag ID;
  const char *name;
  double resolusion;
  double position;//rad
  double position_last;//上一个时间戳
  double w;//rad/s
  double w_last;
}POSITION_SENSOR;

typedef struct LEG
{
    //足端坐标
    double Px;
    double Py;
    //足端速度
    double Vx;
    double Vy;
    //足端力（实时）
    double Fx;
    double Fy;
    //目标足端坐标
    double want_Px;
    double want_Py;
} LEG;

typedef struct robot_feature
{
    double radius_of_wheel; //m
    double height_of_center; //m
    double mass_body;  //kg
    double mass_wheel; //kg
    double velocity;  //m/s

    MOTOR motor[6];
    POSITION_SENSOR position_sensor[6];
    
    LEG leg[2];
    //IMU imu;
    //ACCE acce;
    //GYRO gyro;

} robot_feature;

void robot_init();
    void motor_init(double angle_set);
    void position_sensor_init();
    void key_mouse_init();
void flash_sensor_data();
    void key_data();
    void MOTOR_data();
    void leg_data();
        void LEG_solution_pos(int RL,double A,double B);
        void LEG_solution_pos_opposite(int RL,double X,double Y);
void control_master();
    void stable_leg(int i, double p, double I,double D);
void print_data();

void perform_motor();



#endif