/* 
 *  Planar Odometry 
 */
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

//#include <common_smart_nav/ArduGoal.h>

#include <Servo.h>


#include <stdio.h>
// Other includes
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>

/* Wait function */
void delay_ms(uint16_t millis)
{
    while (millis) {
        _delay_ms(1);
        millis--;
    }
}

/***********/
/* Defines */
/***********/
#define TICK_PER_MM_LEFT 	(18.6256756)
#define TICK_PER_MM_RIGHT 	(18.6256756)
#define TICK_PER_M_LEFT 	(17270.0)//(18205.6756)
#define TICK_PER_M_RIGHT 	(17270.0)//(18205.6756)
//#define DIAMETER 	        0.19950//0.19814//0.1970 //0.2951                       // Distance between the 2 wheels
double DIAMETER = 0.1980;

#define DISTANCE_REAR_WHEELS    0.055

#define CORFUGE             1.2

#define TWOPI 			    6.2831853070
#define RAD2DEG 		    57.2958     /* radians to degrees conversion */

#define ENC_TICKS 100
#define QUAD_TICKS (4*ENC_TICKS)
#define GEAR_REDUCTION (50)
#define TOTAL_TICKS (GEAR_REDUCTION * QUAD_TICKS)

#define WHEEL_DIAMETER 0.07
#define WHEEL_PERIMETER (WHEEL_DIAMETER * 3.14159265)

#define WHEEL_DISTANCE 0.230
#define TURN_PERIMETER (WHEEL_DISTANCE * 3.14159265)

// Types of motor
#define ALPHA_MOTOR 		0
#define DELTA_MOTOR 		1
#define LEFT_MOTOR 		2
#define RIGHT_MOTOR 		3

#define ALPHADELTA 		0
#define LEFTRIGHT 		1

#define COMMAND_DONE 		0
#define PROCESSING_COMMAND 	1
#define WAITING_BEGIN 		2
#define ERROR 			3

volatile signed long ALPHA_MAX_SPEED    =     40000;//13000//3000//13000 
#define ALPHA_MAX_ACCEL         30000//32000//300//800
#define ALPHA_MAX_DECEL         64000//500//1600 
#define ALPHA_MIN_SPEED         6000 
#define ALPHA_MIN_ERROR_ZERO    4 
volatile signed long DELTA_MAX_SPEED    =     50000;//23000//4000//23000  
#define DELTA_MAX_ACCEL         10000//40000//300//1000     
#define DELTA_MAX_DECEL         88000//600//2200 
#define DELTA_MIN_SPEED         7000//20000  
#define DELTA_MIN_ERROR_ZERO    2  

#define ALPHA_P         	200//200//400//1200
#define ALPHA_I         	0//25//50
#define ALPHA_D         	100//500//2000//8000
#define DELTA_P         	2000//1200//5000
#define DELTA_I         	0
#define DELTA_D         	1000//2000//4000

#define POWER_TO_RIGHT          1.25

//#define PATH_FOLLOWING          1
#define FORWARD                 0
#define REVERSE                 1

#define LEFT_REAR_SENSOR        38
#define RIGHT_REAR_SENSOR       40

#define LEFT_SERVO		34
#define RIGHT_SERVO		32

#define START_PIN               40

/***********************/
/* Specific structures */
/***********************/
struct motor {
    int type;
    volatile signed long des_speed;
    signed long cur_speed;
    long last_error;
    long error_sum;
    int kP;
    int kI;
    int kD;
    signed long accel;
    signed long decel;
    signed long max_speed;
    signed long min_speed;
    signed long min_error_zero;
    float distance;
};

struct robot {
    double pos_X;
    double pos_Y;
    double theta;
    double desX;
    double desY;
    double desTheta;
};

struct RobotCommand {
    char state;
    double current_distance;
    double desired_distance;
};

struct Point {
    int x;
    int y;
};

/********************/
/* Global variables */
/********************/
struct motor left_motor;
struct motor right_motor;
struct motor alpha_motor;
struct motor delta_motor;

struct robot maximus;

struct RobotCommand bot_command_delta;
struct RobotCommand prev_bot_command_delta;
struct RobotCommand bot_command_alpha;

volatile long left_cnt = 0;
volatile long right_cnt = 0;

char output_ON = 0;
char roboclaw_ON = 0;
char motion_control_ON = 1;
int8_t color = 1;//-1;

int last_left = 0;
int last_right = 0;

int left_diff = 0;
int right_diff = 0;

float total_distance = 0.0;
float total_theta = 0.0;
float prev_total_distance = 0.0;
float prev_theta = 0.0;

unsigned int entier;
char display1, display2, display3, display4, display5, display6, display7;

char serial_command;

int front_distance = 50;
int prev_front_distance = 50;

long global_time_counter = 0;

struct Point *my_color_points;

struct Point way_points[20];
int way_point_index = 0;

int cpt_alpha = 0;

int cpt_output = 0;

int reading = 0;

float tempfloat;

volatile char transmit_status = 1;      // 1 if OK / 0 if not finished

#define ACCURACY 2

struct Point prev_position;

int global_cpt = 0;

uint8_t cpt_asserv = 0;
int alpha_and_theta = 0;
int last_goal = 0;
double desired_last_theta = 0;
int reverse_mode = 0;

double ticks_per_m = TOTAL_TICKS / WHEEL_PERIMETER;
double m_per_tick = WHEEL_PERIMETER / TOTAL_TICKS;



Servo left_servo;  // create servo object to control a servo
Servo right_servo;  // create servo object to control a servo


double ticks_per_rad = (TOTAL_TICKS / TURN_PERIMETER) / TWOPI;
double rad_per_tick = 1 / ticks_per_rad;

void rotate(double heading, double attitude, double bank,
            geometry_msgs::Quaternion * pose);

/****************************/
/* INITIALIZATION FUNCTIONS */
/****************************/
void init_Robot(struct robot *my_robot);
void init_Command(struct RobotCommand *cmd);
void init_motors(void);

/***********************/
/* ROBO CLAW FUNCTIONS */
/***********************/
// Used to change the speed value of motor 1
void write_RoboClaw_speed_M1(char addr, signed long speed);
// Used to change the speed value of motor 2
void write_RoboClaw_speed_M2(char addr, signed long speed);
// Used to change the speed value of motors 1 and 2
void write_RoboClaw_speed_M1M2(char addr, signed long speedM1,
                               signed long speedM2);
// Used to change the speed value of motor 1 and 2 during a specific distance
void write_RoboClaw_speed_dist_M1M2(char addr, signed long speedM1,
                                    signed long distanceM1,
                                    signed long speedM2,
                                    signed long distanceM2);
// Used to change the speed value of motor 1 and 2 during a specific distance with a specific acceleration
void
write_RoboClaw_allcmd_M1M2(char addr, signed long accel,
                           signed long speedM1, signed long distanceM1,
                           signed long speedM2, signed long distanceM2);

// Used to change the speed value of motors 1 and 2
void write_SaberTooth_speed_M1M2(char addr, signed long speedM1,
                                 signed long speedM2);

/************************/
/* CONVERSION FUNCTIONS */
/************************/
signed long convert_dist2ticks(signed long distance);
signed long convert_ticks2dist(signed long ticks);

/********************/
/* MOTORS FUNCTIONS */
/********************/
void move_motors(char type);
void update_motor(struct motor *used_motor);

/********************************/
/* POSITION ESTIMATION FUNCTION */
/********************************/

/* Compute the position of the robot */
void get_Odometers(void);



ros::NodeHandle nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

//geometry_msgs::Pose2D goal;
int goals_index = 0;

double x = 1.0;
double y = 0.0;
double theta = 1.57;

char base_link[] = "/petit_base_link";
char odom[] = "/petit_odom";


void colorCb(const std_msgs::Int32 & msg)
{
  if(msg.data == 1) {
    color = 1;
  }
  else {
    color = -1;
  }

}

ros::Subscriber < std_msgs::Int32 > ros_color("color", &colorCb);

void leftServoCb(const std_msgs::UInt16 & msg)
{
  left_servo.write(msg.data);
}
ros::Subscriber < std_msgs::UInt16 > left_servo_ros("left_servo", &leftServoCb);

void rightServoCb(const std_msgs::UInt16 & msg)
{
  right_servo.write(msg.data);
}
ros::Subscriber < std_msgs::UInt16 > right_servo_ros("right_servo", &rightServoCb);

void calibrateCb(const std_msgs::Int32 & msg)
{
  switch(msg.data) {
    case 0:
      calibrate_x_min(&maximus);
      break;
    case 1:
      calibrate_y_min(&maximus);
      break;
    case 2:
      calibrate_x_max(&maximus);
      break;
    case 3:
      calibrate_y_max(&maximus);
      break;
    default:
      calibrate_x_min(&maximus);
      break;
  }    
}
ros::Subscriber < std_msgs::Int32 > calibrate_ros("calibrate", &calibrateCb);

void setDiameterCb(const std_msgs::Float32 & msg)
{
  DIAMETER = msg.data;
}
ros::Subscriber < std_msgs::Float32 > set_diameter_ros("setDiameter", &setDiameterCb);

std_msgs::Empty start_message;
ros::Publisher start_pub("start_match", &start_message);
std_msgs::Bool bool_message;
ros::Publisher pub_calib_done("calib_done", &bool_message);


/***********************/
/* INTERRUPT FUNCTIONS */
/***********************/

// External Interrupt 4 service routine => PIN2
ISR(INT4_vect)
{
    //#asm("cli")
    if ((PINB & 0x10) != 0) {
        if ((PINE & 0x10) != 0)
            left_cnt--;
        else
            left_cnt++;
    } else {
        if ((PINE & 0x10) == 0)
            left_cnt--;
        else
            left_cnt++;
    }

    //#asm("sei")
}

// External Interrupt 5 service routine => PIN3
ISR(INT5_vect)
{
    if ((PINK & 0x80) != 0) {
        if ((PINE & 0x20) != 0)
            right_cnt++;
        else
            right_cnt--;
    } else {
        if ((PINE & 0x20) == 0)
            right_cnt++;
        else
            right_cnt--;
    }

}

// Pin change 0-7 interrupt service routine => PIN10
ISR(PCINT0_vect)
{
    if ((PINE & 0x10) != 0) {
        if ((PINB & 0x10) != 0) {
            left_cnt++;
        } else
            left_cnt--;
    } else {
        if ((PINB & 0x10) == 0) {
            left_cnt++;
        } else
            left_cnt--;
    }

}

// Pin change 16-23 interrupt service routine => PIN-ADC15
ISR(PCINT2_vect)
{
    if ((PINE & 0x20) != 0) {
        if ((PINK & 0x80) != 0)
            right_cnt--;
        else
            right_cnt++;
    } else {
        if ((PINK & 0x80) == 0)
            right_cnt--;
        else
            right_cnt++;
    }

}

// Timer 1 overflow interrupt service routine
ISR(TIMER1_OVF_vect)
{
    sei();                      // enable interrupts
    get_Odometers();
/*
    do_motion_control();
    if ((transmit_status) == 1)
      move_motors(ALPHADELTA);
*/
/*
    if (cpt_asserv > 3) {
        if (motion_control_ON == 1) {
            do_motion_control();
            if (roboclaw_ON == 1)
                if ((transmit_status) == 1)
                    move_motors(ALPHADELTA);    // Update the motor speed
        } //else {
          //  if (roboclaw_ON == 1)
          //      move_motors(LEFTRIGHT); // Update the motor speed
        //}
        cpt_asserv = 0;
    } else {
        cpt_asserv++;
    }
*/
}

void setup()
{

    // Input/Output Ports initialization
    // Port A initialization
    // Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In
    // State7=T State6=T State5=T State4=T State3=T State2=T State1=T State0=T
    PORTA = 0x00;
    DDRA = 0x00;

    // Port B initialization
    // Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=Out
    // State7=T State6=T State5=T State4=T State3=T State2=T State1=T State0=T
    PORTB = 0x00;
    DDRB = 0x00;

    // Port C initialization
    // Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In
    // State7=T State6=T State5=T State4=T State3=T State2=T State1=T State0=T
    PORTC = 0x00;
    DDRC = 0x00;

    // Port D initialization
    // Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In
    // State7=T State6=T State5=T State4=T State3=T State2=T State1=T State0=T
    PORTD = 0x00;
    DDRD = 0x00;

    // Port E initialization
    // Func2=In Func1=In Func0=In
    // State2=T State1=T State0=T
    PORTE = 0x00;
    DDRE = 0x00;

    PORTK = 0x00;
    DDRK = 0x00;

    pinMode(13, OUTPUT);


    // Timer/Counter 1 initialization
    // Clock source: System Clock
    // Clock value: 62,500 kHz
    // Mode: Ph. correct PWM top=00FFh
    // OC1A output: Discon.
    // OC1B output: Discon.
    // OC1C output: Discon.
    // Noise Canceler: Off
    // Input Capture on Falling Edge
    // Timer 1 Overflow Interrupt: On
    // Input Capture Interrupt: Off
    // Compare A Match Interrupt: Off
    // Compare B Match Interrupt: Off
    // Compare C Match Interrupt: Off
    TCCR1A = 0x01;
    TCCR1B = 0x03;              // 4
    TCNT1H = 0x00;
    TCNT1L = 0x00;
    ICR1H = 0x00;
    ICR1L = 0x00;
    OCR1AH = 0x00;
    OCR1AL = 0x00;
    OCR1BH = 0x00;
    OCR1BL = 0x00;
    OCR1CH = 0x00;
    OCR1CL = 0x00;

    // External Interrupt(s) initialization
    EICRA = 0x00;
    EICRB = 0x05;
    EIMSK = 0x30;
    EIFR = 0x30;
    // Interrupt on PCINT
    PCICR = 0x05;
    PCIFR = 0x05;
    PCMSK0 = 0x10;
    PCMSK1 = 0x00;
    PCMSK2 = 0x80;


    //ETIMSK=0x00;

    Serial2.begin(38400);

    // Timer(s)/Counter(s) Interrupt(s) initialization
    TIMSK1 |= 0x01;
    TIFR1 |= 0x01;


    /******************************/
    /* Initialization of the code */
    /******************************/
    //write_RoboClaw_speed_M1M2(128, 0, 0);

    //Serial2.print(170, BYTE); // Init baudrate of Sabertooth

    init_motors();              // Init motors
    init_Robot(&maximus);       // Init robot status

    init_Command(&bot_command_delta);   // Init robot command
    init_Command(&bot_command_alpha);   // Init robot command


    //write_RoboClaw_PID_M1(129, 150384, 80005, 60076, 44000); // 94000 // 120000
    //write_RoboClaw_PID_M2(129, 150384, 80005, 60076, 44000);

    // Global enable interrupts
    sei();
    
    

    pinMode(LEFT_REAR_SENSOR, INPUT);
    pinMode(RIGHT_REAR_SENSOR, INPUT);
    digitalWrite(LEFT_REAR_SENSOR, HIGH);
    digitalWrite(RIGHT_REAR_SENSOR, HIGH); 
    pinMode(START_PIN, INPUT);


    pinMode(9, OUTPUT);
    digitalWrite(9, LOW);
    pinMode(8, OUTPUT);
    digitalWrite(8, HIGH);

    pinMode(6, OUTPUT);
    digitalWrite(6, LOW);
    pinMode(7, OUTPUT);
    digitalWrite(7, HIGH);

    right_servo.attach(RIGHT_SERVO);
    left_servo.attach(LEFT_SERVO);

    nh.initNode();
    broadcaster.init(nh);

    nh.subscribe(ros_color);

    nh.subscribe(left_servo_ros);
    nh.subscribe(right_servo_ros);
    
    nh.subscribe(calibrate_ros);
        
    nh.subscribe(set_diameter_ros);
    
    nh.advertise(start_pub);
    nh.advertise(pub_calib_done);
    
      
/*  color = 0;
    while(color == 0) { // color not choose  
      nh.spinOnce();
      delay(10);
    }
  */
    // Enable motion control for auto init
    //alpha_and_theta = 0;
    //unset_alpha_theta_mode();
    motion_control_ON = 1;
    roboclaw_ON = 1;
    
    // auto init
    color = 1;//-1;
    //init_first_position(&maximus); 
    left_servo.write(75); 
    //bad_init_position(&maximus);
    left_servo.write(95);

    // Disable motion control
    motion_control_ON = 0;
    roboclaw_ON = 0;
    //alpha_and_theta = 1;
    //set_alpha_theta_mode();

    motion_control_ON = 1;
    roboclaw_ON = 1;
/*
    if ( digitalRead(START_PIN) == 0 ) {

      while ( digitalRead(START_PIN) == 0 ) {

         delay(30);
  
      }

    }
    
    while ( digitalRead(START_PIN) == 1 ) {
     
      t.header.frame_id = odom;
      t.child_frame_id = base_link;
  
      t.transform.translation.x = maximus.pos_X;
      t.transform.translation.y = maximus.pos_Y;
  
      t.transform.rotation = tf::createQuaternionFromYaw(maximus.theta);
      t.header.stamp = nh.now();
  
      broadcaster.sendTransform(t);
  
      nh.spinOnce();
      delay(10);
  
      nh.spinOnce();
      delay(10);
  
      nh.spinOnce();
      delay(10);
        
    }
*/
//    start_pub.publish(&start_message);
//    start_pub.publish(&start_message);
    
}

void loop()
{
    // tf odom->base_link
    t.header.frame_id = odom;
    t.child_frame_id = base_link;

    t.transform.translation.x = maximus.pos_X;
    t.transform.translation.y = maximus.pos_Y;

    t.transform.rotation = tf::createQuaternionFromYaw(maximus.theta);
    t.header.stamp = nh.now();

    broadcaster.sendTransform(t);

    nh.spinOnce();
    delay(5);

    nh.spinOnce();
    delay(10);

    nh.spinOnce();
    delay(10);
/*
    nh.spinOnce();
    delay(10);
*/

}


/****************************/
/* INITIALIZATION FUNCTIONS */
/****************************/
void init_Robot(struct robot *my_robot)
{
    my_robot->pos_X = 0;        // -700
    my_robot->pos_Y = 0.0;     // 700
    my_robot->theta = 0;        //PI/2;                                   // PI/2
    my_robot->desX = 0;
    my_robot->desY = 0;
    my_robot->desTheta = 0;     //PI/2;
}

void init_blue_Robot(struct robot *my_robot)
{
    my_robot->pos_X = 3000;     //-1459;                               // -700
    my_robot->pos_Y = 3000;     // 192
    my_robot->theta = 0;        // 0
    my_robot->desX = 3000;      // - 1459
    my_robot->desY = 3000;      // 192
    my_robot->desTheta = 0.0;
}                               // 0

void init_red_Robot(struct robot *my_robot)
{
    my_robot->pos_X = 1459;     // -700
    my_robot->pos_Y = 192;      // 700
    my_robot->theta = PI;       // PI/2
    my_robot->desX = 1459;
    my_robot->desY = 192;
    my_robot->desTheta = PI;
}

void init_Command(struct RobotCommand *cmd)
{
    cmd->state = COMMAND_DONE;
    cmd->current_distance = 0;
    cmd->desired_distance = 0;
}

void init_motors(void)
{
    /* Left motor initialization */
    left_motor.type = LEFT_MOTOR;
    left_motor.des_speed = 0;
    left_motor.cur_speed = 0;
    left_motor.last_error = 0;
    left_motor.error_sum = 0;
    left_motor.kP = 12;
    left_motor.kI = 6;
    left_motor.kD = 1;
    left_motor.accel = 5;
    left_motor.decel = 5;
    left_motor.max_speed = 30;
    left_motor.min_speed 	= 0;
    left_motor.min_error_zero 	= 0;
    left_motor.distance = 0.0;

    /* Right motor initialization */
    right_motor.type = RIGHT_MOTOR;
    right_motor.des_speed = 0;
    right_motor.cur_speed = 0;
    right_motor.last_error = 0;
    right_motor.error_sum = 0;
    right_motor.kP = 12;
    right_motor.kI = 6;
    right_motor.kD = 1;
    right_motor.accel = 5;
    right_motor.decel = 5;
    right_motor.max_speed = 30;
    right_motor.min_speed 	= 0;
    right_motor.min_error_zero 	= 0;
    right_motor.distance = 0.0;

    /* Alpha motor initialization */
    alpha_motor.type 		= ALPHA_MOTOR;
    alpha_motor.des_speed 	= 0;
    alpha_motor.cur_speed 	= 0;
    alpha_motor.last_error 	= 0;
    alpha_motor.error_sum 	= 0;
    alpha_motor.kP 		= ALPHA_P; 
    alpha_motor.kI 		= ALPHA_I;
    alpha_motor.kD 		= ALPHA_D; 
    alpha_motor.accel 		= ALPHA_MAX_ACCEL;
    alpha_motor.decel 		= ALPHA_MAX_DECEL;
    alpha_motor.max_speed 	= ALPHA_MAX_SPEED;
    alpha_motor.min_speed 	= ALPHA_MIN_SPEED;
    alpha_motor.min_error_zero 	= ALPHA_MIN_ERROR_ZERO; 
    alpha_motor.distance 	= 0.0;

    /* Delta motor initialization */
    delta_motor.type 		= DELTA_MOTOR;
    delta_motor.des_speed 	= 0;
    delta_motor.cur_speed 	= 0;
    delta_motor.last_error 	= 0;
    delta_motor.error_sum 	= 0;
    delta_motor.kP 		= DELTA_P;
    delta_motor.kI 		= DELTA_I;
    delta_motor.kD 		= DELTA_D;
    delta_motor.accel 		= DELTA_MAX_ACCEL;
    delta_motor.decel 		= DELTA_MAX_DECEL;
    delta_motor.max_speed 	= DELTA_MAX_SPEED;
    delta_motor.min_speed       = DELTA_MIN_SPEED;
    delta_motor.min_error_zero  = DELTA_MIN_ERROR_ZERO;
    delta_motor.distance 	= 0.0;
}

void bad_init_position(struct robot *my_robot)
{
    
}

void calibrate_x_min(struct robot *my_robot)
{
    std_msgs::Bool succeed;
    int i = 0;
    
    while ( (i < 50) && ((digitalRead(LEFT_REAR_SENSOR) == 1) || (digitalRead(RIGHT_REAR_SENSOR) == 1)) ) {
        delay(100);
        t.header.frame_id = odom;
        t.child_frame_id = base_link;
    
        t.transform.translation.x = maximus.pos_X;
        t.transform.translation.y = maximus.pos_Y;
    
        t.transform.rotation = tf::createQuaternionFromYaw(maximus.theta);
        t.header.stamp = nh.now();
    
        broadcaster.sendTransform(t);
        nh.spinOnce();
        i++;
    }
    
    delay(300);
    
    // Set the Y position and theta
    my_robot->theta = 0;
    //my_robot->pos_Y = 0.000 + DISTANCE_REAR_WHEELS;
    my_robot->pos_X = -1.500 + DISTANCE_REAR_WHEELS; 
        
    if(i>=50)
    {
      succeed.data = false;
    }
    else
    {
      succeed.data = true;
    }
    pub_calib_done.publish(&succeed);        
        
        
}

void calibrate_y_min(struct robot *my_robot)
{
    std_msgs::Bool succeed;
    int i = 0;
    
    while ( (i < 50) && ((digitalRead(LEFT_REAR_SENSOR) == 1) || (digitalRead(RIGHT_REAR_SENSOR) == 1)) ) {
        delay(100);
        t.header.frame_id = odom;
        t.child_frame_id = base_link;
    
        t.transform.translation.x = maximus.pos_X;
        t.transform.translation.y = maximus.pos_Y;
    
        t.transform.rotation = tf::createQuaternionFromYaw(maximus.theta);
        t.header.stamp = nh.now();
    
        broadcaster.sendTransform(t);
        nh.spinOnce();
        i++;
    }
    
    delay(300);
    
    // Set the Y position and theta
    my_robot->theta = 1.570796;
    my_robot->pos_Y = 0.000 + DISTANCE_REAR_WHEELS;
    //my_robot->pos_X = -1.500 + DISTANCE_REAR_WHEELS; 
        
    if(i>=50)
    {
      succeed.data = false;
    }
    else
    {
      succeed.data = true;
    }
    pub_calib_done.publish(&succeed);        
        
        
}

void calibrate_x_max(struct robot *my_robot)
{
    std_msgs::Bool succeed;
    int i = 0;
    
    while ( (i < 50) && ((digitalRead(LEFT_REAR_SENSOR) == 1) || (digitalRead(RIGHT_REAR_SENSOR) == 1)) ) {
        delay(100);
        t.header.frame_id = odom;
        t.child_frame_id = base_link;
    
        t.transform.translation.x = maximus.pos_X;
        t.transform.translation.y = maximus.pos_Y;
    
        t.transform.rotation = tf::createQuaternionFromYaw(maximus.theta);
        t.header.stamp = nh.now();
    
        broadcaster.sendTransform(t);
        nh.spinOnce();
        i++;
    }
    
    delay(300);
    
    // Set the Y position and theta
    my_robot->theta = 3.1415926;
    //my_robot->pos_Y = 0.000 + DISTANCE_REAR_WHEELS;
    my_robot->pos_X = 1.500 - DISTANCE_REAR_WHEELS; 
        
    if(i>=50)
    {
      succeed.data = false;
    }
    else
    {
      succeed.data = true;
    }
    pub_calib_done.publish(&succeed);        
        
        
}

void calibrate_y_max(struct robot *my_robot)
{
    std_msgs::Bool succeed;
    int i = 0;
    
    while ( (i < 50) && ((digitalRead(LEFT_REAR_SENSOR) == 1) || (digitalRead(RIGHT_REAR_SENSOR) == 1)) ) {
        delay(100);
        t.header.frame_id = odom;
        t.child_frame_id = base_link;
    
        t.transform.translation.x = maximus.pos_X;
        t.transform.translation.y = maximus.pos_Y;
    
        t.transform.rotation = tf::createQuaternionFromYaw(maximus.theta);
        t.header.stamp = nh.now();
    
        broadcaster.sendTransform(t);
        nh.spinOnce();
        i++;
    }
    
    delay(300);
    
    // Set the Y position and theta
    my_robot->theta = -1.570796;
    my_robot->pos_Y = 2.000 - DISTANCE_REAR_WHEELS;
    //my_robot->pos_X = -1.500 + DISTANCE_REAR_WHEELS; 
        
    if(i>=50)
    {
      succeed.data = false;
    }
    else
    {
      succeed.data = true;
    }
    pub_calib_done.publish(&succeed);        
        
        
}


void init_first_position(struct robot *my_robot)
{
 
}






// Used to change the speed value of motors 1 and 2
void write_SaberTooth_speed_M1M2(char addr, signed long speedM1,
                                 signed long speedM2)
{
    char checkSUM1, checkSUM2;
    char m1, m2;

    m1 = (char) (((speedM1 * 64) / 40000) + 64);
    m2 = (char) (((speedM2 * 64) / 40000) + 192);

    // Motor 1
    Serial2.print((char)m1);

    // Motor 2
    Serial2.print((char)m2);

}



/************************/
/* CONVERSION FUNCTIONS */
/************************/
signed long convert_dist2ticks(signed long distance)
{
    return (distance * TICK_PER_MM_RIGHT);
}

signed long convert_ticks2dist(signed long ticks)
{
    return (ticks / TICK_PER_MM_RIGHT);
}

/********************/
/* MOTORS FUNCTIONS */
/********************/
inline void move_motors(char type)
{

}

void update_motor(struct motor *used_motor)
{
    switch (used_motor->type) {
    case LEFT_MOTOR:
        used_motor->cur_speed = left_diff;
        break;
    case RIGHT_MOTOR:
        used_motor->cur_speed = right_diff;
        break;
    case ALPHA_MOTOR:
        used_motor->cur_speed = left_diff - right_diff;
        break;
    case DELTA_MOTOR:
        used_motor->cur_speed = (left_diff + right_diff) / 2;
        break;
    default:
        break;
    }
}


/********************************/
/* POSITION ESTIMATION FUNCTION */
/********************************/

/* Compute the position of the robot */
inline void get_Odometers(void)
{
    long left_wheel = 0;
    long right_wheel = 0;

    double left_mm = 0.0;
    double right_mm = 0.0;

    double distance = 0.0;
    double orient = 0.0;
    double mean_orient = 0.0;

    // Le coef Gargamel
    double K = 1;
    //double derive_x, derive_y;
    double dx, dy;

    left_wheel = left_cnt;
    right_wheel = right_cnt;

    left_diff = last_left - left_wheel;
    right_diff = last_right - right_wheel;

    last_left = left_wheel;
    last_right = right_wheel;
/*
    left_mm = ((double) left_diff) / TICK_PER_M_LEFT;
    right_mm = ((double) right_diff) / TICK_PER_M_LEFT;

    distance = (left_mm + right_mm) / 2;
*/
    //total_distance += distance;
    
    distance = (double)((left_diff + right_diff)) / 2;
    
    bot_command_delta.current_distance += (distance / TICK_PER_M_LEFT);

    orient = (right_diff - left_diff) / (DIAMETER * TICK_PER_M_LEFT);
    mean_orient = maximus.theta + (orient / 2.0);
    if (mean_orient > PI)
        mean_orient -= TWOPI;
    if (mean_orient < (-PI))
        mean_orient += TWOPI;

    maximus.theta += orient;
    if (maximus.theta > PI)
        maximus.theta -= TWOPI;
    if (maximus.theta < (-PI))
        maximus.theta += TWOPI;

    //total_theta += orient;

    if (orient == 0) // Pour éviter la division par zéro
        K=1;
    else
        K= ( sin(orient / 2) ) / (orient /2);

    dy = K * distance * sin(mean_orient);
    dx = K * distance * cos(mean_orient);

/*
    derive_x =   CORFUGE * orient * dy;
    derive_y = - CORFUGE * orient * dx;
*/

    maximus.pos_Y = maximus.pos_Y + (dy / TICK_PER_M_LEFT); // + derive_y;
    maximus.pos_X = maximus.pos_X + (dx / TICK_PER_M_LEFT); // + derive_x;

    bot_command_alpha.current_distance += orient;

//    update_motor(&left_motor);
//    update_motor(&right_motor);
//    update_motor(&alpha_motor);
//    update_motor(&delta_motor);
}

void set_alpha_theta_mode()
{
    alpha_motor.kP 		= ALPHA_P; 
    alpha_motor.kI 		= ALPHA_I;
    alpha_motor.kD 		= ALPHA_D; 
    
    alpha_and_theta = 1;
}

void unset_alpha_theta_mode()
{
    alpha_motor.kP 		= 3 * ALPHA_P; 
    alpha_motor.kI 		= ALPHA_I;
    alpha_motor.kD 		= 2 * ALPHA_D; 
    
    alpha_and_theta = 0;
}
/*******************************/
/* MOTION CONTROL FUNCTIONS */
/*******************************/
inline void do_motion_control(void)
{

if(alpha_and_theta == 0) {
    // PID angle
    alpha_motor.des_speed = compute_position_PID(&bot_command_alpha, &alpha_motor);

    // PID distance
    if ((bot_command_alpha.state == WAITING_BEGIN) || (bot_command_alpha.state == PROCESSING_COMMAND)) {        // If alpha motor have not finished its movement 

    } else {
        if ((bot_command_delta.state != PROCESSING_COMMAND) && (prev_bot_command_delta.state == WAITING_BEGIN)) {
            prev_bot_command_delta.state = PROCESSING_COMMAND;
            set_new_command(&bot_command_delta, prev_bot_command_delta.desired_distance);
        }
        //delta_motor.des_speed = compute_position_PID(&bot_command_delta, &delta_motor);

    }
    delta_motor.des_speed = compute_position_PID(&bot_command_delta, &delta_motor);

    /*if (bot_command_alpha.state == WAITING_BEGIN) {
       bot_command_alpha.state = PROCESSING_COMMAND;
       } */
}
else {


    // PID angle
    alpha_motor.des_speed = compute_position_PID(&bot_command_alpha, &alpha_motor);

    // PID distance
/*    if ((bot_command_alpha.state == WAITING_BEGIN) || (bot_command_alpha.state == PROCESSING_COMMAND)) {        // If alpha motor have not finished its movement 
        double ang = angle_coord(&maximus, goal.x, goal.y, reverse_mode) * RAD2DEG;
        if (abs(ang) > 3
            && (bot_command_alpha.state == PROCESSING_COMMAND))
            set_new_command(&bot_command_alpha, ang);

        alpha_motor.des_speed =
            compute_position_PID(&bot_command_alpha, &alpha_motor);

        double dist = distance_coord(&maximus, goal.x, goal.y, reverse_mode);
        //double max_possible_speed = 1050000 * dist / ang;
        //double max_possible_speed = 105000 * abs(dist) / abs(ang);      //35000
        //double max_possible_speed = 625000 * abs(dist) / abs(ang);      //35000
        double max_possible_speed = 6000000 * abs(dist) / abs(ang);      //35000
        if (max_possible_speed < 70)
            max_possible_speed = 0;
        delta_motor.max_speed =
            min(max_possible_speed, DELTA_MAX_SPEED - 500); //1500
        set_new_command(&bot_command_delta, dist);
        prev_bot_command_delta.state = WAITING_BEGIN;
    } else {
        if ((prev_bot_command_delta.state == WAITING_BEGIN)) {
            double dist = distance_coord(&maximus, goal.x, goal.y, reverse_mode);

            delta_motor.max_speed = DELTA_MAX_SPEED;
            set_new_command(&bot_command_delta, dist);

            prev_bot_command_delta.state = PROCESSING_COMMAND;
        }
        
        if( (last_goal == 1) && (bot_command_delta.state == COMMAND_DONE)) {

          double angletodo = desired_last_theta - maximus.theta;
   
          if (angletodo > PI)
            angletodo = angletodo - 2 * PI;
          if (angletodo < -PI)
            angletodo = 2 * PI + angletodo;

          set_new_command(&bot_command_alpha, angletodo * RAD2DEG );
            
          //set_new_command(&bot_command_alpha, desired_last_theta);
            last_goal = 0;
            //alpha_and_theta = 0;
            unset_alpha_theta_mode();
        }
        
    }*/
    delta_motor.des_speed =
        compute_position_PID(&bot_command_delta, &delta_motor);

}


}

void set_new_command(struct RobotCommand *cmd, double distance)
{
    cmd->state = WAITING_BEGIN;
    cmd->current_distance = 0;
    cmd->desired_distance = distance;
}

long compute_position_PID(struct RobotCommand *cmd,
                          struct motor *used_motor)
{
    long tmp = 0;
    return tmp;
}

// Compute the distance to do to go to (x, y)
double distance_coord(struct robot *my_robot, double x1, double y1, int reverse)
{
    double x = 0;
    x = sqrt(pow(fabs(x1 - my_robot->pos_X), 2) +
             pow(fabs(y1 - my_robot->pos_Y), 2));
             
    if(reverse == 1)
      x = -x;
    return x;
}

// Compute the angle to do to go to (x, y)
double angle_coord(struct robot *my_robot, double x1, double y1, int reverse)
{
    double angletodo = 0;
    if ((x1 < my_robot->pos_X) && (y1 < my_robot->pos_Y)) {
        angletodo =
            -PI / 2 -
            atan(fabs((x1 - my_robot->pos_X) / (y1 - my_robot->pos_Y)));
    } else if ((x1 > my_robot->pos_X) && (y1 < my_robot->pos_Y)) {
        angletodo =
            -atan(fabs((y1 - my_robot->pos_Y) / (x1 - my_robot->pos_X)));
    } else if ((x1 > my_robot->pos_X) && (y1 > my_robot->pos_Y)) {
        angletodo =
            atan(fabs((y1 - my_robot->pos_Y) / (x1 - my_robot->pos_X)));
    } else if ((x1 < my_robot->pos_X) && (y1 > my_robot->pos_Y)) {
        angletodo =
            PI / 2 +
            atan(fabs((x1 - my_robot->pos_X) / (y1 - my_robot->pos_Y)));
    } else if ((x1 < my_robot->pos_X) && (y1 == my_robot->pos_Y)) {     // 
        angletodo = -PI;
    } else if ((x1 > my_robot->pos_X) && (y1 == my_robot->pos_Y)) {     // 
        angletodo = 0;
    } else if ((x1 == my_robot->pos_X) && (y1 < my_robot->pos_Y)) {     // 
        angletodo = -PI / 2;
    } else if ((x1 == my_robot->pos_X) && (y1 > my_robot->pos_Y)) {     // 
        angletodo = PI / 2;
    } else
        angletodo = 0;

    angletodo = angletodo - my_robot->theta;

    if (angletodo > PI)
        angletodo = angletodo - 2 * PI;
    if (angletodo < -PI)
        angletodo = 2 * PI + angletodo;

    if(reverse == 1){
      if (angletodo < 0)
        angletodo = (angletodo + PI);
      else
        angletodo = (angletodo - PI);
    }

    return angletodo;
}

void goto_xy(double x, double y)
{
    double ang, dist;

    ang = angle_coord(&maximus, x, y, FORWARD) * RAD2DEG;
    set_new_command(&bot_command_alpha, ang);

    dist = distance_coord(&maximus, x, y, FORWARD);
    set_new_command(&prev_bot_command_delta, dist);
    bot_command_delta.state = WAITING_BEGIN;
}

void goto_xy_back(double x, double y)
{
    double ang, dist;

    ang = angle_coord(&maximus, x, y, REVERSE) * RAD2DEG;
    set_new_command(&bot_command_alpha, ang);

    dist = distance_coord(&maximus, x, y, REVERSE);
    set_new_command(&prev_bot_command_delta, dist);
    bot_command_delta.state = WAITING_BEGIN;
}

void rotate(double heading, double attitude, double bank,
            geometry_msgs::Quaternion * pose)
{
    // Assuming the angles are in radians.
    double c1 = cos(heading / 2);
    double s1 = sin(heading / 2);
    double c2 = cos(attitude / 2);

    double s2 = sin(attitude / 2);
    double c3 = cos(bank / 2);
    double s3 = sin(bank / 2);
    double c1c2 = c1 * c2;
    double s1s2 = s1 * s2;

    pose->w = c1c2 * c3 - s1s2 * s3;
    pose->x = c1c2 * s3 + s1s2 * c3;
    pose->y = s1 * c2 * c3 + c1 * s2 * s3;
    pose->z = c1 * s2 * c3 - s1 * c2 * s3;
}




