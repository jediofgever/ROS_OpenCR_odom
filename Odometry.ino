#include <OpenCR.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

#define go 50
#define back 51
#define left 52
#define right 53



#define encoder_LB 4
volatile long LBpulses = 0;
volatile bool LB;

#define encoder_LF 7
volatile long LFpulses = 0;
volatile bool LF;


//Right Encoder
#define encoder_RB 2
volatile long RBpulses = 0;
volatile bool RB;

#define encoder_RF 3
volatile long RFpulses = 0;
volatile bool RF;

unsigned long old_time;

const float Pi = 3.14159;
const double R = 0.09;         //Wheel Radius
const int N = 120 ;            // Number of Ticks Per revolution
const double L = 46 ;           //Distance between left and right wheels

double x = 0.0;
double y = 0.0;
double theta = 0;

double dt = 0.0;

double dc = 0.0;
double dr = 0.0;
double dl = 0.0;

double dtheta = 0.0;
int delta_tick = 0;

int total_ticks = 0;

int ex_Left_Ticks = 0;
int ex_Right_Ticks = 0;


char base_link[] = "/base_link";
char odom[] = "/odom";


ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;


void SetupEncoders()
{
  pinMode(encoder_RB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder_RB), RBcounter, RISING );

  pinMode(encoder_LB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder_LB), LBcounter, RISING );

  pinMode(encoder_RF, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder_RF), RFcounter, RISING );

  pinMode(encoder_LF, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder_LF), LFcounter, RISING );
}




void LBcounter()
{
  LB = digitalRead(encoder_LB);   // read the input pin
  LBpulses -= (digitalRead(go) == LOW || digitalRead(right) == LOW) ? -1 : +1;



}
void RBcounter()
{
  RB = digitalRead(encoder_RB);   // read the input pin
  RBpulses -= (digitalRead(go) == LOW || digitalRead(left) == LOW) ? -1 : +1;

}

void LFcounter()
{
  LF = digitalRead(encoder_LF);   // read the input pin
  LFpulses -= (digitalRead(go) == LOW || digitalRead(right) == LOW) ? -1 : +1;


}

void RFcounter()
{
  RF = digitalRead(encoder_RF);   // read the input pin
  RFpulses -= (digitalRead(go) == LOW || digitalRead(left) == LOW) ? -1 : +1;

}


void messageCb( const geometry_msgs::Twist& msg) {
  float move1 = msg.linear.x;
  float move2 = msg.angular.z;
  if (move1 == 0 && move2 >= 0.5)
  {
    moveLeft();   //a(0,1)



  }
  else if (move1 == 0 && move2 <= -0.5)
  {
    moveRight();  //d(0,-1)


  }
  else if (move1 >= 1 && move2 == 0)
  {
    moveFront();  //w(1,0)

  }
  else if (move1 <= -1 && move2 == 0)
  {
    moveback();  //x(-1,0)

  }
  else if (move1 == 0 && move2 == 0)
  {
    movestop();  //s(0,0)
  }
}
ros::Subscriber<geometry_msgs::Twist> sub("/turtle1/cmd_vel", &messageCb );

void moveLeft() {
  digitalWrite(52, LOW);
  nh.loginfo("left");
}

void moveFront() {
  digitalWrite(50, LOW);
  nh.loginfo("forward");
}
void moveback() {
  digitalWrite(51, LOW);
  nh.loginfo("backward");
}
void moveRight() {
  digitalWrite(53, LOW);
  nh.loginfo("right");
}
void die() {
  digitalWrite(50, HIGH);
  digitalWrite(51, HIGH);
  digitalWrite(52, HIGH);
  digitalWrite(53, HIGH);
}
void movestop() {
  digitalWrite(50, HIGH);
  digitalWrite(51, HIGH);
  digitalWrite(52, HIGH);
  digitalWrite(53, HIGH);
  nh.loginfo("stop");
}



void setup()
{

  Serial.begin(9600);
  for (int i = 50 ; i < 54; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH);

  }
  SetupEncoders();

  nh.initNode();
  nh.subscribe(sub);
  broadcaster.init(nh);
}

void loop()
{
  if (millis() - old_time >= 500 )
  {
    Serial.print(RFpulses); Serial.println(LFpulses);
    int Left_Ticks = (LBpulses + LFpulses) / 2; // average of front and back encoders to get more accurate reading

    int Right_Ticks = (RBpulses + RFpulses) / 2; // average of front and back encoders to get more accurate reading

    int Center_Ticks = (Right_Ticks + Left_Ticks) / 2; // average of left and righ encoders to get center

    int L_delta_Tick = Left_Ticks - ex_Left_Ticks;
    int R_delta_Tick = Right_Ticks - ex_Right_Ticks;
    int C_delta_Tick = (L_delta_Tick + R_delta_Tick) / 2;

    dl = 2 * Pi * R * L_delta_Tick / N ;
    dr = 2 * Pi * R * R_delta_Tick / N ;
    dc = (dl + dr) / 2;


    x += cos(theta) * dc;
    y += sin(theta) * dc;
    theta += (dr - dl) / L;

    if (theta > 3.14)
      theta = -3.14;

    // tf odom->base_link
    t.header.frame_id = odom;
    t.child_frame_id = base_link;

    t.transform.translation.x = x;
    t.transform.translation.y = y;

    t.transform.rotation = tf::createQuaternionFromYaw(theta);
    t.header.stamp = nh.now();

    broadcaster.sendTransform(t);
    nh.spinOnce();

    delay(500);
    old_time = millis();
    ex_Left_Ticks = Left_Ticks;
    ex_Right_Ticks = Right_Ticks ;
  }
}


