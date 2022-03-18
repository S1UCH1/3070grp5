#include <ros2arduino.h>
#include <due_can.h>

#define XRCEDDS_PORT  Serial
#define ID  1


// Input
int rm_target_speed[] = {0,0,0,0};

// Info
const int MAX_OUTPUT = 16384;

// PID - Speed
int rm_speed[4];
int rm_speed_error[4], rm_last_speed_error[4];
double rm_sk_pid[3] = {0};
double rm_so_pid[3] = {0};

// PID - Position
int rm_position_error[4], rm_last_position_error[4];
double rm_pk_pid[3] = {0};
double rm_po_pid[3] = {0};

CAN_FRAME tx_msg, rx_msg_1, rx_msg_2;
int rm_output[4] = {0,0,0,0};

void PIDSpeedCalculate(int i) {
  rm_last_speed_error[i] = rm_speed_error[i];

  rm_speed_error[i] = rm_target_speed[i] - rm_speed[i];
  rm_speed_error[i] = rm_speed[i] > 32768 ? rm_speed_error[i] + 65536 : rm_speed_error[i];

  rm_so_pid[0] = rm_sk_pid[0] * rm_speed_error[i];
  rm_so_pid[1] += (rm_sk_pid[1] * rm_speed_error[i]);
  rm_so_pid[2] = rm_sk_pid[2] * (rm_speed_error[i] - rm_last_speed_error[i]);

  rm_so_pid[1] = constrain(rm_so_pid[1], -1000, 1000);

  rm_output[i] = rm_so_pid[0] + rm_so_pid[1] + rm_so_pid[2];
  rm_output[i] = constrain(rm_output[i], -MAX_OUTPUT, MAX_OUTPUT);
}

void sendRMMotorCurrent(int i1) {
  tx_msg.data.byte[0] = i1 >> 8;
  tx_msg.data.byte[1] = i1;

  Can1.sendFrame(tx_msg);
}

void subscribe_wheel_speed1(std_msgs::Float32* msg, void* arg)
{
  (void)(arg);
      rm_target_speed[0] = (int)msg->data;
      rm_target_speed[0] = constrain(rm_target_speed[0], -10000, 10000);
}

void subscribe_wheel_speed2(std_msgs::Float32* msg, void* arg)
{
  (void)(arg);
      rm_target_speed[1] = (int)msg->data;
      rm_target_speed[1] = constrain(rm_target_speed[1], -10000, 10000);
}

void subscribe_wheel_speed3(std_msgs::Float32* msg, void* arg)
{
  (void)(arg);
      rm_target_speed[2] = (int)msg->data;
      rm_target_speed[2] = constrain(rm_target_speed[2], -10000, 10000);
}

void subscribe_wheel_speed4(std_msgs::Float32* msg, void* arg)
{
  (void)(arg);
      rm_target_speed[3] = (int)msg->data;
      rm_target_speed[3] = constrain(rm_target_speed[3], -10000, 10000);
}


class wheel_speedSub : public ros2::Node
{
public:
  wheel_speedSub()
  : Node("ros2arduino_sub_node")
  {
    this->createSubscriber<std_msgs::Float32>("whlspd1", (ros2::CallbackFunc)subscribe_wheel_speed1, nullptr);
    this->createSubscriber<std_msgs::Float32>("whlspd2", (ros2::CallbackFunc)subscribe_wheel_speed2, nullptr);
    this->createSubscriber<std_msgs::Float32>("whlspd3", (ros2::CallbackFunc)subscribe_wheel_speed3, nullptr);
    this->createSubscriber<std_msgs::Float32>("whlspd4", (ros2::CallbackFunc)subscribe_wheel_speed4, nullptr);
    
  }
};


void setup() 
{
  XRCEDDS_PORT.begin(115200);
  while (!XRCEDDS_PORT); 

  ros2::init(&XRCEDDS_PORT);
  Serial.begin(115200);
  
  Can0.begin(CAN_BPS_1000K);  //  For communication with RM motors

  rm_sk_pid[0] = 2.0;
  rm_sk_pid[1] = 0.0;
  rm_sk_pid[2] = 2.75;

  rm_pk_pid[0] = 1.0;
  rm_pk_pid[1] = 0.0;
  rm_pk_pid[2] = 500.0;

  tx_msg.id = 0x200;
  tx_msg.length = 8;
}

void loop() 
{
  static wheel_speedSub wheel_speedNode;

  ros2::spin(&wheel_speedNode);
  // Local testing
  if(Serial.available() > 0) {
    for (int i = 0; i < 4; i++)
    {
    rm_target_speed[i] = Serial.parseInt();
    rm_target_speed[i] = constrain(rm_target_speed[i], -10000, 10000);
    }
     if (Serial.read() == '\n');
}
  for (int i = 0; i < 4; i++)
  {
    Serial.print(rm_target_speed[i]);
    Serial.print(" ");
  }
  
  Serial.println();
  Can0.watchFor();
  Can0.read(rx_msg_2);
  
  for (int i = 0; i < 4; i++)
  {
  rm_speed[i] = rx_msg_2.data.byte[2] << 8 | rx_msg_2.data.byte[3];
  PIDSpeedCalculate(i);
  sendRMMotorCurrent(rm_output[i]);
  }
  
}
