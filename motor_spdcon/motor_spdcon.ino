#include <ros2arduino.h>
#include <due_can.h>

#define XRCEDDS_PORT  Serial
#define ID  1

// Input
int rm_target_speed[] = {0,0,0,0};
int rm_current[] = {0,0,0,0};
double pid_speed[4][3]={0};

// Info
const int MAX_OUTPUT = 16384;

// PID
int rm_speed[4];
int rm_speed_error[4], rm_last_speed_error[4];
double rm_sk_pid[4][3] = {0};
double rm_so_pid[4][3] = {0};

CAN_FRAME tx_msg, rx_msg;
int rm_output[4] = {0,0,0,0};

// Calculates PID and puts it in rm_output, ready to be sent
void PIDSpeedCalculate() {
  for (int id = 0; id < 4; id++)
  {
  rm_last_speed_error[id] = rm_speed_error[id];

  rm_speed_error[id] = rm_target_speed[id] - rm_speed[id];
  rm_speed_error[id] = rm_speed[id] > 32768 ? rm_speed_error[id] + 65536 : rm_speed_error[id];

  rm_so_pid[id][0] = rm_sk_pid[id][0] * rm_speed_error[id];
  rm_so_pid[id][1] += (rm_sk_pid[id][1] * rm_speed_error[id]);
  rm_so_pid[id][2] = rm_sk_pid[id][2] * (rm_speed_error[id] - rm_last_speed_error[id]);

  rm_so_pid[id][1] = constrain(rm_so_pid[id][1], -1000, 1000);

  rm_output[id] = rm_so_pid[id][0] + rm_so_pid[id][1] + rm_so_pid[id][2];
  rm_output[id] = constrain(rm_output[id], -MAX_OUTPUT, MAX_OUTPUT);
  }
}

// Sends the calculated current to CAN bus
void sendRMMotorCurrent() {

  for (int i =0; i < 4; i++){
    tx_msg.data.byte[i * 2] = rm_output[i] >> 8;
    tx_msg.data.byte[i * 2 + 1] = rm_output[i];
    }

  Can0.sendFrame(tx_msg);
}

// Subscribers for whlspd(1-4)
void subscribe_wheel_speed1(std_msgs::Int16* msg, void* arg)
{
  (void)(arg);
      rm_target_speed[0] = (int)msg->data;
      rm_target_speed[0] = constrain(rm_target_speed[0], -10000, 10000);
}

void subscribe_wheel_speed2(std_msgs::Int16* msg, void* arg)
{
  (void)(arg);
      rm_target_speed[1] = (int)msg->data;
      rm_target_speed[1] = constrain(rm_target_speed[1], -10000, 10000);
}

void subscribe_wheel_speed3(std_msgs::Int16* msg, void* arg)
{
  (void)(arg);
      rm_target_speed[2] = (int)msg->data;
      rm_target_speed[2] = constrain(rm_target_speed[2], -10000, 10000);
}

void subscribe_wheel_speed4(std_msgs::Int16* msg, void* arg)
{
  (void)(arg);
      rm_target_speed[3] = (int)msg->data;
      rm_target_speed[3] = constrain(rm_target_speed[3], -10000, 10000);
}


class Wheel_speedSub : public ros2::Node {
  public:
    Wheel_speedSub(): Node("ros2arduino_sub_node") {
      sub1 = this->createSubscriber<std_msgs::Int16>("whlspd1", (ros2::CallbackFunc)subscribe_wheel_speed1, NULL);
      sub2 = this->createSubscriber<std_msgs::Int16>("whlspd2", (ros2::CallbackFunc)subscribe_wheel_speed2, NULL);
      sub3 = this->createSubscriber<std_msgs::Int16>("whlspd3", (ros2::CallbackFunc)subscribe_wheel_speed3, NULL);
      sub4 = this->createSubscriber<std_msgs::Int16>("whlspd4", (ros2::CallbackFunc)subscribe_wheel_speed4, NULL);
    }

  private:
    ros2::Subscriber<std_msgs::Int16>* sub1;
    ros2::Subscriber<std_msgs::Int16>* sub2;
    ros2::Subscriber<std_msgs::Int16>* sub3;
    ros2::Subscriber<std_msgs::Int16>* sub4;
};


// Main program begins here //////////////////////////////////////////////////////////////////////////////////////////////


void setup()
{
  XRCEDDS_PORT.begin(115200);
  while (!XRCEDDS_PORT);

  ros2::init(&XRCEDDS_PORT);

  // Initialize default global PID params
  for(int i = 0; i < 4; i++) {
    rm_sk_pid[i][0] = 1.25;
    rm_sk_pid[i][1] = 0.0;
    rm_sk_pid[i][2] = 2.75;
  }

  // Add special-case PID params here
  rm_sk_pid[1][0] = 1.0;
  rm_sk_pid[1][2] = 2.75;

  Can0.begin(CAN_BPS_1000K);  //  For communication with RM motors

  tx_msg.id = 0x200;
  tx_msg.length = 8;

}

void loop()
{
  static Wheel_speedSub wheel_speedNode;

  ros2::spin(&wheel_speedNode);

  Can0.watchFor();
  Can0.read(rx_msg);
  rm_speed[rx_msg.id - 0x201] = rx_msg.data.byte[2] << 8 | rx_msg.data.byte[3];

  PIDSpeedCalculate();
  sendRMMotorCurrent();
}
