#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;

// Publisher for encoder lengths
std_msgs::Float32MultiArray encoder_msg;
ros::Publisher encoder_pub("/encoder_lengths", &encoder_msg);

// Encoder pins
#define ENCODER1_A 7
#define ENCODER1_B 8
#define ENCODER2_A 9
#define ENCODER2_B 10

// Cable lengths (initial values)
volatile long encoder1_position = 0;
volatile long encoder2_position = 0;

const float STEPS_PER_METER = 1000.0;  // Example calibration value

// Interrupt service routines for encoders
void encoder1_ISR() {
  if (digitalRead(ENCODER1_B) == HIGH) {
    encoder1_position++;
  } else {
    encoder1_position--;
  }
}

void encoder2_ISR() {
  if (digitalRead(ENCODER2_B) == HIGH) {
    encoder2_position++;
  } else {
    encoder2_position--;
  }
}

void setup() {
  nh.initNode();
  nh.advertise(encoder_pub);

  // Initialize encoder pins
  pinMode(ENCODER1_A, INPUT);
  pinMode(ENCODER1_B, INPUT);
  pinMode(ENCODER2_A, INPUT);
  pinMode(ENCODER2_B, INPUT);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), encoder1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), encoder2_ISR, RISING);
}

void loop() {
  // Calculate cable lengths
  float cable_length_1 = encoder1_position / STEPS_PER_METER;
  float cable_length_2 = encoder2_position / STEPS_PER_METER;

  // Populate and publish encoder message
  encoder_msg.data_length = 2;
  encoder_msg.data[0] = cable_length_1;
  encoder_msg.data[1] = cable_length_2;
  encoder_pub.publish(&encoder_msg);

  nh.spinOnce();  // Handle ROS communication
  delay(10);
}
