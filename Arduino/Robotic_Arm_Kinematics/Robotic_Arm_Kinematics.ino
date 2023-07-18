

// Include libraries
#include <Servo.h>
#include <SoftwareSerial.h>

// Servo objects
Servo servo2; // Joint 2
Servo servo3; // Joint 3
Servo servo4; // Joint 4
Servo servo5; // For the gripper

// SoftwareSerial port used just for debugging
SoftwareSerial debugSerial(3, 6); // RX, TX

// ---- Motor variables and constants ----
// Configuration space limits for each motor
#define MOTORDC1_INF_LIMIT -90
#define MOTORDC1_SUP_LIMIT 90

#define SERVO2_INF_LIMIT -25
#define SERVO2_SUP_LIMIT 155

#define SERVO3_INF_LIMIT -135
#define SERVO3_SUP_LIMIT 45

#define SERVO4_INF_LIMIT -90
#define SERVO4_SUP_LIMIT 90

// For the gripper
#define SERVO5_INF_LIMIT 50 // Open gripper (change to your liking)
#define SERVO5_SUP_LIMIT 90 // Closed gripper (change to your liking)

// Signal pins for driving the dc motor
#define DCMOTOR_CTRL_PIN_INA_1   7
#define DCMOTOR_CTRL_PIN_INB_1   4
#define DCMOTOR_CTRL_PIN_PWM_1   5

// These store angles for dc motor1
float motor1_target_angle = 0;
float motor1_target_angle_prev;
float motor1_current_angle = 0;

// Pin number, start angle and target angles for each servo
#define SERVO_2_PIN 10
#define SERVO2_START_ANGLE 90
float servo2_target_angle = SERVO2_START_ANGLE;
float servo2_target_angle_prev = SERVO2_START_ANGLE;

#define SERVO_3_PIN 11
#define SERVO3_START_ANGLE  -90
float servo3_target_angle = SERVO3_START_ANGLE;
float servo3_target_angle_prev = SERVO3_START_ANGLE;

#define SERVO_4_PIN 12
#define SERVO4_START_ANGLE  -90
float servo4_target_angle = SERVO4_START_ANGLE;
float servo4_target_angle_prev = SERVO4_START_ANGLE;

#define SERVO_5_PIN 8
#define SERVO5_START_ANGLE  SERVO5_INF_LIMIT
float servo5_target_angle = SERVO5_START_ANGLE;
float servo5_target_angle_prev = SERVO5_START_ANGLE;

// ---- Encoder variables and constants ----
// Encoder pins
int encoder_pin_a = 2;
int encoder_pin_b = 9;
volatile long encoder_ticks = 0; // Stores the encoder ticks
volatile bool encoder_pin_b_state; // For the encoder interrupt service routine

// ---- PID control variables and constants ----
#define PID_SAMPLE_TIME   1 // The loop time for the PID in ms.
#define ENC_TICKS_PER_REV 6296.0 // Change this to refelct your encoder's

float Kp = 400;   // PID proportional constant
float Kd = 0.07;  // PID pderivative constant
float Ki = 0.001; // PID integral constant

// For storing PID errors
float integral_error = 0;
float derivative_error = 0;
float proportional_error = 0;
float proportional_error_prev = 0;

float pid_pwm_value = 0; // The PWM value to drive dc motor1

unsigned long pid_last_millis = 0; // For computing PID loop time
unsigned long print_last_millis = 0; // For computing debug print loop time

// ---- Moving average filter variables and constants ----
#define NUM_FILTERING_POINTS  20 // number of filtering points
float prev_angles_buffer[NUM_FILTERING_POINTS]; // Angle readings bufer

// ---- Serial command interface variables and constants ---- 
// States for parsing command frames
enum t_states {
  MOTOR_DC1_STATE,
  SERVO2_STATE,
  SERVO3_STATE,
  SERVO4_STATE,
  SERVO5_STATE
};

int state = MOTOR_DC1_STATE; // Initial state
String str_received_angle; // Stores received angles from the companion computer
int advance_step_degrees = 10; // For manually positioning the motors

void setup() {
  // Start serial communications with the companion computer
  Serial.begin(9600);

  // Start serial communications for debugging
  debugSerial.begin(57600); //38400
  debugSerial.println("--- Debugging ready ---");

  // Initialize servo motors
  servo2.attach(SERVO_2_PIN);
  // Convert between conf. spaces before driving to start angles
  servo2.write(map(servo2_target_angle, SERVO2_INF_LIMIT, SERVO2_SUP_LIMIT, 180, 0));
  delay(500);

  servo3.attach(SERVO_3_PIN);
  // Convert between conf. spaces before driving to start angles
  servo3.write(map(servo3_target_angle, SERVO3_INF_LIMIT, SERVO3_SUP_LIMIT, 0, 180));
  delay(500);

  servo4.attach(SERVO_4_PIN);
  // Convert between conf. spaces before driving to start angles
  servo4.write(map(servo4_target_angle, SERVO4_INF_LIMIT, SERVO4_SUP_LIMIT, 0, 180));
  delay(500);

  // Gripper is open by default
  servo5.attach(SERVO_5_PIN);
  servo5.write(SERVO5_INF_LIMIT);
  delay(500);

  // Initialize pins and interrupt for bi-phase encoder
  pinMode(encoder_pin_a, INPUT);
  pinMode(encoder_pin_b, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_pin_a), Encoder_Isr, RISING);
}

void loop() {

  Receive_Commands(); // Check if there are command frames from the companion computer

  // Apply new angle to servo2
  if (servo2_target_angle != servo2_target_angle_prev) {
    // Convert between conf. spaces before driving to new angle
    float ang_conv = map(servo2_target_angle, SERVO2_INF_LIMIT, SERVO2_SUP_LIMIT, 180, 0);
    servo2.write(round(ang_conv));
    delay(15);
    servo2_target_angle_prev = servo2_target_angle;
  }

  // Apply new angle to servo3
  if (servo3_target_angle != servo3_target_angle_prev) {
    // Convert between conf. spaces before driving to new angle
    float ang_conv = map(servo3_target_angle, SERVO3_INF_LIMIT, SERVO3_SUP_LIMIT, 0, 180);
    servo3.write(round(ang_conv));
    delay(15);
    servo3_target_angle_prev = servo3_target_angle;
  }

  // Apply new angle to servo4
  if (servo4_target_angle != servo4_target_angle_prev) {
    // Convert between conf. spaces before driving to new angle
    float ang_conv = map(servo4_target_angle, SERVO4_INF_LIMIT, SERVO4_SUP_LIMIT, 0, 180);
    servo4.write(round(ang_conv));
//    servo4.write(servo4_target_angle);
    delay(15);
    servo4_target_angle_prev = servo4_target_angle;
  }

  // Apply new angle to servo5 (gripper)
  if (servo5_target_angle != servo5_target_angle_prev) {
    servo5.write(servo5_target_angle);
    delay(15);
    servo5_target_angle_prev = servo5_target_angle;
  }

  // Run PID control for dc motor1
  if ((millis() - pid_last_millis) >= PID_SAMPLE_TIME) // Check if PID loop period has passed
  {
    pid_last_millis = millis();
    
    Compute_Motor_Position(); // Get the actual motor angle

    Push_Angle(motor1_current_angle); // Store the angle in buffer

    // Verify tolerance range for target angle
    if(abs(motor1_target_angle-Compute_Filtered_Angle()) > 0.5) {
      // Move the motor if still outside the tolerance range
      // Use filtered angle to compute PID
      pid_pwm_value = Compute_Pid(motor1_target_angle, Compute_Filtered_Angle());
    }
    else {
      // Stop the motor, otherwise
      pid_pwm_value = 0;
    }

    // Drive the motor in the required direction
    if (pid_pwm_value > 0) {
      // Drive the motor counter clock wise
      digitalWrite(DCMOTOR_CTRL_PIN_INA_1, HIGH);
      digitalWrite(DCMOTOR_CTRL_PIN_INB_1, LOW);
      analogWrite(DCMOTOR_CTRL_PIN_PWM_1, pid_pwm_value);
    } else {
      // Drive the motor clock wise
      digitalWrite(DCMOTOR_CTRL_PIN_INA_1, LOW);
      digitalWrite(DCMOTOR_CTRL_PIN_INB_1, HIGH);
      analogWrite(DCMOTOR_CTRL_PIN_PWM_1, -pid_pwm_value);
    }
  }
  else {
    // Only for debugging in the IDE's serial plotter
    // comment if the MATLAB code is running, otherwise the
    // MATLAB code will crash
//    Print_Pid_Info(); 
  }
}

/* Compute PID control for dc motor */
int Compute_Pid(float setpoint, float process_var_reading)
{
  float control_output = 0;

  // Compute errors (proportional, integral and derivative)
  proportional_error = setpoint - process_var_reading;
  integral_error = integral_error + proportional_error;
  derivative_error = (proportional_error - proportional_error_prev);

  // Apply the PID formula
  control_output = (Kp * proportional_error) + (Kd * derivative_error) + (Ki * integral_error);

  // Store current proportional error for next interation
  proportional_error_prev = proportional_error;

  // Limit the output (absolute value) to 0~255 range
  // to drive the motor by using Arduino's analogWrite PWM
  if (control_output > 255) {
    control_output = 255;
  }
  else if (control_output < -255) {
    control_output = -255;
  }
  
  return control_output;
}

/* Compute dc motor's current posicion by using encoder data */
void Compute_Motor_Position()
{
  // Compute the angle from encoder ticks
  motor1_current_angle = (encoder_ticks) * 360.0 / (ENC_TICKS_PER_REV);
}

/* Push the last read angle to the buffer */
void Push_Angle(float val)
{
  for (int i = 0; i < (NUM_FILTERING_POINTS - 1); ++i) {
    prev_angles_buffer[i] = prev_angles_buffer[i + 1];
  }
  prev_angles_buffer[NUM_FILTERING_POINTS - 1] = val;
}

/* Compute filtered angle with moving average filter */
float Compute_Filtered_Angle()
{
  float sum;

  sum = 0;
  for (int i = 0; i < (NUM_FILTERING_POINTS); ++i) {
    sum = sum + prev_angles_buffer[i];
  }

  return (sum / NUM_FILTERING_POINTS);
}

/* Print PID data just for debugging */
void Print_Pid_Info()
{
  if ((millis() - print_last_millis) >= 150)
  {
    print_last_millis = millis();

    Serial.print("  TGT_ANG: ");     Serial.print(motor1_target_angle);
    Serial.print("  CUR_ANG: ");     Serial.print(motor1_current_angle);
    Serial.print("  PID_PWM: ");    Serial.print(pid_pwm_value);
    Serial.print("  FILT_ANG: "); Serial.print(Compute_Filtered_Angle());
    Serial.print("  ENC_TIC: ");        Serial.print(encoder_ticks);

    Serial.println();
  }
}

/* Receive and parse serial commands from the companion computer */
void Receive_Commands() {
  char c;

  // Read a character from serial buffer, if available
  if (Serial.available()) {
    c = Serial.read();
  }

  // Verify if character is motor ID
  // and change state to corresponding motor
  if (c == '1') {
    state = MOTOR_DC1_STATE;
  }
  else if (c == '2') {
    state = SERVO2_STATE;
  }
  else if (c == '3') {
    state = SERVO3_STATE;
  }
  else if (c == '4') {
    state = SERVO4_STATE;
  }
  else if (c == '5') {
    state = SERVO5_STATE;
  }

  // Parse command for current motor
  switch (state) {

    case MOTOR_DC1_STATE:
      // If c == '$', then extract angle data
      if (c == '$') {
        // At 9600 baud, we receive 1 char. in 1ms, 4 chars (an angle number) in 4ms
        delay(6); 
        c = Serial.read();
        while (c != '&') {
          str_received_angle += (char)c;
          c = Serial.read();
        }
        motor1_target_angle = str_received_angle.toFloat();
        str_received_angle = "";
      }

      // The following 'hidden' commands are for debugging
      // and manually positioning the motors
      // Else, if c == '-', decrease target angle
      else if (c == '-') {
        motor1_target_angle = motor1_target_angle - advance_step_degrees;
      }

      // Else, if c == '+', increase target angle
      else if (c == '+') {
        motor1_target_angle = motor1_target_angle + advance_step_degrees;
      }

      // Else, if c == 'c', reset to start angle
      else if (c == 'c' || c == 'C') {
        encoder_ticks = 0;
        motor1_target_angle = 0;
        motor1_current_angle = 0;
      }

      // Verify sup. and inf. limits for the configuration
      // space and truncate the angle between those limits
      if (motor1_target_angle != motor1_target_angle_prev) {
        if (motor1_target_angle < MOTORDC1_INF_LIMIT) {
          debugSerial.println("motor1 inf. limit!");
          motor1_target_angle = MOTORDC1_INF_LIMIT;
        }
        else if (motor1_target_angle > MOTORDC1_SUP_LIMIT) {
          debugSerial.println("motor1 sup. limit!");
          motor1_target_angle = MOTORDC1_SUP_LIMIT;
        }
        // Print received angle just for debugging
        debugSerial.print("motor1_target_angle: ");
        debugSerial.println(motor1_target_angle);
        motor1_target_angle_prev = motor1_target_angle;
      }
      break;

    case SERVO2_STATE:
      // If c == '$', then extract angle data
      if (c == '$') {
        // At 9600 baud, we receive 1 char. in 1ms, 4 chars (an angle number) in 4ms
        delay(6); 
        c = Serial.read();
        while (c != '&') {
          str_received_angle += (char)c;
          c = Serial.read();
        }
        servo2_target_angle = str_received_angle.toFloat();
        str_received_angle = "";
      }

      // The following 'hidden' commands are for debugging
      // and manually positioning the motors
      // Else, if c == '-', decrease target angle
      else if (c == '-') {
        servo2_target_angle = servo2_target_angle - advance_step_degrees;
      }

      // Else, if c == '+', increase target angle
      else if (c == '+') {
        servo2_target_angle = servo2_target_angle + advance_step_degrees;
      }

      // Else, if c == 'c', reset to start angle
      else if (c == 'c' || c == 'C') {
        servo2_target_angle = SERVO2_START_ANGLE;
      }

      // Verify sup. and inf. limits for the configuration
      // space and truncate the angle between those limits
      if (servo2_target_angle != servo2_target_angle_prev) {
        if (servo2_target_angle < SERVO2_INF_LIMIT) {
          debugSerial.println("servo2 inf. limit!");
          servo2_target_angle = SERVO2_INF_LIMIT;
        }
        else if (servo2_target_angle > SERVO2_SUP_LIMIT) {
          debugSerial.println("servo2 sup. limit!");
          servo2_target_angle = SERVO2_SUP_LIMIT;
        }
        // Print received angle just for debugging
        debugSerial.print("servo2_target_angle: ");
        debugSerial.println(servo2_target_angle);
      }
      break;

    case SERVO3_STATE:
      // If c == '$', then extract angle data
      if (c == '$') {
        // At 9600 baud, we receive 1 char. in 1ms, 4 chars (an angle number) in 4ms
        delay(6); 
        c = Serial.read();
        while (c != '&') {
          str_received_angle += (char)c;
          c = Serial.read();
        }
        servo3_target_angle = str_received_angle.toFloat();
        str_received_angle = "";
      }

      // The following 'hidden' commands are for debugging
      // and manually positioning the motors
      // Else, if c == '-', decrease target angle
      else if (c == '-') {
        servo3_target_angle = servo3_target_angle - advance_step_degrees;
      }

      // Else, if c == '+', increase target angle
      else if (c == '+') {
        servo3_target_angle = servo3_target_angle + advance_step_degrees;
      }

      // Else, if c == 'c', reset to start angle
      else if (c == 'c' || c == 'C') {
        servo3_target_angle = SERVO3_START_ANGLE;
      }

      // Verify sup. and inf. limits for the configuration
      // space and truncate the angle between those limits
      if (servo3_target_angle != servo3_target_angle_prev) {
        if (servo3_target_angle < SERVO3_INF_LIMIT) {
          debugSerial.println("servo3 inf. limit!");
          servo3_target_angle = SERVO3_INF_LIMIT;
        }
        else if (servo3_target_angle > SERVO3_SUP_LIMIT) {
          debugSerial.println("servo3 sup. limit!");
          servo3_target_angle = SERVO3_SUP_LIMIT;
        }
        // Print received angle just for debugging
        debugSerial.print("servo3_target_angle: ");
        debugSerial.println(servo3_target_angle);
      }
      break;

    case SERVO4_STATE:
      // If c == '$', then extract angle data
      if (c == '$') {
        // At 9600 baud, we receive 1 char. in 1ms, 4 chars (an angle number) in 4ms
        delay(6); 
        c = Serial.read();
        while (c != '&') {
          str_received_angle += (char)c;
          c = Serial.read();
        }
        servo4_target_angle = str_received_angle.toFloat();
        str_received_angle = "";
      }

      // The following 'hidden' commands are for debugging
      // and manually positioning the motors
      // Else, if c == '-', decrease target angle
      else if (c == '-') {
        servo4_target_angle = servo4_target_angle - advance_step_degrees;

        if (servo4_target_angle < SERVO4_INF_LIMIT) {
          servo4_target_angle = SERVO4_INF_LIMIT;
        }
      }

      // Else, if c == '+', increase target angle
      else if (c == '+') {
        servo4_target_angle = servo4_target_angle + advance_step_degrees;
        if (servo4_target_angle > SERVO4_SUP_LIMIT) {
          servo4_target_angle = SERVO4_SUP_LIMIT;
        }
      }

      // Else, if c == 'c', reset to start angle
      else if (c == 'c' || c == 'C') {
        servo4_target_angle = SERVO4_START_ANGLE;
      }

      // Verify sup. and inf. limits for the configuration
      // space and truncate the angle between those limits
      if (servo4_target_angle != servo4_target_angle_prev) {
        if (servo4_target_angle < SERVO4_INF_LIMIT) {
          debugSerial.println("servo4 inf. limit!");
          servo4_target_angle = SERVO4_INF_LIMIT;
        }
        else if (servo4_target_angle > SERVO4_SUP_LIMIT) {
          debugSerial.println("servo4 sup. limit!");
          servo4_target_angle = SERVO4_SUP_LIMIT;
        }
        // Print received angle just for debugging
        debugSerial.print("servo4_target_angle: ");
        debugSerial.println(servo4_target_angle);
      }
      break;

    case SERVO5_STATE:
      // If c == '$', then extract angle data
      if (c == '$') {
        // At 9600 baud, we receive 1 char. in 1ms, 4 chars (an angle number) in 4ms
        delay(6); 
        c = Serial.read();
        while (c != '&') {
          str_received_angle += (char)c;
          c = Serial.read();
        }
        servo5_target_angle = str_received_angle.toFloat();
        str_received_angle = "";
      }

      // The following 'hidden' commands are for debugging
      // and manually positioning the motors
      // Else, if c == '-', decrease target angle
      else if (c == '-') {
        servo5_target_angle = servo5_target_angle - advance_step_degrees;

        if (servo5_target_angle < SERVO5_INF_LIMIT) {
          servo5_target_angle = SERVO5_INF_LIMIT;
        }
      }

      // Else, if c == '+', increase target angle
      else if (c == '+') {
        servo5_target_angle = servo5_target_angle + advance_step_degrees;
        if (servo5_target_angle > SERVO5_SUP_LIMIT) {
          servo5_target_angle = SERVO5_SUP_LIMIT;
        }
      }

      // Else, if c == 'c', reset to start angle
      else if (c == 'c' || c == 'C') {
        servo5_target_angle = SERVO5_START_ANGLE;
      }

      // Verify sup. and inf. limits for the configuration
      // space and truncate the angle between those limits
      if (servo5_target_angle != servo5_target_angle_prev) {
        if (servo5_target_angle < SERVO5_INF_LIMIT) {
          debugSerial.println("servo5 inf. limit!");
          servo5_target_angle = SERVO5_INF_LIMIT;
        }
        else if (servo5_target_angle > SERVO5_SUP_LIMIT) {
          debugSerial.println("servo5 sup. limit!");
          servo5_target_angle = SERVO5_SUP_LIMIT;
        }
        // Print received angle just for debugging
        debugSerial.print("servo5_target_angle: ");
        debugSerial.println(servo5_target_angle);
      }
      break;

    default: break;
  }
}

/* Interrupt service routine for the encoder */
void Encoder_Isr()
{
  encoder_pin_b_state = digitalRead(encoder_pin_b);
  encoder_ticks += encoder_pin_b_state ? -1 : +1; // Count a new tick
}
