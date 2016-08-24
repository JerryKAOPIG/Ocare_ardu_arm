#include <Arduino.h>
#include <Servo.h>

#include <Modbus.h>
#include <ModbusSerial.h>


//#define DEBUG

#define MB_SLAVER_ID_		(4)

// Define the Modbus HOLD_REGISTER Mapping
enum StateHoldRegister {
	LEFT_MOTOR1_DEGREE,
	LEFT_MOTOR2_DEGREE,
	SLIDER_STATE,
	ARM_MODE,
	EFFORT_CATCH_LEVEL,
	CMD_LEFT_MOTOR1_DEGREE,
	CMD_LEFT_MOTOR2_DEGREE,
	CMD_SLIDER_STATE,
	CMD_ARM_MODE
};

/******Defination of the Slider state
 * SLIDER_HOME :       Slider at Home position
 * SLIDER_OPENING:     Slider Opening now
 * SLIDER_OPENED:      Slider is Opened, the arm can work around now
 * SLIDER_RETURNING:   Slider is Returning to HOME now
 * **************************/
#define SLIDER_LEFT_STATE_MASK          (0x0f)
#define SLIDER_LEFT_STATUS_BIT_SHIFT    (0)

#define SLIDER_RIGHT_STATE_MASK         (0xf0)
#define SLIDER_RIGHT_STATUS_BIT_SHIFT   (4)

enum SliderState  {
		SLIDER_L_HOME       = (1u << SLIDER_LEFT_STATUS_BIT_SHIFT),
		SLIDER_L_OPENING    = (2u << SLIDER_LEFT_STATUS_BIT_SHIFT),
		SLIDER_L_OPENED     = (3u << SLIDER_LEFT_STATUS_BIT_SHIFT),
		SLIDER_L_RETURNING  = (4u << SLIDER_LEFT_STATUS_BIT_SHIFT),
		SLIDER_R_HOME       = (1u << SLIDER_RIGHT_STATUS_BIT_SHIFT),
		SLIDER_R_OPENING    = (2u << SLIDER_RIGHT_STATUS_BIT_SHIFT),
		SLIDER_R_OPENED     = (3u << SLIDER_RIGHT_STATUS_BIT_SHIFT),
		SLIDER_R_RETURNING  = (4u << SLIDER_RIGHT_STATUS_BIT_SHIFT)
};

/******Defination of the Slider Command
 * SLIDER_[LR]O_CMD:   Open the slider to the right position
 * SLIDER_[LR]C_CMD:   Close the slider to the home position
 *
 * **************************/
#define SLIDER_LEFT_CMD_MASK        (0x0f)
#define SLIDER_LEFT_CMD_BIT_SHIFT   (0)

#define SLIDER_RIGHT_CMD_MASK  			(0xf0)
#define SLIDER_RIGHT_CMD_BIT_SHIFT  (4)
enum SliderStateCMD  {
		SLIDER_LO_CMD = (1u << SLIDER_LEFT_CMD_BIT_SHIFT),
		SLIDER_LC_CMD = (0u << SLIDER_LEFT_CMD_BIT_SHIFT),
		SLIDER_RO_CMD = (1u << SLIDER_RIGHT_CMD_BIT_SHIFT),
		SLIDER_RC_CMD = (0u << SLIDER_RIGHT_CMD_BIT_SHIFT)
};

/******Defination of the Arm Mode
* HOME :           Arm at Home position
* BUTTON_POSE:     Arm is at push button pose
* FREE_CONTROLL:   Arm is at motor controllable state,
*                  the arm can be controll by ROS now
* **************************/
enum ArmMode {
	ARM_HOME,
	BUTTON_POSE,
	FREE_CONTROLL
};

/******Defination of the Arm Mode Command
* CMD_HOME :           Arm go to Home position
* CMD_BUTTON_POSE:     Arm go to push button pose
* CMD_FREE_CONTROLL:   Arm go to motor controllable state,
*                      let arm can be controll by ROS.
* **************************/
enum ArmModeCMD {
	CMD_HOME,
	CMD_BUTTON_POSE,
	CMD_FREE_CONTROLL
};


/**************** The I/O Pin Define ********************/

#define MOTOR_RIGHT_ARM1_ID 				(1)
#define MOTOR_RIGHT_ARM2_ID 				(2)
#define MOTOR_LEFT_ARM1_ID					(3)
#define MOTOR_LEFT_ARM2_ID					(4)

#define SLIDER_LEFT_STOP_SIGNAL_E		(47)
#define SLIDER_LEFT_STOP_SIGNAL_S		(46)
#define SLIDER_LEFT_STEPPER_EN  		(53)
#define SLIDER_LEFT_STEPPER_CW  		(51)
#define SLIDER_LEFT_STEPPER_CLK  		(49)

#define SLIDER_RIGHT_STOP_SIGNAL_E	(44)
#define SLIDER_RIGHT_STOP_SIGNAL_S	(45)
#define SLIDER_RIGHT_STEPPER_EN			(52)
#define SLIDER_RIGHT_STEPPER_CW			(50)
#define SLIDER_RIGHT_STEPPER_CLK		(48)

#define LEFT_CATCH_SERVO						(13)
/*******************************************************/

/********************** Config *************************/

#define LEFT_MOTOR1_INIT_VALUE				(512)
#define LEFT_MOTOR2_INIT_VALUE				(512)

#define LEFT_MOTOR1_MAX_VALUE					(0)
#define LEFT_MOTOR2_MAX_VALUE					(0)

#define LEFT_MOTOR1_MIN_VALUE					(0)
#define LEFT_MOTOR2_MIN_VALUE					(0)

#define CATCH_SERVO_INIT_VALUE				(50)
#define CATCH_SERVO_FULL_OPEN_VALUE		(82)
#define CATCH_SERVO_FULL_CLOSE_VALUE	(130)

#define RIGHT_MOTOR1_INIT_VALUE				(520)
#define RIGHT_MOTOR2_INIT_VALUE				(815)

#define RIGHT_MOTOR1_MAX_VALUE				(844)
#define	RIGHT_MOTOR2_MAX_VALUE				(815)

#define RIGHT_MOTOR1_MIN_VALUE				(520)
#define RIGHT_MOTOR2_MIN_VALUE				(211)

#define RIGHT_MOTOR1_BTN_POSE					(770)
#define RIGHT_MOTOR2_BTN_POSE					(815)

#define RIGHT_MOTOR2_CLOSE_CATCH_THRESHOLD	(750)

/*******************************************************/

// The clk count used to push the slider via Stepper Motor at 16 step mode
#define ref_count 								(120000)

// Modbus context object
ModbusSerial mb;

uint16_t input_motor1_degree				= LEFT_MOTOR1_INIT_VALUE;
uint16_t input_motor2_degree				= LEFT_MOTOR2_INIT_VALUE;
uint16_t input_slider_state					= SLIDER_LC_CMD | SLIDER_RC_CMD;
uint16_t input_arm_mode							= ArmModeCMD::CMD_HOME;
uint16_t input_effort_catch_level		= CATCH_SERVO_INIT_VALUE;

uint16_t output_motor1_degree				= LEFT_MOTOR1_INIT_VALUE;
uint16_t output_motor2_degree				= LEFT_MOTOR2_INIT_VALUE;
uint16_t output_slider_state				= SLIDER_L_OPENED | SLIDER_R_OPENED;
uint16_t output_arm_mode						= ArmMode::ARM_HOME;
uint16_t output_effort_catch_level	= CATCH_SERVO_INIT_VALUE;

/******* The catch servo on right arm2 ********/

Servo right_catch_servo;

/*********************************************/

long int time = 0;
long int count = 0;

// Init the pin I/O
void init_pin();

// Init the ModbusSerial
void init_modbus();

// Init the AX-12 motor
void init_ax_motor();

// Send the AX-12 motor position command
void AX12_POS(int id, int data);

// Sync the local register and the local varivale
void modbus_sync();

// Set the Servo catch level
void set_catch_level(int _catch_level);

void setup()
{
	/* Setup the ModbusSerial */
	init_modbus();

	/* Setup the pin I/O mode */
	init_pin();

	// Setup the DEBUG tunnel via serial0(USB port)
	Serial.begin(115200);

	/* Setup the AX-12 motor */
	init_ax_motor();


}

void loop()
{

	// Show the current time via Serial port
	//Serial.println(micros() - time);
	// Get the current time
	time = micros();

	/********** Update the register and sync ************/

	// Note : the sync modbus task will be trigger via Serial3 interrupt
	// Sync the register with Raspberry Pi 2 ROS
	modbus_sync();

	/****************************************************/

	// ARM is home
	// Effot catch level is 0
	// Then Slide CMD Valid
	if (output_arm_mode == ArmMode::ARM_HOME) {

			// NOTE: When CW is HIGH Volatage, The slider will goto outside
			// NOTE: When CW is LOW Volatage, The slider will goto inside


			// command is open left
			if((input_slider_state & SLIDER_LEFT_CMD_MASK)  == SLIDER_LO_CMD
				&&
				(output_slider_state & SLIDER_LEFT_STATE_MASK) != SLIDER_L_OPENED) {

				// Setup the stepper CCW mode
				digitalWrite(SLIDER_LEFT_STEPPER_CW, HIGH);

				// If left slider limit switch is trigger, then STOP left lisder opening
				if( digitalRead(SLIDER_LEFT_STOP_SIGNAL_E) != HIGH) {

					digitalWrite(SLIDER_LEFT_STEPPER_CLK, HIGH);
					digitalWrite(SLIDER_LEFT_STEPPER_CLK, HIGH);
					digitalWrite(SLIDER_LEFT_STEPPER_CLK, HIGH);
					digitalWrite(SLIDER_LEFT_STEPPER_CLK, HIGH);
					digitalWrite(SLIDER_LEFT_STEPPER_CLK, LOW);

					// Clear Left slider bit state
					output_slider_state &= ~SLIDER_LEFT_STATE_MASK;
					// Set Left slider bit state
					output_slider_state |= SLIDER_L_OPENING;

				}
				else {

					// Clear Right slider bit state
					output_slider_state &= ~SLIDER_LEFT_STATE_MASK;
					// Set Left slider bit state
					output_slider_state |= SLIDER_L_OPENED;

				}

			}
			// command is close left
			if((input_slider_state & SLIDER_LEFT_CMD_MASK)  == SLIDER_LC_CMD
				&&
				(output_slider_state & SLIDER_LEFT_STATE_MASK) != SLIDER_L_HOME) {

				// Setup the stepper CCW mode
				digitalWrite(SLIDER_LEFT_STEPPER_CW, LOW);

				// If left slider limit switch is trigger, then STOP left slider closing
				if( digitalRead(SLIDER_LEFT_STOP_SIGNAL_S) != HIGH) {

					digitalWrite(SLIDER_LEFT_STEPPER_CLK, HIGH);
					digitalWrite(SLIDER_LEFT_STEPPER_CLK, HIGH);
					digitalWrite(SLIDER_LEFT_STEPPER_CLK, HIGH);
					digitalWrite(SLIDER_LEFT_STEPPER_CLK, HIGH);
					digitalWrite(SLIDER_LEFT_STEPPER_CLK, LOW);

					// Clear Left slider bit state
					output_slider_state &= ~SLIDER_LEFT_STATE_MASK;
					// Set Left slider bit state
					output_slider_state |= SLIDER_L_RETURNING;

				}
				else {

					// Clear Right slider bit state
					output_slider_state &= ~SLIDER_LEFT_STATE_MASK;
					// Set Left slider bit state
					output_slider_state |= SLIDER_L_HOME;

				}

			}

			// If not zero then command is open
			if((input_slider_state & SLIDER_RIGHT_CMD_MASK) == SLIDER_RO_CMD
			  &&
			 	(output_slider_state & SLIDER_RIGHT_STATE_MASK) != SLIDER_R_OPENED) {

				// Setup the stepper CCW mode
				digitalWrite(SLIDER_RIGHT_STEPPER_CW, HIGH);

				// If right slider limit switch is trigger, then STOP right slider opening
				if( digitalRead(SLIDER_RIGHT_STOP_SIGNAL_E) != HIGH) {

					digitalWrite(SLIDER_RIGHT_STEPPER_CLK, HIGH);
					digitalWrite(SLIDER_RIGHT_STEPPER_CLK, HIGH);
					digitalWrite(SLIDER_RIGHT_STEPPER_CLK, HIGH);
					digitalWrite(SLIDER_RIGHT_STEPPER_CLK, HIGH);
					digitalWrite(SLIDER_RIGHT_STEPPER_CLK, LOW);

					// Clear Left slider bit state
					output_slider_state &= ~SLIDER_RIGHT_STATE_MASK;
					// Set Left slider bit state
					output_slider_state |= SLIDER_R_OPENING;

				}
				else {

					// Clear Right slider bit state
					output_slider_state &= ~SLIDER_RIGHT_STATE_MASK;
					// Set Left slider bit state
					output_slider_state |= SLIDER_R_OPENED;

				}
			}

			if((input_slider_state & SLIDER_RIGHT_CMD_MASK) == SLIDER_RC_CMD
				&&
				(output_slider_state & SLIDER_RIGHT_STATE_MASK) != SLIDER_R_HOME) {

				// Setup the stepper CCW mode
				digitalWrite(SLIDER_RIGHT_STEPPER_CW, LOW);

				// If right slider limit switch is trigger, then STOP right slider closing
				if( digitalRead(SLIDER_RIGHT_STOP_SIGNAL_S) != HIGH) {

					digitalWrite(SLIDER_RIGHT_STEPPER_CLK, HIGH);
					digitalWrite(SLIDER_RIGHT_STEPPER_CLK, HIGH);
					digitalWrite(SLIDER_RIGHT_STEPPER_CLK, HIGH);
					digitalWrite(SLIDER_RIGHT_STEPPER_CLK, HIGH);
					digitalWrite(SLIDER_RIGHT_STEPPER_CLK, LOW);

					// Clear Left slider bit state
					output_slider_state &= ~SLIDER_RIGHT_STATE_MASK;
					// Set Left slider bit state
					output_slider_state |= SLIDER_R_RETURNING;

				}
				else {

					// Clear Right slider bit state
				  output_slider_state &= ~SLIDER_RIGHT_STATE_MASK;
					// Set Left slider bit state
					output_slider_state |= SLIDER_R_HOME;

				}
			}

			// Write the slider state to the register
			mb.Hreg(StateHoldRegister::SLIDER_STATE, output_slider_state );
	}


	// Whatever Silider mode is home or not
	// ARM HOME CMD Valid
	if (input_arm_mode == ArmModeCMD::CMD_HOME) {

			AX12_POS(MOTOR_RIGHT_ARM2_ID, RIGHT_MOTOR2_INIT_VALUE);
			AX12_POS(MOTOR_RIGHT_ARM1_ID, RIGHT_MOTOR1_INIT_VALUE);

			// Initial the servo catch value
			set_catch_level(CATCH_SERVO_INIT_VALUE);

			output_motor1_degree = RIGHT_MOTOR1_INIT_VALUE;
			mb.Hreg(StateHoldRegister::LEFT_MOTOR1_DEGREE, output_motor1_degree );

			output_motor2_degree = RIGHT_MOTOR2_INIT_VALUE;
			mb.Hreg(StateHoldRegister::LEFT_MOTOR2_DEGREE, output_motor2_degree );

			output_arm_mode = ArmMode::ARM_HOME;
			mb.Hreg(StateHoldRegister::ARM_MODE, output_arm_mode );
	}

	// Right Slider is opened
	// Then ARM BUTTON POSE CMD Valid
	if ((output_slider_state & SLIDER_RIGHT_STATE_MASK) == SLIDER_R_OPENED
		&&
		input_arm_mode == ArmModeCMD::CMD_BUTTON_POSE ) {

			AX12_POS(MOTOR_RIGHT_ARM2_ID, RIGHT_MOTOR2_BTN_POSE);
			AX12_POS(MOTOR_RIGHT_ARM1_ID, RIGHT_MOTOR1_BTN_POSE);

			// Initial the servo catch value
			set_catch_level(CATCH_SERVO_INIT_VALUE);

			output_motor1_degree = RIGHT_MOTOR1_BTN_POSE;
			mb.Hreg(StateHoldRegister::LEFT_MOTOR1_DEGREE, output_motor1_degree );

			output_motor2_degree = RIGHT_MOTOR2_BTN_POSE;
			mb.Hreg(StateHoldRegister::LEFT_MOTOR2_DEGREE, output_motor2_degree );

			output_arm_mode = ArmMode::BUTTON_POSE;
			mb.Hreg(StateHoldRegister::ARM_MODE, output_arm_mode );
	}

  // Right Slider is opened
	// Then ARM FREE CONTROLL CMD Valid
	if ((output_slider_state & SLIDER_RIGHT_STATE_MASK) == SLIDER_R_OPENED
		&&
		input_arm_mode == ArmModeCMD::CMD_FREE_CONTROLL) {

			AX12_POS(MOTOR_RIGHT_ARM2_ID, input_motor2_degree);
			AX12_POS(MOTOR_RIGHT_ARM1_ID, input_motor1_degree);

			output_motor1_degree = input_motor1_degree;
			mb.Hreg(StateHoldRegister::LEFT_MOTOR1_DEGREE, output_motor1_degree );

			output_motor2_degree = input_motor2_degree;
			mb.Hreg(StateHoldRegister::LEFT_MOTOR2_DEGREE, output_motor2_degree );

			output_arm_mode = ArmMode::FREE_CONTROLL;
			mb.Hreg(StateHoldRegister::ARM_MODE, output_arm_mode );

			// Set the servo catch value
			set_catch_level(input_effort_catch_level);

			output_effort_catch_level = input_effort_catch_level;
			mb.Hreg(StateHoldRegister::EFFORT_CATCH_LEVEL, output_effort_catch_level );
	}
}

void init_modbus(){

	// Config Mosbus Serial
	mb.config(&Serial3, 115200, SERIAL_8N2);

	// Config Slave ID
	mb.setSlaveId(MB_SLAVER_ID_);

	// Add the mapping register to specifid Register
	mb.addHreg(StateHoldRegister::LEFT_MOTOR1_DEGREE);
	mb.addHreg(StateHoldRegister::LEFT_MOTOR2_DEGREE);
	mb.addHreg(StateHoldRegister::SLIDER_STATE);
	mb.addHreg(StateHoldRegister::ARM_MODE);
	mb.addHreg(StateHoldRegister::EFFORT_CATCH_LEVEL);
	mb.addHreg(StateHoldRegister::CMD_LEFT_MOTOR1_DEGREE);
	mb.addHreg(StateHoldRegister::CMD_LEFT_MOTOR2_DEGREE);
	mb.addHreg(StateHoldRegister::CMD_ARM_MODE);
	mb.addHreg(StateHoldRegister::CMD_SLIDER_STATE);

	// Initial the register
	mb.Hreg(LEFT_MOTOR1_DEGREE, LEFT_MOTOR1_INIT_VALUE);
	mb.Hreg(LEFT_MOTOR2_DEGREE, LEFT_MOTOR2_INIT_VALUE);
	mb.Hreg(SLIDER_STATE, SLIDER_L_OPENED | SLIDER_R_OPENED );
	// mb.Hreg(SLIDER_STATE, SliderState::HOME);
	mb.Hreg(ARM_MODE, ArmMode::ARM_HOME);
	mb.Hreg(EFFORT_CATCH_LEVEL, CATCH_SERVO_INIT_VALUE);
	mb.Hreg(CMD_LEFT_MOTOR1_DEGREE, LEFT_MOTOR1_INIT_VALUE);
	mb.Hreg(CMD_LEFT_MOTOR2_DEGREE, LEFT_MOTOR2_INIT_VALUE);
	mb.Hreg(CMD_SLIDER_STATE, SLIDER_LC_CMD | SLIDER_RC_CMD );
	// mb.Hreg(CMD_SLIDER_STATE, SliderStateCMD::OPEN);
	mb.Hreg(CMD_ARM_MODE, ArmModeCMD::CMD_HOME);

}

void init_pin() {

		// Setup stepper motor pin
	pinMode(SLIDER_LEFT_STEPPER_EN, OUTPUT);
	pinMode(SLIDER_LEFT_STEPPER_CW, OUTPUT);
	pinMode(SLIDER_LEFT_STEPPER_CLK, OUTPUT);

	pinMode(SLIDER_RIGHT_STEPPER_EN, OUTPUT);
	pinMode(SLIDER_RIGHT_STEPPER_CW, OUTPUT);
	pinMode(SLIDER_RIGHT_STEPPER_CLK, OUTPUT);

	// Setup the Limit Switch
	pinMode(SLIDER_LEFT_STOP_SIGNAL_E, INPUT);
	pinMode(SLIDER_LEFT_STOP_SIGNAL_S, INPUT);
	pinMode(SLIDER_RIGHT_STOP_SIGNAL_E, INPUT);
	pinMode(SLIDER_RIGHT_STOP_SIGNAL_S, INPUT);

	// Setup the Catch Servo
	right_catch_servo.attach(LEFT_CATCH_SERVO);
	// Remapping the servo value
	int servo_value(0);
	servo_value = map(CATCH_SERVO_INIT_VALUE,
		0,
		100,
		CATCH_SERVO_FULL_OPEN_VALUE,
		CATCH_SERVO_FULL_CLOSE_VALUE);
	right_catch_servo.write(servo_value);

	// Enable stepper motor
	digitalWrite(SLIDER_LEFT_STEPPER_EN, LOW);
	digitalWrite(SLIDER_RIGHT_STEPPER_EN, LOW);

}

void init_ax_motor() {

	// For AI Motor communication
	Serial1.begin(1000000);

	// let Left Motor go to home pose
	AX12_POS(MOTOR_LEFT_ARM2_ID, LEFT_MOTOR2_INIT_VALUE);
	AX12_POS(MOTOR_LEFT_ARM1_ID, LEFT_MOTOR1_INIT_VALUE);

}

void modbus_sync() {

	// sync all state immediately
	mb.task();

	// Read all ModbusSerial mb context to global variable
	input_motor1_degree				= mb.Hreg(StateHoldRegister::CMD_LEFT_MOTOR1_DEGREE);
	input_motor2_degree				= mb.Hreg(StateHoldRegister::CMD_LEFT_MOTOR2_DEGREE);
	input_slider_state				= mb.Hreg(StateHoldRegister::CMD_SLIDER_STATE);
	input_arm_mode						= mb.Hreg(StateHoldRegister::CMD_ARM_MODE);
	input_effort_catch_level	= mb.Hreg(StateHoldRegister::EFFORT_CATCH_LEVEL);

	// Write all global variable to ModbusSerial mb context
	output_motor1_degree			= mb.Hreg(StateHoldRegister::LEFT_MOTOR1_DEGREE);
	output_motor2_degree			= mb.Hreg(StateHoldRegister::LEFT_MOTOR2_DEGREE);
	output_slider_state				= mb.Hreg(StateHoldRegister::SLIDER_STATE);
	output_arm_mode						= mb.Hreg(StateHoldRegister::ARM_MODE);
	output_effort_catch_level	= mb.Hreg(StateHoldRegister::EFFORT_CATCH_LEVEL);

}

void AX12_POS(int id, int data)
{

	int sum = 0;
	//���m�\�� FF FF ID 05 03 1e data1 data2 �����X
	//�����X=not(id_2 + 5 + 3 + 1e + data1 + data2)
	byte AX12_OUT[9] = { 0xff,0xff,0x01,0x05,0x03,0x1e,0x00,0x00,0xd2 };
	sum = id + 5 + 3 + 30 + (data / 256) + (data % 256);//�p�ⰻ���X
	AX12_OUT[2] = byte(id);
	AX12_OUT[6] = byte(data % 256); //���g�C�줸���Ʀ�
	AX12_OUT[7] = byte(data / 256); //���g���줸���Ʀ�
	AX12_OUT[8] = ~(byte(sum));
	for (int i = 0; i < 9; i++)
	{
		//send a content
		Serial1.write(AX12_OUT[i]);
	}

#ifdef DEBUG
	Serial.print("AX-12 motor packet");
	Serial.print('\n');
	for (int i = 0; i < 9; i++) {
		Serial.print(AX12_OUT[i], HEX);
		Serial.print('\t');
	}
	Serial.print('\n');
#endif // DEBUG
}

void set_catch_level(int _catch_level) {
	int servo_value(0);

	if(output_motor2_degree > RIGHT_MOTOR2_CLOSE_CATCH_THRESHOLD)
		_catch_level = CATCH_SERVO_INIT_VALUE;

	servo_value = map(_catch_level,
		0,
		100,
		CATCH_SERVO_FULL_OPEN_VALUE,
		CATCH_SERVO_FULL_CLOSE_VALUE);
		right_catch_servo.write(servo_value);
	return;
}

// // Trigger Modbus when Serial3 interrupt
// void serialEvent3() {
// 	// Do modbus tesk : Receive and reply
// 	mb.task();
// }
