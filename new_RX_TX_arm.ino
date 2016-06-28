#include <Arduino.h>
#include <Modbus.h>
#include <ModbusSerial.h>

#define DEBUG

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
* HOME :       Slider at Home position
* OPENING:     Slider Opening now
* OPENED:      Slider is Opened, the arm can work around now
* RETURNING:   Slider is Returning to HOME now
* **************************/
enum SliderState {
	HOME,
	OPENING,
	OPENED,
	RETURNING
};

/******Defination of the Slider Command
* OPEN :           Open the slider to the right position
* CLOSE:           Close the slider to the home position
*
* **************************/
enum SliderStateCMD {
	OPEN,
	CLOSE
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

#define MOTOR_LEFT_ARM1_ID 					(1)
#define MOTOR_LEFT_ARM2_ID 					(2)

#define LEFT_SLIDER_STOP_SIGNAL_E		(22)
#define LEFT_SLIDER_STOP_SIGNAL_S		(24)
#define LEFT_SLIDER_STEPPER_EN  		(26)
#define LEFT_SLIDER_STEPPER_CW  		(28)
#define LEFT_SLIDER_STEPPER_CLK  		(30)

#define RIGHT_SLIDER_STOP_SIGNAL_E	(23)
#define RIGHT_SLIDER_STOP_SIGNAL_S	(25)
#define RIGHT_SLIDER_STEPPER_EN			(27)
#define RIGHT_SLIDER_STEPPER_CW			(29)
#define RIGHT_SLIDER_STEPPER_CLK		(31)

/********************************************************/

// The clk count used to push the slider via Stepper Motor at 16 step mode
#define ref_count 								(120000)

// Modbus context object
ModbusSerial mb;

uint16_t input_motor1_degree				= 0;
uint16_t input_motor2_degree				= 0;
uint16_t input_slider_state					= 0;
uint16_t input_arm_mode							= 0;
uint16_t input_effort_catch_level		= 0;

uint16_t output_motor1_degree				= 0;
uint16_t output_motor2_degree				= 0;
uint16_t output_slider_state				= 0;
uint16_t output_arm_mode						= 0;
uint16_t output_effort_catch_level	= 0;

long int time = 0;
long int count = 0;

void setup()
{

  // Config Mosbus Serial
	mb.config(&Serial3, 115200, SERIAL_8N2);
	// Config Slave ID
	mb.setSlaveId(5);

	// Setup the DEBUG tunnel via serial0(USB port)
	Serial.begin(115200);

	// Add the mapping register to specifid Register
	mb.addHreg(LEFT_MOTOR1_DEGREE);
	mb.addHreg(LEFT_MOTOR2_DEGREE);
	mb.addHreg(SLIDER_STATE);
	mb.addHreg(ARM_MODE);
	mb.addHreg(EFFORT_CATCH_LEVEL);
	mb.addHreg(CMD_LEFT_MOTOR1_DEGREE);
	mb.addHreg(CMD_LEFT_MOTOR2_DEGREE);
	mb.addHreg(CMD_SLIDER_STATE);
	mb.addHreg(CMD_ARM_MODE);

	// Initial the register
	mb.Hreg(LEFT_MOTOR1_DEGREE, 0);
	mb.Hreg(LEFT_MOTOR2_DEGREE, 0);
	mb.Hreg(SLIDER_STATE, SliderState::HOME);
	mb.Hreg(ARM_MODE, ArmMode::ARM_HOME);
	mb.Hreg(EFFORT_CATCH_LEVEL, 0);
	mb.Hreg(CMD_LEFT_MOTOR1_DEGREE, 512);
	mb.Hreg(CMD_LEFT_MOTOR2_DEGREE, 512 -307);
	mb.Hreg(CMD_SLIDER_STATE, SliderStateCMD::CLOSE);
	mb.Hreg(CMD_ARM_MODE, ArmModeCMD::CMD_HOME);



	// For AI Motor communication
	Serial1.begin(1000000);

	// Setup stepper motor pin
	pinMode(LEFT_SLIDER_STEPPER_EN, OUTPUT);
	pinMode(LEFT_SLIDER_STEPPER_CW, OUTPUT);
	pinMode(LEFT_SLIDER_STEPPER_CLK, OUTPUT);

	pinMode(RIGHT_SLIDER_STEPPER_EN, OUTPUT);
	pinMode(RIGHT_SLIDER_STEPPER_CW, OUTPUT);
	pinMode(RIGHT_SLIDER_STEPPER_CLK, OUTPUT);

	// Setup the Limit Switch
	pinMode(LEFT_SLIDER_STOP_SIGNAL_E, INPUT);
	pinMode(LEFT_SLIDER_STOP_SIGNAL_S, INPUT);
	pinMode(RIGHT_SLIDER_STOP_SIGNAL_E, INPUT);
	pinMode(RIGHT_SLIDER_STOP_SIGNAL_S, INPUT);

	// Enable stepper motor
	digitalWrite(LEFT_SLIDER_STEPPER_EN, LOW);

}

void loop()
{

	// Show the current time via Serial port
	Serial.println(micros() - time);
	// Get the current time
	time = micros();

	/********** Update the register and sync ************/

	// Note : the sync modbus task will be trigger via Serial3 interrupt


	//Logic Loop
	input_motor1_degree				= mb.Hreg(StateHoldRegister::CMD_LEFT_MOTOR1_DEGREE);
	input_motor2_degree				= mb.Hreg(StateHoldRegister::CMD_LEFT_MOTOR2_DEGREE);
	input_slider_state				= mb.Hreg(StateHoldRegister::CMD_SLIDER_STATE);
	input_arm_mode						= mb.Hreg(StateHoldRegister::CMD_ARM_MODE);
	input_effort_catch_level	= mb.Hreg(StateHoldRegister::EFFORT_CATCH_LEVEL);

	output_motor1_degree			= mb.Hreg(StateHoldRegister::LEFT_MOTOR1_DEGREE);
	output_motor2_degree			= mb.Hreg(StateHoldRegister::LEFT_MOTOR2_DEGREE);
	output_slider_state				= mb.Hreg(StateHoldRegister::SLIDER_STATE);
	output_arm_mode						= mb.Hreg(StateHoldRegister::ARM_MODE);
	output_effort_catch_level	= mb.Hreg(StateHoldRegister::EFFORT_CATCH_LEVEL);

	/****************************************************/

	// Slider is home OR opening
	// ARM is home
	// Effot catch level is 0
	// Then Slide OPEN Valid
	if ((output_slider_state == SliderState::HOME || output_slider_state == SliderState::OPENING) &&
		output_arm_mode == ArmMode::ARM_HOME &&
		output_effort_catch_level == 0 &&
		input_slider_state == SliderStateCMD::OPEN) {

			// Setup the stepper CCW mode
			digitalWrite(LEFT_SLIDER_STEPPER_CW, LOW);
			digitalWrite(RIGHT_SLIDER_STEPPER_CW, LOW);

			// If left slider limit switch is trigger, then STOP left lisder opening
			if( digitalRead(LEFT_SLIDER_STOP_SIGNAL_E) != 1) {

				digitalWrite(LEFT_SLIDER_STEPPER_CLK, HIGH);
				digitalWrite(LEFT_SLIDER_STEPPER_CLK, HIGH);
				digitalWrite(LEFT_SLIDER_STEPPER_CLK, HIGH);
				digitalWrite(LEFT_SLIDER_STEPPER_CLK, HIGH);
				digitalWrite(LEFT_SLIDER_STEPPER_CLK, LOW);

			}

			// If right slider limit switch is trigger, then STOP right lisder opening
			if( digitalRead(RIGHT_SLIDER_STOP_SIGNAL_E) != 1) {

				digitalWrite(RIGHT_SLIDER_STEPPER_CLK, HIGH);
				digitalWrite(RIGHT_SLIDER_STEPPER_CLK, HIGH);
				digitalWrite(RIGHT_SLIDER_STEPPER_CLK, HIGH);
				digitalWrite(RIGHT_SLIDER_STEPPER_CLK, HIGH);
				digitalWrite(RIGHT_SLIDER_STEPPER_CLK, LOW);

			}

			// If all Limit switch is triggered.
			// Then Slider status switch to OPENED
			if (digitalRead(LEFT_SLIDER_STOP_SIGNAL_E) == 1 &&
			 		digitalRead(RIGHT_SLIDER_STOP_SIGNAL_E) == 1) {

				output_slider_state = SliderState::OPENED;
			}

			// Write the slider state to the register
			mb.Hreg(StateHoldRegister::SLIDER_STATE, output_slider_state );
	}


	// Slider is opened
	// Then ARM HOME CMD Valid
	if (output_slider_state == SliderState::OPENED &&
		input_arm_mode == ArmModeCMD::CMD_HOME) {

			AX12_POS(MOTOR_LEFT_ARM1_ID, 512);
			AX12_POS(MOTOR_LEFT_ARM2_ID, 512 - 307);

			output_motor1_degree = 512;
			mb.Hreg(StateHoldRegister::LEFT_MOTOR1_DEGREE, output_motor1_degree );

			output_motor2_degree = 512 -307;
			mb.Hreg(StateHoldRegister::LEFT_MOTOR2_DEGREE, output_motor2_degree );

			output_arm_mode = ArmMode::ARM_HOME;
			mb.Hreg(StateHoldRegister::ARM_MODE, output_arm_mode );
	}

	// Slider is opened
	// Then ARM BUTTON POSE CMD Valid
	if (output_slider_state == SliderState::OPENED &&
		input_arm_mode == ArmModeCMD::CMD_BUTTON_POSE ) {

			AX12_POS(MOTOR_LEFT_ARM1_ID, 512 + 307);
			AX12_POS(MOTOR_LEFT_ARM2_ID, 512 + 307);

			output_motor1_degree = 512 + 307;
			mb.Hreg(StateHoldRegister::LEFT_MOTOR1_DEGREE, output_motor1_degree );

			output_motor2_degree = 512 + 307;
			mb.Hreg(StateHoldRegister::LEFT_MOTOR2_DEGREE, output_motor2_degree );

			output_arm_mode = ArmMode::BUTTON_POSE;
			mb.Hreg(StateHoldRegister::ARM_MODE, output_arm_mode );
	}

  // Slider is opened
	// Then ARM FREE CONTROLL CMD Valid
	if (output_slider_state == SliderState::OPENED &&
		input_arm_mode == ArmModeCMD::CMD_FREE_CONTROLL) {

			AX12_POS(MOTOR_LEFT_ARM1_ID, input_motor1_degree);
			AX12_POS(MOTOR_LEFT_ARM2_ID, input_motor2_degree);

			output_motor1_degree = input_motor1_degree;
			mb.Hreg(StateHoldRegister::LEFT_MOTOR1_DEGREE, output_motor1_degree );

			output_motor2_degree = input_motor2_degree;
			mb.Hreg(StateHoldRegister::LEFT_MOTOR2_DEGREE, output_motor2_degree );

			output_arm_mode = ArmMode::FREE_CONTROLL;
			mb.Hreg(StateHoldRegister::ARM_MODE, output_arm_mode );

			output_effort_catch_level = input_effort_catch_level;
			mb.Hreg(StateHoldRegister::EFFORT_CATCH_LEVEL, output_effort_catch_level );
	}

	// ARM is HOME
	// Effort catch level is 0
	// Slider is closing OR opened
	// Then Slider CLOSE CMD Valid
	if (output_arm_mode == ArmMode::ARM_HOME &&
		output_effort_catch_level == 0 &&
		(output_slider_state == SliderState::RETURNING || output_slider_state == SliderState::OPENED) &&
		input_slider_state == SliderStateCMD::CLOSE) {

		// Setup the stepper CW mode
		digitalWrite(LEFT_SLIDER_STEPPER_CW, HIGH);
		digitalWrite(RIGHT_SLIDER_STEPPER_CW, HIGH);

		// If left slider limit switch is trigger, then STOP left slider closing
		if( digitalRead(LEFT_SLIDER_STOP_SIGNAL_S) != 1) {

			digitalWrite(LEFT_SLIDER_STEPPER_CLK, HIGH);
			digitalWrite(LEFT_SLIDER_STEPPER_CLK, HIGH);
			digitalWrite(LEFT_SLIDER_STEPPER_CLK, HIGH);
			digitalWrite(LEFT_SLIDER_STEPPER_CLK, HIGH);
			digitalWrite(LEFT_SLIDER_STEPPER_CLK, LOW);

		}

		// If right slider limit switch is trigger, then STOP right slider closing
		if( digitalRead(RIGHT_SLIDER_STOP_SIGNAL_S) != 1) {

			digitalWrite(RIGHT_SLIDER_STEPPER_CLK, HIGH);
			digitalWrite(RIGHT_SLIDER_STEPPER_CLK, HIGH);
			digitalWrite(RIGHT_SLIDER_STEPPER_CLK, HIGH);
			digitalWrite(RIGHT_SLIDER_STEPPER_CLK, HIGH);
			digitalWrite(RIGHT_SLIDER_STEPPER_CLK, LOW);

		}

		// If all Limit switch is triggered.
		// Then Slider status switch to HOME
		if (digitalRead(LEFT_SLIDER_STOP_SIGNAL_S) == 1 &&
				digitalRead(RIGHT_SLIDER_STOP_SIGNAL_S) == 1) {

			output_slider_state = SliderState::HOME;
		}

		// Write the slider state to the register
		mb.Hreg(StateHoldRegister::SLIDER_STATE, output_slider_state);
	}
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


// Trigger Modbus when Serial3 interrupt
void serialEvent3() {
	// Do modbus tesk : Receive and reply
	mb.task();
}
