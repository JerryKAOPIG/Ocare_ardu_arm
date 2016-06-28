
#include <Modbus.h>
#include <ModbusSerial.h>

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

const byte motor_1_id = 1;
const byte motor_2_id = 2;

// Modbus object
ModbusSerial mb;

uint16_t input_motor1_degree			= 0;
uint16_t input_motor2_degree			= 0;
uint16_t input_slider_state				= 0;
uint16_t input_arm_mode					= 0;
uint16_t input_effort_catch_level		= 0;

uint16_t output_motor1_degree			= 0;
uint16_t output_motor2_degree			= 0;
uint16_t output_slider_state			= 0;
uint16_t output_arm_mode				= 0;
uint16_t output_effort_catch_level		= 0;

long int time = 0;

const int Enable_Stepper_motor = 6;
const int Select_Rotation_Direction = 7;
const int Rotation_Times = 8;

const long int ref_count = 120000;
long int count = 0;

void setup()
{

  // Config Mosbus Serial
	mb.config(&Serial3, 115200, SERIAL_8N2);
	// Config Slave ID
	mb.setSlaveId(5);

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

	// Setup for 7-SEG Display
	DDRA |= 0x0F;

	// stepper motor pin
	pinMode(Enable_Stepper_motor, OUTPUT);
	pinMode(Select_Rotation_Direction, OUTPUT);
	pinMode(Rotation_Times, OUTPUT);

}

void loop()
{
	Serial.println(micros() - time);
	time = micros();
	

	//Logic Loop
	input_motor1_degree			= mb.Hreg(StateHoldRegister::CMD_LEFT_MOTOR1_DEGREE);
	input_motor2_degree			= mb.Hreg(StateHoldRegister::CMD_LEFT_MOTOR2_DEGREE);
	input_slider_state			= mb.Hreg(StateHoldRegister::CMD_SLIDER_STATE);
	input_arm_mode				= mb.Hreg(StateHoldRegister::CMD_ARM_MODE);
	input_effort_catch_level	= mb.Hreg(StateHoldRegister::EFFORT_CATCH_LEVEL);

	output_motor1_degree		= mb.Hreg(StateHoldRegister::LEFT_MOTOR1_DEGREE);
	output_motor2_degree		= mb.Hreg(StateHoldRegister::LEFT_MOTOR2_DEGREE);
	output_slider_state			= mb.Hreg(StateHoldRegister::SLIDER_STATE);
	output_arm_mode				= mb.Hreg(StateHoldRegister::ARM_MODE);
	output_effort_catch_level	= mb.Hreg(StateHoldRegister::EFFORT_CATCH_LEVEL);

	PORTA = output_arm_mode;

	// enable stepper motor 
	digitalWrite(Enable_Stepper_motor, LOW);

	if ((output_slider_state == SliderState::HOME || output_slider_state == SliderState::OPENING) &&
		output_arm_mode == ArmMode::ARM_HOME &&
		output_effort_catch_level == 0 &&
		input_slider_state == SliderStateCMD::OPEN) {
	
			digitalWrite(Select_Rotation_Direction, LOW);
			
			if (count < ref_count) {

				digitalWrite(Rotation_Times, HIGH);
				digitalWrite(Rotation_Times, HIGH);
				digitalWrite(Rotation_Times, HIGH);
				digitalWrite(Rotation_Times, HIGH);
				digitalWrite(Rotation_Times, LOW);

				count++;
				output_slider_state = SliderState::OPENING;
			}
			
			if (count == ref_count) {
			
				output_slider_state = SliderState::OPENED;
			}
		
			mb.Hreg(StateHoldRegister::SLIDER_STATE, output_slider_state );
	}

	if (output_slider_state == SliderState::OPENED &&
		input_arm_mode == ArmModeCMD::CMD_HOME) {

			AX12_POS(motor_1_id, 512);
			AX12_POS(motor_2_id, 512 - 307);
			
			output_motor1_degree = 512;
			mb.Hreg(StateHoldRegister::LEFT_MOTOR1_DEGREE, output_motor1_degree );

			output_motor2_degree = 512 -307;
			mb.Hreg(StateHoldRegister::LEFT_MOTOR2_DEGREE, output_motor2_degree );

			output_arm_mode = ArmMode::ARM_HOME;
			mb.Hreg(StateHoldRegister::ARM_MODE, output_arm_mode );
	}

	if (output_slider_state == SliderState::OPENED &&
		input_arm_mode == ArmModeCMD::CMD_BUTTON_POSE ) {
		
			AX12_POS(motor_1_id, 512 + 307);
			AX12_POS(motor_2_id, 512 + 307);
			
			output_motor1_degree = 512 + 307;
			mb.Hreg(StateHoldRegister::LEFT_MOTOR1_DEGREE, output_motor1_degree );

			output_motor2_degree = 512 + 307;
			mb.Hreg(StateHoldRegister::LEFT_MOTOR2_DEGREE, output_motor2_degree );

			output_arm_mode = ArmMode::BUTTON_POSE;
			mb.Hreg(StateHoldRegister::ARM_MODE, output_arm_mode );
	}

	if (output_slider_state == SliderState::OPENED &&
		input_arm_mode == ArmModeCMD::CMD_FREE_CONTROLL) {

			AX12_POS(motor_1_id, input_motor1_degree);
			AX12_POS(motor_2_id, input_motor2_degree);
			
			output_motor1_degree = input_motor1_degree;
			mb.Hreg(StateHoldRegister::LEFT_MOTOR1_DEGREE, output_motor1_degree );

			output_motor2_degree = input_motor2_degree;
			mb.Hreg(StateHoldRegister::LEFT_MOTOR2_DEGREE, output_motor2_degree );

			output_arm_mode = ArmMode::FREE_CONTROLL;
			mb.Hreg(StateHoldRegister::ARM_MODE, output_arm_mode );

			output_effort_catch_level = input_effort_catch_level;
			mb.Hreg(StateHoldRegister::EFFORT_CATCH_LEVEL, output_effort_catch_level );
	}

	if (output_arm_mode == ArmMode::ARM_HOME &&
		output_effort_catch_level == 0 &&
		(output_slider_state == SliderState::RETURNING || output_slider_state == SliderState::OPENED) &&
		input_slider_state == SliderStateCMD::CLOSE) {

		digitalWrite(Select_Rotation_Direction, HIGH);

		if (count > 0) {

			digitalWrite(Rotation_Times, HIGH);
			digitalWrite(Rotation_Times, HIGH);
			digitalWrite(Rotation_Times, HIGH);
			digitalWrite(Rotation_Times, HIGH);
			digitalWrite(Rotation_Times, LOW);

			count--;

			output_slider_state = SliderState::RETURNING;
		}

		if (count == 0) {

			output_slider_state = SliderState::HOME;
		}

		mb.Hreg(StateHoldRegister::SLIDER_STATE, output_slider_state);
	}
}

void AX12_POS(int id, int data)
{
	
	int sum = 0;
	//位置功能 FF FF ID 05 03 1e data1 data2 偵錯碼
	//偵錯碼=not(id_2 + 5 + 3 + 1e + data1 + data2)
	byte AX12_OUT[9] = { 0xff,0xff,0x01,0x05,0x03,0x1e,0x00,0x00,0xd2 };
	sum = id + 5 + 3 + 30 + (data / 256) + (data % 256);//計算偵錯碼
	AX12_OUT[2] = byte(id);
	AX12_OUT[6] = byte(data % 256); //改寫低位元資料串
	AX12_OUT[7] = byte(data / 256); //改寫高位元資料串
	AX12_OUT[8] = ~(byte(sum));
	for (int i = 0; i < 9; i++)
	{
		Serial1.write(AX12_OUT[i]); //send a content
		//delay(5); //delay 5 ms
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

void serialEvent3() {
	// Do modbus tesk : Receive and reply
	mb.task();
}


