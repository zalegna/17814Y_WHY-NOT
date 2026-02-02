#include "main.h"
#include "au/au.hpp"
#include "dlib/controllers/error_derivative_settler.hpp"
#include "dlib/controllers/error_time_settler.hpp" // IWYU pragma: keep
#include "dlib/controllers/feedforward.hpp" // IWYU pragma: keep
#include "dlib/controllers/pid.hpp"
#include "dlib/dlib.hpp" // IWYU pragma: keep
#include "dlib/hardware/chassis.hpp"
#include "dlib/hardware/imu.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp" // IWYU pragma: keep
#include "pros/motors.hpp" // IWYU pragma: keep
#include "pros/optical.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include <cstdio>
#include <initializer_list>
#include <iostream>
using namespace au;

signed char y;

//BASE CODE: Template code
//Blocks scored:

//Drive motors
pros::MotorGroup leftMotors ({-12, 11, -13}, pros::v5::MotorGears::rpm_600);
pros::MotorGroup rightMotors ({18, -19, 20}, pros::v5::MotorGears::rpm_600);

//Intake motors
pros::Motor bottomStage (-4);
pros::Motor midStage(-3);
pros::Motor topStage(-2);

//Pistons
pros::adi::DigitalOut loader('C');
pros::adi::DigitalOut wing('D');
pros::adi::DigitalOut scoreBlocker('B');
pros::adi::DigitalOut intakeUpE('E');
pros::adi::DigitalOut intakeUpF('F');

//Sensors
pros::Imu imu(5);
pros::Distance distSns(8);

//test motor
// pros::Motor test(11);

pros::Controller controller (pros::E_CONTROLLER_MASTER);

//////////////////////////////////////////////////////////////////////////////////////
//PID configuration
/////////////////////////////////////////////////////////////////////////////////////
//Robot class and methods. No touchie
class Robot {
public:
	dlib::Chassis chassis;
	dlib::Imu imu;

	dlib::Odometry odom;
	std::unique_ptr<pros::Task> odom_updater;

	dlib::Pid<Meters> move_pid;
	dlib::ErrorDerivativeSettler<Meters> move_settler;

	dlib::Pid<Degrees> turn_pid;
	dlib::ErrorDerivativeSettler<Degrees> turn_settler;

	Robot(
		dlib::ChassisConfig chassis_config, 
		dlib::ImuConfig imu_config,
		dlib::PidConfig move_pid_config,
		dlib::ErrorDerivativeSettler<Meters> move_pid_settler,
		dlib::PidConfig turn_pid_config,
		dlib::ErrorDerivativeSettler<Degrees> turn_pid_settler
	) : 
		chassis(chassis_config), 
		imu(imu_config),
		move_pid(move_pid_config),
		move_settler(move_pid_settler),
		turn_pid(turn_pid_config),
		turn_settler(turn_pid_settler),
		odom() {

	}

	Quantity<Degrees, double> getHeading(){
		return imu.get_heading();
	}
	void calibrate_imu() {
		imu.initialize();
	}

	// Odom task
	void start_odom() {
		chassis.initialize();

		odom_updater = std::make_unique<pros::Task>([this]() {
			while (true) {
				odom.update(
					chassis.left_motors_displacement(), 
					chassis.right_motors_displacement(), 
					ZERO,
					imu.get_rotation()
				);

				pros::delay(20);
			}
		});
	}

	// Use PID to do a relative movement
	void move_with_pid(Quantity<Meters, double> displacement, double maxVoltage=12) {
		auto start_displacement = chassis.forward_motor_displacement();
		auto target_displacement = dlib::relative_target(start_displacement, displacement);
		
		move_pid.reset();
		move_settler.reset();

		while (!move_settler.is_settled(move_pid.get_error(), move_pid.get_derivative())) {
			auto error = dlib::linear_error(target_displacement, chassis.forward_motor_displacement());
			auto voltage = move_pid.update(error, milli(seconds)(20));
			
			if (abs(voltage.in(volts))>maxVoltage){ 
				if(voltage.in(volts)<0)
					voltage = -volts(maxVoltage); 
				else
					voltage = volts(maxVoltage);
			}
			
			chassis.move_voltage(voltage);
			
			pros::delay(20);
		}
	}

	void move_with_pid(double displacement, double maxVoltage=12) {
		move_with_pid(inches(displacement), maxVoltage);
	}

	void turn_with_pid(Quantity<Degrees, double> heading, double maxVoltage=12) {
		auto target_heading = heading;

		turn_pid.reset();
		turn_settler.reset();

		while (!turn_settler.is_settled(turn_pid.get_error(), turn_pid.get_derivative())) {
			auto error = dlib::angular_error(heading, imu.get_rotation());
			auto voltage = turn_pid.update(error, milli(seconds)(20));
			
			if (abs(voltage.in(volts))>maxVoltage){ 
				if(voltage.in(volts)<0)
					voltage = -volts(maxVoltage); 
				else
					voltage = volts(maxVoltage);
			}
			
			chassis.turn_voltage(-voltage);
			std::cout << error << std::endl;
			pros::delay(20);
		}
	}

	void turn_with_pid(double heading, double maxVoltage=12) {
		turn_with_pid(degrees(heading), maxVoltage);
	}

	void turn_to_point(dlib::Vector2d point) {
		auto angle = odom.angle_to(point);
		turn_with_pid(angle);
	}

	void turn_to_point(double x, double y) {
		turn_to_point({inches(x), inches(y)});
	}

	void move_to_point(dlib::Vector2d point) {
		turn_to_point(point);

		auto displacement = odom.displacement_to(point);
		move_with_pid(displacement);
	}

	void move_to_point(double x, double y) {
		move_to_point({inches(x), inches(y)});
	}
};

// Configure the chassis!!
dlib::ChassisConfig chassis_config = dlib::ChassisConfig::from_drive_rpm(
	{{-12, 11, -13}},	// right motors	// left motors
	{{18, -19, 20}},	// right motors
	pros::MotorGearset::blue,
	rpm(450),	// the drivebase rpm
	inches(3.25)	// the drivebase wheel diameter
);

//PID Tuning and Configurations!!!
dlib::ImuConfig imu_config {
	5,	// imu port
	1.00	// optional imu scaling constant
};

//PID TUNING - LATERAL MOVEMENT
//1. In autonomous(), put in the command moveForward(24). This SHOULD make the robot move forward one tile.
//2. Test it to make sure it runs. There's a good chance it'll oscillate back and forth.
//3. Begin tuning the gains, which are found below. Kp is the proportional gain, and is how reactive the system is. 
// More Kp means more oscillations.
//4. Next tune Kd, which is the derviative gain. This will damp the oscillations.
//5. After that, tune the error derivative settler. This will make the robot move on based on how close it is to setpoint.

//Tuning tips - Kd should be a lot smaller than Kp. You want higher Kp in general.
dlib::PidConfig move_pid_config {
	{
		60, 	// kp, porportional gain
		0, 	// ki, integral gain
		4 	// kd, derivative gain
	}
};
dlib::ErrorDerivativeSettler<Meters> move_pid_settler {
	meters(0.4),		// error threshold, the maximum error the pid can settle at
	meters_per_second(0.01) // derivative threshold, the maximum instantaneous error over time the pid can settle at
};
dlib::PidConfig turn_pid_gains {
	{
		26, 	// kp, porportional gain
		0, 	    // ki, integral gain
		2       // kd, derivative gain
	}
};
dlib::ErrorDerivativeSettler<Degrees> turn_pid_settler {
	degrees(5),		// error threshold, the maximum error the pid can settle at
	degrees_per_second(1.1)	// derivative threshold, the maximum instantaneous error over time the pid can settle at
};

//Create the robot object
Robot robot = Robot(
	chassis_config,
	imu_config,
	move_pid_config,
	move_pid_settler,
	turn_pid_gains,
	turn_pid_settler
);

//////////////////////////////////////////////////////////////////////////////////////
//Initalize
/////////////////////////////////////////////////////////////////////////////////////
void initialize() {
	auto profile = dlib::TrapezoidProfile<Meters>::from_constraints(
		centi(meters)(60),
		meters_per_second(1.5),
		meters_per_second_squared(3)
	);
	// imu.reset();
	profile.calculate(seconds(0));
	robot.calibrate_imu();
}

void disabled() {}

void competition_initialize() {}

//////////////////////////////////////////////////////////////////////////////////////
//Drive Code
/////////////////////////////////////////////////////////////////////////////////////

//Toggle variables 
bool loaderToggle=false;
bool wingToggle=false;
bool scoreBlockerToggle=false;
bool intakeUpToggleDir=false;
bool intakeUpTogglePow=false;


int iv=600;

// bool wrongColor(int your_color){
// 	if ((your_color==1&&optical.get_hue()>160&&optical.get_hue()<230) || (your_color==2&&optical.get_hue()<20)){
// 		return true;
// 	}
// 	else
// 		return false;
// }

int maxSpeed=600;

void opcontrol() {
	// Tasks here
	pros::Task printHeading([]{
		while (true) {
			//To print messages, connect to the terminal through the command "pros t"
			std::cout << imu.get_heading() << std::endl;
			pros::delay(500);
		}
	});
	while (true) {
		int distance=distSns.get_distance();

		//Drive code (tank drive)
		leftMotors.move(controller.get_analog(ANALOG_LEFT_Y));
		rightMotors.move(controller.get_analog(ANALOG_RIGHT_Y));

		//Intake code 
		if(controller.get_digital(DIGITAL_R1)){
			topStage.move_velocity(500);
			bottomStage.move_velocity(600);
			midStage.move_velocity(600);
			if (controller.get_digital(DIGITAL_L1)) {
				topStage.move_velocity(-400);
			}
		}
		else if(controller.get_digital(DIGITAL_R2)){
			topStage.move_velocity(-maxSpeed);
			bottomStage.move_velocity(-600);
			midStage.move_velocity(-600);
			if (controller.get_digital(DIGITAL_L1)) 
				topStage.move_velocity(maxSpeed);
		}
		else if(controller.get_digital(DIGITAL_B)){
			if (distance<100){
				topStage.move_velocity(0);
				bottomStage.move_velocity(0);
				midStage.move_velocity(0);

				intakeUpTogglePow=true;
				intakeUpToggleDir=true;
			}
			else{
				topStage.move_velocity(-100);
				bottomStage.move_velocity(-30);
				midStage.move_velocity(-100);
			}
		}
		else{
			topStage.move_velocity(0);
			bottomStage.move_velocity(0);
			midStage.move_velocity(0);
		}
		
		//PISTONS
		loader.set_value(loaderToggle);
		wing.set_value(wingToggle);
		intakeUpF.set_value(intakeUpToggleDir);
		intakeUpE.set_value(intakeUpTogglePow);

		scoreBlocker.set_value(scoreBlockerToggle);

		if (controller.get_digital_new_press(DIGITAL_UP)) 
			loaderToggle=!loaderToggle;
		if (controller.get_digital_new_press(DIGITAL_Y)) 
			wingToggle=!wingToggle;

		if (controller.get_digital_new_press(DIGITAL_X)) 
			scoreBlockerToggle=!scoreBlockerToggle;

		if (controller.get_digital_new_press(DIGITAL_RIGHT)){ 
			intakeUpTogglePow=!intakeUpTogglePow;
			intakeUpToggleDir=false;
		}

		if (controller.get_digital_new_press(DIGITAL_DOWN)){
			intakeUpTogglePow=!intakeUpTogglePow;
			intakeUpToggleDir=true;
		}

		
		
		//Loop delay for while loop
        pros::delay(5);
	}
}


//////////////////////////////////////////////////////////////////////////////////////
//Autonomous
/////////////////////////////////////////////////////////////////////////////////////
int dv=600;
void startOdom(){
	robot.start_odom();
}

//These are the basic functions that will be used for movement
//Adjusting max voltage effectively adjusts speed. Default is 12
void moveForward(double distance, double maxVoltage=12){
	robot.move_with_pid(distance, maxVoltage);
}
void moveBackwards(double distance, double maxVoltage=12){
	robot.move_with_pid(-distance, maxVoltage);
}
void turnRight(double degrees, double maxVoltage=12){
	robot.turn_with_pid((imu.get_rotation()+degrees), maxVoltage);
}
void turnLeft(double degrees, double maxVoltage=12){
	robot.turn_with_pid((imu.get_rotation()-degrees), maxVoltage);
}


void autonomous() {
	pros::Task printHeading([]{
		while (true) {
			//To print messages, connect to the terminal through the command "pros t"
			std::cout << imu.get_heading() << std::endl;
			pros::delay(500);
		}
	});
	// //MAKE SURE TO START ODOM OTHERWISE THE CODE BREAKSSSSS
	startOdom();
	moveForward(7);
}
