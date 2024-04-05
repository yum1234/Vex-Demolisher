#include "main.h"
#include <memory>
using namespace std;

//opcontrol declerations
void arcadeDrive(std::shared_ptr<ChassisController> drive, Controller &controller);
void flywheelHandling(ControllerButton &flywheelSpinIn, ControllerButton &flywheelSpinOut, Motor &flywheel);
void intakeHandling(ControllerButton &intakeIn, ControllerButton &intakeOut, Motor &intake);
void wingHandling(ControllerButton &wingsIn, ControllerButton &wingsOut, Motor &leftWing, Motor &rightWing);
void testAutoInitiate(ControllerButton &testAuto);

//autonomous declarations
void openWings(std::shared_ptr<AsyncPositionController<double, double>> leftWing, std::shared_ptr<AsyncPositionController<double, double>> rightWing);
void closeWings(std::shared_ptr<AsyncPositionController<double, double>> leftWing, std::shared_ptr<AsyncPositionController<double, double>> rightWing);
void waitWings(std::shared_ptr<AsyncPositionController<double, double>> leftWing, std::shared_ptr<AsyncPositionController<double, double>> rightWing);
void resetGPS(pros::Gps &gps, std::shared_ptr<OdomChassisController> drive);

struct {
	//chassis
	int topLeftMotor;
	int topRightMotor;
	int bottomLeftMotor;
	int bottomRightMotor;
	//general motor
	int intake;
	int flywheel;
	int leftWing;
	int rightWing;
	//sensors
	int gps;


} ports;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	//setting struct

	//general
	ports.intake = 5;
	ports.flywheel = 7;
	ports.leftWing = -3;
	ports.rightWing = 8;

	//chassis
	ports.topLeftMotor = -2;
	ports.topRightMotor = 9;
	ports.bottomRightMotor = 10;
	ports.bottomLeftMotor = -1;

	//sensors
	ports.gps = 11;

	//for lcd screen
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello Ryan and Nathan!");
	pros::lcd::register_btn1_cb(on_center_button);




}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	//Resources
		//https://www.skillsusa.org/wp-content/uploads/2023/11/2024-SkillsUSA-MRT-Game-Manual-S-PS.pdf
		//https://www.vexrobotics.com/over-under-manual
		//https://okapilib.github.io/OkapiLib/md_docs_tutorials_walkthrough_asyncAutonomousMovement.html
		//https://okapilib.github.io/OkapiLib/md_docs_tutorials_walkthrough_odometry.html

	//safety switch:
	//ControllerButton testAuto(ControllerDigital::X);

	//Chassis decleration
	std::shared_ptr<OdomChassisController> drive = ChassisControllerBuilder()
		.withMotors({ports.topLeftMotor, ports.bottomLeftMotor}, {ports.topRightMotor, ports.bottomRightMotor}) //https://www.vexforum.com/t/okapilib-turning-incorrect-angle/86794
		// Green gearset, 4 in wheel diam, 11.5 in wheel track
		.withDimensions(AbstractMotor::gearset::green*(60.0/36.0), {{4_in, 11.751_in}, imev5GreenTPR})
		.withOdometry(StateMode::CARTESIAN)
		.buildOdometry();

	//async motor declerations
		//intake
	std::shared_ptr<AsyncPositionController<double, double>> intake = AsyncPosControllerBuilder()
    	.withMotor(ports.intake) // lift motor port 3
    	.build();
		//flywheel
	std::shared_ptr<AsyncPositionController<double, double>> flywheel = AsyncPosControllerBuilder()
		.withMotor(ports.flywheel)
		.build();
		//right wing
	std::shared_ptr<AsyncPositionController<double, double>> rightWing = AsyncPosControllerBuilder()
		.withMotor(ports.rightWing)
		.build();
		//left wing
	std::shared_ptr<AsyncPositionController<double, double>> leftWing = AsyncPosControllerBuilder()
		.withMotor(ports.leftWing)
		.build();

	//sensors declerations
		//GPS
        double xOffset = -3; //Offset for the GPS from center of rotation
        double yOffset = 0;
		pros::GPS gps(ports.gps, xOffset, yOffset);
		pros::c::gps_status_s_t gpsData; 

	//main process

	//Turning right and moving a bit to allow the gps to reset odometry frame
	drive->turnToAngle(90_deg);
	drive->moveDistance(10_in);
	resetGPS(gps, drive);

	//Going to match load zone
	drive->driveToPoint({-1200_mm, 1200_mm});
	drive->turnToAngle(135_deg);
	drive->moveDistance(-12_in);

	//pushing balls in front
	openWings(leftWing, rightWing);
		//smashes into middle bar and hopefully straightens out
	drive->moveDistance(90_in);
	closeWings(leftWing, rightWing);
	resetGPS(gps, drive);

	//goes back to match load zone
	drive->moveDistance(-10_in);
	drive->driveToPoint({-1200_mm, 1200_mm});

	//Goes through top passage way
	drive->driveToPoint({-900_mm, 1500_mm});
	drive->turnToAngle(90_deg);
	resetGPS(gps, drive);
	drive->driveToPoint({900_mm, 1500_mm});
	resetGPS(gps, drive);

	//Going into other court
		//Goes to square below
	drive->driveToPoint({900_mm, 900_mm});
	resetGPS(gps, drive);
		//Goes inside court
	drive->driveToPoint({300_mm, 900_mm});
	resetGPS(gps, drive);
	drive->turnToAngle(180_deg);
		//drives down court to the middle
	openWings(leftWing, rightWing);
	drive->driveToPoint({300_mm, 0_mm});
	resetGPS(gps, drive);
		//Turns towards goal
	drive->turnToAngle(90_deg);
		//slams into goal
	drive->moveDistance(50_in);
	closeWings(leftWing, rightWing);
		//reverses
	drive->moveDistance(-50_in);
	resetGPS(gps, drive);
		//Goes down other half of court
	drive->turnToAngle(180_deg);
	drive->driveToPoint({300_mm, -900_mm});
	resetGPS(gps, drive);
		//turns toward goal
	drive->turnToAngle(90_deg);
	drive->moveDistance(50_in);
	

	

	


}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	//Controller declerations
	Controller controller;
		//for flywhel
	ControllerButton flywheelSpinIn(ControllerDigital::X);
	ControllerButton flywheelSpinOut(ControllerDigital::B);
		//for intake
	ControllerButton intakeIn(ControllerDigital::R1);
	ControllerButton intakeOut(ControllerDigital::R2);
		//for wings
	ControllerButton wingsIn(ControllerDigital::L1);
	ControllerButton wingsOut(ControllerDigital::L2);
		//for testing autonomous control
	ControllerButton testAuto(ControllerDigital::Y);


	//motor declerations
	Motor intake(ports.intake);
	Motor flywheel(ports.flywheel);
	Motor leftWing(ports.leftWing);
	Motor rightWing(ports.rightWing);

	//motor configuration declerations
	flywheel.setBrakeMode(okapi::AbstractMotor::brakeMode::coast); //https://www.vexforum.com/t/strange-okapi-xdrivemodel-setbrakemode-behavior/95937
	intake.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	leftWing.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	rightWing.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	//Chassis decleration
	std::shared_ptr<ChassisController> drive = ChassisControllerBuilder()
		.withMotors({ports.topLeftMotor, ports.bottomLeftMotor}, {ports.topRightMotor, ports.bottomRightMotor}) //https://www.vexforum.com/t/okapilib-turning-incorrect-angle/86794
		// Green gearset, 4 in wheel diam, 11.5 in wheel track
		.withDimensions(AbstractMotor::gearset::green*(60.0/36.0), {{4_in, 11.751_in}, imev5GreenTPR})
		.build();

	//main loop
	while (true) {
		arcadeDrive(drive, controller);
		flywheelHandling(flywheelSpinIn, flywheelSpinOut, flywheel); 
		intakeHandling(intakeIn, intakeOut, intake);
		wingHandling(wingsIn, wingsOut, leftWing, rightWing);

		testAutoInitiate(testAuto);

		//give scheduler time to do other tasks
		pros::delay(10);
	}

	


}



//functions for op control
void arcadeDrive(std::shared_ptr<ChassisController> drive, Controller &controller) {
	//handles driving
	
	drive->getModel()->driveVector(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightX)); //uses velocity mode but still off
	
	//below is deprecated code
	//double pidConstant; //This is because the bot has slightly more power on the right rather then the left. So we are going to decrease the right.
	
		
	
	//double leftAnalog = controller.getAnalog(ControllerAnalog::leftY); //gets control inputs at particular input
	//double rightAnalog = controller.getAnalog(ControllerAnalog::rightX);

	/*
	if(leftAnalog > 0.02 || rightAnalog > 0.02) { //.02 instead of 0 to account for the deadzone on the sticks of the controller.
		pidConstant = 0.1; //This is nesscary else the bot will be always moving despite no input. Only when positive power is applied the bot should counteract that power.
	} else {
		pidConstant = 0;
	}
	*/


	//drive->getModel()->left(leftAnalog + rightAnalog); 
	//drive->getModel()->right(leftAnalog - rightAnalog - pidConstant);
}

void flywheelHandling(ControllerButton &flywheelSpinIn, ControllerButton &flywheelSpinOut, Motor &flywheel) {
	//handles buttons and motors for the flywheel
	if(flywheelSpinIn.isPressed()) {
		flywheel.moveVelocity(200);
	}

	else if(flywheelSpinOut.isPressed()) {
		flywheel.moveVelocity(-200);
	}

	else {
		flywheel.moveVelocity(0);
	}

}

void intakeHandling(ControllerButton &intakeIn, ControllerButton &intakeOut, Motor &intake) {
	//handles buttons and motors for the intake
	if(intakeIn.isPressed()) {
		intake.moveVelocity(200);
	}

	else if(intakeOut.isPressed()) {
		intake.moveVelocity(-200);
	}

	else {
		intake.moveVelocity(0);
	}
}

void wingHandling(ControllerButton &wingsIn, ControllerButton &wingsOut, Motor &leftWing, Motor &rightWing) {
	//handles buttons and motors for the wings.
	if(wingsIn.isPressed()) {
		leftWing.moveVelocity(200);
		rightWing.moveVelocity(200);
	}

	else if(wingsOut.isPressed()) {
		leftWing.moveVelocity(-200);
		rightWing.moveVelocity(-200);
	}
	
	else {
		leftWing.moveVelocity(0);
		rightWing.moveVelocity(0);
	}

}

void testAutoInitiate(ControllerButton &testAuto) {
	if(testAuto.isPressed()) {
		autonomous();
	}
}

//functions for autonomous
void openWings(std::shared_ptr<AsyncPositionController<double, double>> leftWing, std::shared_ptr<AsyncPositionController<double, double>> rightWing) {
	//opens both wings
	leftWing->setTarget(90);
	rightWing->setTarget(90);
}

void closeWings(std::shared_ptr<AsyncPositionController<double, double>> leftWing, std::shared_ptr<AsyncPositionController<double, double>> rightWing) {
	//closes both wings
	leftWing->setTarget(-90);
	rightWing->setTarget(-90);
}

void waitWings(std::shared_ptr<AsyncPositionController<double, double>> leftWing, std::shared_ptr<AsyncPositionController<double, double>> rightWing) {
	//waits for the wings to finish their targets
	leftWing->waitUntilSettled();
	rightWing->waitUntilSettled();
}

void moveCood(pros::Gps &gps, std::shared_ptr<OdomChassisController> drive, double targetX, double targetY, double targetAngle) {
	//this function calculates the difference between the current gps point, and the target gps point (place you want to go on the vex coodinate system).
	//Then it goes to use that difference and add it to the odometry, and sets that as the point to move towards.
	//Inches only!!!
	//https://www.vexforum.com/t/using-okapi-units/83933 convert double to qlength or some other units.
	//NOTE!!! THIS FUNCTION IS DEPRECATED
	pros::c::gps_status_s_t gpsData; //define a strut
	gpsData = gps.get_status();

	QLength diffX = (targetX-gpsData.x)*1.0_m; //convert meters to inches
	QLength diffY = -(targetY-gpsData.y)*1.0_m; //negative is reqiured due to the different coodinate systems
	QAngle diffAngle = (90.0+(targetAngle-gps.get_heading()))*1_deg; //the +90 degrees is reqiured to translate angle systems
	
	QLength relativeX = drive->getState().x;
	QLength relativeY = drive->getState().y;
	QAngle relativeAngle = drive->getState().theta;

	drive->driveToPoint({relativeX + diffX, relativeY + diffY});
	drive->turnToAngle(relativeAngle + diffAngle);

}

void resetGPS(pros::Gps &gps, std::shared_ptr<OdomChassisController> drive) {
	//This function checks the current position with the gps, and sets that as the Odom state.
	//https://pros.cs.purdue.edu/v5/tutorials/topical/gps.html gps guide
	//https://www.vexforum.com/t/using-okapi-units/83933 convert double to qlength or some other units.
	pros::delay(2000); //Has to stop for one second to get accurate reading.
	pros::c::gps_status_s_t gpsData; //define a strut
	gpsData = gps.get_status();

	
	drive->setState({gpsData.x*1_m, gpsData.y*1_m, gps.get_heading()*1_deg}); //https://okapilib.github.io/OkapiLib/md_docs_tutorials_walkthrough_odometry.html


}
