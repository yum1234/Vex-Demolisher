#include "../include/main.h"
#include <memory>
using namespace std;

void arcade(std::shared_ptr<ChassisController> drive, Controller &controller);
void flywheelHandling(ControllerButton &flywheelSpinIn, ControllerButton &flywheelSpinOut, Motor &flywheel);

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
void autonomous() {}

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
	ControllerButton flywheelSpinIn(ControllerDigital::R1);
	ControllerButton flywheelSpinOut(ControllerDigital::R2);

	//motor declerations
	Motor intake(5);
	Motor flywheel(6);
	Motor leftwing(7);
	Motor rightwing(8);

	//motor configuration declerations
	flywheel.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	leftwing.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	rightwing.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	//Chassis decleration
	std::shared_ptr<ChassisController> drive = ChassisControllerBuilder()
			.withMotors(1, 2, 3, 4)
			// Green gearset, 4 in wheel diam, 11.5 in wheel track
			.withDimensions(AbstractMotor::gearset::green*(60.0/36.0), {{4_in, 11.751_in}, imev5GreenTPR})
			.build();

	while (true) {
		arcade(drive, controller);
		flywheelHandling(flywheelSpinIn, flywheelSpinOut, flywheel); 


		pros::delay(10);
	}

	


}


void arcade(std::shared_ptr<ChassisController> drive, Controller &controller) {
	//handles driving
	drive->getModel()->arcade(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::leftX));

}

void flywheelHandling(ControllerButton &flywheelSpinIn, ControllerButton &flywheelSpinOut, Motor &flywheel) {
	//handles buttons and motors for the flywheel
	if(flywheelSpinIn.isPressed()) {
		flywheel.moveVelocity(1400);
	}

	else if(flywheelSpinOut.isPressed()) {
		flywheel.moveVelocity(-1400);
	}

	else {
		flywheel.moveVelocity(0);
	}

}
