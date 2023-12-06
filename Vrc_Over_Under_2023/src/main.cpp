/*-----------------------------------------------------------------------------------------------------*/
/*                                                                                                     */
/*    Module:       main.cpp                                                                           */
/*    Author:       Travis R. Gray                                                                     */
/*    Created:      11/15/2023, 9:46:29 AM                                                             */
/*    Description:                                                                                     */
/*                                                                                                     */
/*    Motor ports:                                                                                     */
/*    Drivetrain - Left side port 2, Right side port 3                                                 */
/*    Lift arms - Lower left port 9, lower right port 10, upper left port 19, upper right port 20      */
/*    Lift Arm Claw(s) - Port 8                                                                        */
/*    Push arms - Port 4                                                                               */
/*-----------------------------------------------------------------------------------------------------*/
//Drivetrain: l2, r3; LiftArm: ll9, lr10, ul19, ur20; Claw: 8; PushArm: 4
#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain Brain;

// define your global instances of motors and other devices here
// this section assigns motors to ports and assigns motor groups plus the control link for the controller
// controller
vex::controller controller1 = controller(primary);

// assigning motors to port 2 and 3 for smart drivetrain system
vex::motor leftd = motor(PORT16, ratio18_1, false);
vex::motor rightd = motor(PORT17, ratio18_1, true);

// assigning motor(s) to ports 9, 10, 19, and 20 for two motor groups used to control the lift arm.
//  lift arm motor groups
// upper section (claw side) motor group
vex::motor upper_right = motor(PORT20, ratio36_1, false);
vex::motor upper_left = motor(PORT19, ratio36_1, true);

// lower section (base side) motor group
vex::motor lower_left = motor(PORT9, ratio36_1, true);
vex::motor lower_right = motor(PORT10, ratio36_1, false);
vex::motor_group liftarm = motor_group(lower_left, lower_right, upper_left, upper_right);
//

// assigning motor(s) to port 8 for the claw on top of the lift arm
// lift arm claw motor
vex::motor Claw = motor(PORT8, ratio18_1, true);
//

// assigning motor(s) to port 4 for the push flaps on the front of the bot
// push flap motors
vex::motor PushArm = motor(PORT4, ratio18_1, false);
//

// motor setting tweaking, and function writing to call later in code
// this is where the majority of the code functions

// settings for the upper, lower, and push arm motors
// motor settings
int motorSettings()
{
    //drivetrain
    leftd.setMaxTorque(100, pct);
    leftd.setVelocity(50, pct);
    leftd.setBrake(hold);

    rightd.setMaxTorque(100, pct);
    rightd.setVelocity(50, pct);
    rightd.setBrake(hold);

    // lift arm motor settings
    liftarm.setVelocity(80, percent);
    liftarm.setStopping(hold);
    liftarm.setMaxTorque(100, percent);

    // push flap motor settings
    PushArm.setVelocity(80, percent);
    PushArm.setMaxTorque(100, percent);
    PushArm.setStopping(hold);

    // claw motor settings
    Claw.setVelocity(80, percent);
    Claw.setMaxTorque(100, percent);
    return 0;
}

//functions for autonomous period
const float pi = 3.14159;
const float wheel_diameter = 4;
const float wheel_circumference = pi * wheel_diameter;
const float gear_ratio = 0.5;
const float wheel_base_width = 19; //radius for finding arc length 
const float wheel_base_diameter = 26.870057685;
const int auton_drive_pct = 25;

void drive_forward (float inches ) 
{
    float inch_per_degree = wheel_circumference/360;
    float degrees = inches/inch_per_degree;

    leftd.spinFor(degrees*gear_ratio, vex::rotationUnits::deg, auton_drive_pct, vex::velocityUnits::pct, false);
    rightd.spinFor(degrees*gear_ratio, vex::rotationUnits::deg, auton_drive_pct, vex::velocityUnits::pct, false);
    
    wait(1, sec);
}

void turn_left (float degrees)
{
    rightd.setReversed(true);

    const float wheel_travel_length = ((degrees/360)*2*(3.1415)*(wheel_base_width));
    const float motor_spin_degrees = (((wheel_travel_length/wheel_circumference)*360)/2);

    leftd.spinFor(motor_spin_degrees, vex::rotationUnits::deg, auton_drive_pct, vex::velocityUnits::pct, false);
    rightd.spinFor(motor_spin_degrees, vex::rotationUnits::deg, auton_drive_pct, vex::velocityUnits::pct, false);


    wait(1, sec);

}

void turn_right (float degrees)
{
    rightd.setReversed(true);
    //Arc Length = (θ/360) x 2πr
    const float wheel_travel_length = ((degrees/360)*2*(3.1415)*(wheel_base_width));
    const float motor_spin_degrees = (((wheel_travel_length/wheel_circumference)*360)/2);

    leftd.spinFor(motor_spin_degrees, vex::rotationUnits::deg, auton_drive_pct, vex::velocityUnits::pct, false);
    rightd.spinFor(motor_spin_degrees, vex::rotationUnits::deg, auton_drive_pct, vex::velocityUnits::pct, false);
    

    wait(1, sec);

}


void autonomous ()
{
    drive_forward(0.5*12);
    drive_forward(-0.5*12);
    turn_left(90);
    turn_right(90);
}

// functions for the various User_Controls' code
// User_Control code
int User_Control()
{
    // Push flap arm control code
    // push l1 to open arms, push l2 to close arms, otherwise stop and hold position
    if (controller1.ButtonL1.pressing())
    {
        PushArm.spin(forward);
    }
    else if (controller1.ButtonL2.pressing())
    {
        PushArm.spin(reverse);
    }
    else 
    {
        PushArm.stop(hold);
    }

    // Lift Arm control code
    // push r1 to raise, r2 to lower, otherwise stop and hold position
    if (controller1.ButtonR2.pressing())
    {
        liftarm.spin(forward);
    }
    else if (controller1.ButtonR1.pressing())
    {
        liftarm.spin(reverse);
    }
    else
    {
        liftarm.stop(hold);
    }

    // claw controle code
    if (controller1.ButtonX.pressing())
    {
        Claw.setPosition(0, degrees);
        Claw.spinToPosition(275, degrees, false);
    }
    if (controller1.ButtonB.pressing())
    {
        Claw.setPosition(0, degrees);
        Claw.spinToPosition(-275, degrees, false);
    }

    // Drivetrain Code
    leftd.spin(fwd, controller1.Axis3.value(), pct);
    rightd.spin(fwd, controller1.Axis2.value(), pct);
    return 0;
}

// this is the when started function that runs in main when the thing starts
int whenStarted1()
{
    
    Brain.Screen.print("test 10");
    
    // call motor settings
    motorSettings();

    //call autonomous
    autonomous();

    leftd.setReversed(false);
    rightd.setReversed(true);

    // allows for user control by making a continuous loop that will run forever (implement killswitch?)
    while (true)
    {
    

        // call User_Controls
        User_Control();

        // Allow other tasks to run
    }
}

// main function that the brain runs to allow the inside code to run
int main()
{
    whenStarted1();
}