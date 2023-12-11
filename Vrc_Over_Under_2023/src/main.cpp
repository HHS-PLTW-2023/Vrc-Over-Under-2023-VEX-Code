/*-----------------------------------------------------------------------------------------------------*/
/*                                                                                                     */
/*    Module:       main.cpp                                                                           */
/*    Author:       Travis R. Gray                                                                     */
/*    Created:      11/15/2023, 9:46:29 AM                                                             */
/*    Description:                                                                                     */
/*                                                                                                     */
/*       https://api.vexcode.cloud/v5/                                                                 */
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

// A global instance of vex::brain used for prdoubleing to the V5 brain screen
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

vex::encoder rightEnc = encoder(Brain.ThreeWirePort.A);
vex::encoder leftEnc = encoder(Brain.ThreeWirePort.G);
//

// motor setting tweaking, and function writing to call later in code
// this is where the majority of the code functions

// settings for the upper, lower, and push arm motors
// motor settings
double motorSettings()
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

const double pi = 3.14159;
const double wheel_diameter = 4.135;
const double wheel_circumference = pi * wheel_diameter;
double inch_per_degree = wheel_circumference/360;

//Start of PID algorithm
//settings
double kP = 0.3;
double kI = 0.2;
double kD = 0.1;
double turnkP = 0.3;
double turnkI = 0.2;
double turnkD = 0.1;

//autonomous settings
double desiredValue = 200;
double desiredTurnValue = 0;

double error; //sensorValue - desiredValue : position
double prevError = 0; // position 20 msecs ago
double derivitive; // error - prevError : speed
double totalError = 0; // totalError = totalError + error

double turnerror; //sensorValue - desiredValue : position
double turnprevError = 0; // position 20 msecs ago
double turnderivitive; // error - prevError : speed
double turntotalError = 0; // totalError = totalError + error

bool resetEncoders = false;

double whlPositionAverage = 0.0;



//variables modified for use 
bool enableDrivePID = true;

int drivePID()
{
    while (enableDrivePID)
    {

        if (resetEncoders)
        {
            resetEncoders = false;
            leftd.setPosition(0, deg);
            rightd.setPosition(0, deg);
        }


        //Get position of both encoders
        double leftWhlPosition = leftEnc.position(deg);
        double rightWhlPosition = rightEnc.position(deg);

        ///////////////////////////////////
        // Lateral Movement PID
        ///////////////////////////////////

        //average of the two motors
        double whlPositionAverage = (leftWhlPosition + rightWhlPosition) / 2;

        //converting desired value (inches), into degrees needed to turn 
        double desiredDegrees = (desiredValue/inch_per_degree);

        //potential
        error = whlPositionAverage - desiredDegrees;

        //derivitive 
        derivitive = error - prevError;

        //integral (not best for drivetrain)
        //totalError += error;

        double lateralMotorPower = (error * kP + derivitive * kD) / 12; 
        // + totalError * kI (for integral, bad for drivetrain but makes the small changes,
        // helps steady state errors)

        ///////////////////////////////////
        // Turning Movement PID
        ///////////////////////////////////

        //average of the two motors
        double turnDifference = leftWhlPosition - rightWhlPosition;

        //potential
        turnerror = turnDifference - desiredTurnValue;

        //derivitive 
        turnderivitive = turnerror - turnprevError;

        //integral
        //turntotalError += turnerror;

        double turnMotorPower = (turnerror * turnkP + turnderivitive * turnkD) / 12;


        ///////////////////////////////////
        leftd.spin(fwd, lateralMotorPower + turnMotorPower, voltageUnits::volt);
        rightd.spin(fwd, lateralMotorPower - turnMotorPower, voltageUnits::volt);
    

        prevError = error;
        turnprevError = turnerror;

        vex::task::sleep(20);

    }
    return 0;
}



void autonomous ()
{
    vex::task PID(drivePID);

    Brain.Screen.print(whlPositionAverage);

    resetEncoders = true;
    desiredValue = 3;
    desiredTurnValue = 6;

    Brain.Screen.print("----");
    Brain.Screen.print(whlPositionAverage);
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
    
    Brain.Screen.print("test c");
    Brain.Screen.print("-----");
    
    // call motor settings
    motorSettings();

    //call autonomous
    autonomous();


    // allows for user control by making a continuous loop that will run forever (implement killswitch?)
    while (true)
    {
    

        //call User_Controls
        User_Control();

        //Allow other tasks to run
        vex::wait(20, msec);
    }

    return 0;
}

// main function that the brain runs to allow the inside code to run
int main()
{
    whenStarted1();
}