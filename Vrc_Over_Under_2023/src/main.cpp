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
#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain Brain;

// define your global instances of motors and other devices here
// this section assigns motors to ports and assigns motor groups plus the control link for the controller
// controller
vex::controller controller1 = controller(primary);
//

// assigning motors to port 2 and 3 for smart drivetrain system
vex::motor leftd = motor(PORT2, ratio18_1, true);
vex::motor rightd = motor(PORT3, ratio18_1, false);

//

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

// functions for the various subsystems' code
// subsystem code
int subsystem()
{
    // Push flap arm control code
    // push l1 to open arms, pushh l2 to close arms, otherwise stop and hold position
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
    Brain.Screen.printAt(10, 50, "did i download properly?");

    // allows for user control by making a continuous loop that will run forever (implement killswitch?)
    while (true)
    {
        // call motor settings
        motorSettings();
        // call subsystems
        subsystem();

        // Allow other tasks to run
    }
}

// main function that the brain runs to allow the inside code to run
int main()
{
    whenStarted1();
}