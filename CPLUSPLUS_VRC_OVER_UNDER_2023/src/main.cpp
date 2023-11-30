/*-----------------------------------------------------------------------------------------------------*/
/*                                                                                                     */
/*    Module:       main.cpp                                                                           */
/*    Author:       Travis R. Gray                                                                     */
/*    Created:      11/15/2023, 9:46:29 AM                                                             */
/*    Description:  V5 project                                                                         */
/*                                                                                                     */
/*    Motor ports:                                                                                     */
/*    Drivetrain - left side port 2, right side port 3                                                 */
/*    Lift arms - left side G port 9, right side G port 10, left side R port 19, right side R port 20  */
/*    Lift Arm Claw(s) - Port 8                                                                        */
/*    Push arms - port 4                                                                               */
/*-----------------------------------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;

// define your global instances of motors and other devices here
//this section assigns motors to ports and assigns motor groups plus the control link for the controller
// controller 
vex::controller controller1 = controller(primary);
//

// assigning motors to port 2 and 3 for smart drivetrain system
vex::motor LeftDriveSmart = motor(PORT2, ratio18_1, false);
vex::motor RightDriveSmart = motor(PORT3, ratio18_1, true);
vex::drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);
//

//assigning motor(s) to ports 9, 10, 19, and 20 for two motor groups used to control the lift arm. 
// lift arm motor groups
//upper section (claw side) motor group
vex::motor upper_right = motor(PORT20, ratio36_1, false);
vex::motor upper_left = motor(PORT19, ratio36_1, true);
vex::motor_group upper = motor_group(upper_right, upper_left);

//lower section (base side) motor group
vex::motor lower_left = motor(PORT9, ratio36_1, true);
vex::motor lower_right = motor(PORT10, ratio36_1, false);
vex::motor_group lower = motor_group(lower_left, lower_right);
//

//assigning motor(s) to port 8 for the claw on top of the lift arm
//lift arm claw motor
vex::motor Claw = motor(PORT8, ratio18_1, false);
// 

// assigning motor(s) to port 4 for the push flaps on the front of the bot 
//push flap motors
vex::motor PushArm = motor(PORT4, ratio18_1, false);
// 

//motor setting tweaking, and function writing to call later in code
//this is where the majority of the code functions
//claw states and code

//Not yet implemented //
int clawOpen()
{
    Claw.spinFor(forward, 90, degrees);
    Claw.stop(hold);
    return 0;
}
int clawClosed()
{
    Claw.spin(reverse);
    wait(50,msec);
    Claw.stop(hold);
    return 0;
}
//Not yet implemented //

//settings for the upper, lower, and push arm motors
//motor settings
int motorSettings()
{
    //lift arm motor settings
    upper.setVelocity(80, percent);
    upper.setStopping(hold);
    upper.setMaxTorque(100, percent);

    lower.setVelocity(80, percent);
    lower.setStopping(hold);
    lower.setMaxTorque(100, percent);

    //push flap motor settings
    PushArm.setVelocity(80, percent);
    PushArm.setMaxTorque(100, percent);
    PushArm.setStopping(hold);
    return 0;
} 

//functions for the various subsystems' code
//subsystem code
int subsystem()
{
    //Push flap arm control code
    //push l1 to open arms, pushh l2 to close arms, otherwise stop and hold position
    if (controller1.ButtonL1.pressing()){
        PushArm.spin(forward);
    }
    else if (controller1.ButtonL2.pressing()){
        PushArm.spin(reverse);
    }
    else if (not controller1.ButtonL1.pressing() or controller1.ButtonL2.pressing()){
        PushArm.stop();
    }

    //Lift Arm control code
    //push r1 to raise, r2 to lower, otherwise stop and hold position
    if (controller1.ButtonR2.pressing()){
        upper.spin(forward); 
        lower.spin(forward);
    }
    else if (controller1.ButtonR1.pressing()){
        upper.spin(reverse);
        lower.spin(reverse);
    }
    else {
        upper.stop(hold);
        lower.stop(hold);
    }
    return 0;
}
    

    


// this is the when started function that runs in main when the thing starts
    int whenStarted1(){ 
        Brain.Screen.printAt( 10, 50, "Is this thing on?" );
        
        //allows for user control by making a continuous loop that will run forever (implement killswitch?)
        while(1) {

            //call motor settings
            motorSettings();

            //call subsystems
            subsystem();

            // Allow other tasks to run
            this_thread::sleep_for(10);
        }
    }

// main function that the brain runs to allow the inside code to run 
    int main()
    {
        whenStarted1();
    }