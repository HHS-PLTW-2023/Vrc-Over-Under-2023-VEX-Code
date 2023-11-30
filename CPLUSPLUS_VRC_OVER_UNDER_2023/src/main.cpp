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
// controller 
vex::controller controller1 = controller(primary);

// drivetrain motors
vex::motor LeftDriveSmart = motor(PORT2, ratio18_1, false);
vex::motor RightDriveSmart = motor(PORT3, ratio18_1, true);
vex::drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);

// lift arm motor groups
vex::motor one = motor(PORT20, ratio36_1, false);
vex::motor two = motor(PORT19, ratio36_1, true);
vex::motor_group upper = motor_group(one, two);

vex::motor three = motor(PORT9, ratio36_1, true);
vex::motor four = motor(PORT10, ratio36_1, false);
vex::motor_group lower = motor_group(three, four);

//lift arm claw motor
vex::motor Claw = motor(PORT8, ratio18_1, false);

//push flap motors
vex::motor PushArm = motor(PORT4, ratio18_1, false);

//variables 
//claw states and code

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

//subsystem code
int subsystem()
{
    //Push flap arm code
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

    //Lift Arm code
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
    

    



    int whenStarted1(){ 
        Brain.Screen.printAt( 10, 50, "Is this thing on?" );
   
        while(1) {
            //call motor settings
            motorSettings();

            //call subsystems
            subsystem();

       



            // Allow other tasks to run
            this_thread::sleep_for(10);
        }
    }

    int main()
    {
        whenStarted1();
    }