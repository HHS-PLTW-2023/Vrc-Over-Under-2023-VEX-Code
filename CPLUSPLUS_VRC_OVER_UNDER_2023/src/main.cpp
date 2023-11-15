/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       TRAVIS R. GRAY                                            */
/*    Created:      11/15/2023, 9:46:29 AM                                    */
/*    Description:  V5 project                                                */
/*    Motor ports:                                                            */
/*    Drivetrain - left side 2, right side 3                                  */
/*    Lift arms - left side 9, right side 10                                  */
/*    Push arms - port 4                                                      */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;

// define your global instances of motors and other devices here
// controller 
vex::controller Controller1 = controller(primary);

// drivetrain motors
vex::motor LeftDriveSmart = motor(PORT2, ratio18_1, false);
vex::motor RightDriveSmart = motor(PORT3, ratio18_1, true);
vex::drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);

// lift arm motor group
vex::motor LiftArmMotorA = motor(PORT9, ratio36_1, false); //left side
vex::motor LiftArmMotorB = motor(PORT10, ratio36_1, true); //right side
vex::motor_group LiftArm = motor_group(LiftArmMotorA, LiftArmMotorB);

//push flap motors
vex::motor PushArm = motor(PORT4, ratio18_1, false);

    int whenStarted1(){ 
        Brain.Screen.printAt( 10, 50, "Im in so much pain... please end me" );
   
        while(1) {
        //lift arm motor settings
        LiftArm.setVelocity(80, percent);
        LiftArm.setStopping(hold);
        LiftArm.setMaxTorque(100, percent);

        //push flap motor settings
        PushArm.setVelocity(80, percent);
        PushArm.setMaxTorque(100, percent);
        PushArm.setStopping(hold);

        //Push flap arm code
        //push l1 to open arms, pushh l2 to close arms, otherwise stop and hold position
        if (Controller1.ButtonL1.pressing()){
            PushArm.spin(forward);
        }
        else if (Controller1.ButtonL2.pressing()){
            PushArm.spin(reverse);
        }
        else if (not Controller1.ButtonL1.pressing() or Controller1.ButtonL2.pressing()){
            PushArm.stop();
        }

        //Lift Arm code
        //push R1 to raise arms, and push R2 to lower arms, otherwise stop and hold position
        if (Controller1.ButtonR1.pressing()){
            LiftArm.spin(forward);
        }
        else if (Controller1.ButtonR2.pressing()){
            LiftArm.spin(reverse);
        }
        else if (not Controller1.ButtonR1.pressing() or Controller1.ButtonR2.pressing()){
            LiftArm.stop();
        }

        //Claw code
        //use button 'A' to toggle the claw open and close via motor power

        // Allow other tasks to run
        this_thread::sleep_for(10);
        }
    }

    int main()
    {
        whenStarted1();
    }