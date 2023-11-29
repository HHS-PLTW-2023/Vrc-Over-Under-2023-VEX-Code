/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Travis R. Gray                                            */
/*    Created:      11/15/2023, 9:46:29 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*    Motor ports:                                                            */
/*    Drivetrain - left side port 2, right side port 3                        */
/*    Lift arms - left side G port 9, right side G port 10, left side R port 19, right side R port 20  */
/*    Lift Arm Claw(s) - Port 8                                              */
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
vex::motor LiftArmMotorLF = motor(PORT9, ratio36_1, false); //left side green
vex::motor LiftArmMotorRF = motor(PORT10, ratio36_1, true); //right side green 
vex::motor LiftArmMotorLR = motor(PORT16, ratio36_1, false); //left side red
vex::motor LiftArmMotorRR = motor(PORT17, ratio36_1, true); // right side red
vex::motor_group LiftArmF = motor_group(LiftArmMotorLF, LiftArmMotorRF);
vex::motor_group LiftArmR = motor_group(LiftArmMotorLR, LiftArmMotorRR);
//lift arm claw motor
vex::motor Claw = motor(PORT8, ratio18_1, false);

//push flap motors
vex::motor PushArm = motor(PORT4, ratio18_1, false);

//variables 
//claw states
int clawOpen()
{
    Claw.spin(forward);
    wait(50,msec);
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
    LiftArmF.setVelocity(80, percent);
    LiftArmF.setStopping(hold);
    LiftArmF.setMaxTorque(100, percent);

    LiftArmR.setVelocity(80, percent);
    LiftArmR.setStopping(hold);
    LiftArmR.setMaxTorque(100, percent);

    //push flap motor settings
    PushArm.setVelocity(80, percent);
    PushArm.setMaxTorque(100, percent);
    PushArm.setStopping(hold);
    return 0;
} 

//lift motors forward
int liftspinFF()
{
    LiftArmF.spin(forward);
}
int liftspinRF()
{
    LiftArmR.spin(forward);
}
//lift motors reverse
int liftspinFR()
{
    LiftArmF.spin(reverse);
}
int liftspinRR()
{
    LiftArmR.spin(reverse);
}
int liftstopf()
{
    LiftArmF.stop();
}
int liftstopR()
{
    LiftArmR.stop();
}

//subsystem code
int subsystem()
{
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

    //try to make the lift arm motor sets into a function
    if (Controller1.ButtonR1.pressing()){
        liftspinFF and liftspinRF;
    }
    else if (Controller1.ButtonR2.pressing()){
        liftspinFR and liftspinRR;
    }  
    else if (not Controller1.ButtonR1.pressing() or Controller1.ButtonR2.pressing()){
        liftstopf and liftstopR; 
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