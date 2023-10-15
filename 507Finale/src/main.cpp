/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Woojin Lee                                                */
/*    Created:      Thu Aug 28 2023                                           */
/*    Description:  507A 2023-2024 Competition Template                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/
// ---- START VEXCODE CONFIGURED DEVICES ----
#include "vex.h"
#include <cmath>
#include <iostream>
#include <vector>
using namespace vex;
using namespace std;
// A global instance of competition
competition Competition;
// define your global instances of motors and other devices here
brain Brain;
controller Controller1 = controller(primary);
// drivetrain motors
motor backLeft(PORT12, gearSetting::ratio6_1, true);
motor backRight(PORT18, gearSetting::ratio6_1, false);
motor middleLeft(PORT13, gearSetting::ratio6_1, false);
motor middleRight(PORT19, gearSetting::ratio6_1, true);
motor frontLeft(PORT15, gearSetting::ratio6_1, true);
motor frontRight(PORT20, gearSetting::ratio6_1, false);
// intake motor
motor intake(PORT1, gearSetting::ratio6_1, false);
// flywheel motor
motor cata(PORT10, gearSetting::ratio6_1, false);
// motor groupings
motor_group leftSide(backLeft, middleLeft, frontLeft);
motor_group rightSide(backRight, middleRight, frontRight);
drivetrain driveTrain(leftSide, rightSide);
// position sensors
inertial Inertial(PORT16);
rotation par(PORT5, true);
rotation perp(PORT6, true);
rotation rot(PORT11);
// pneumatics
pneumatics fourBar(Brain.ThreeWirePort.G);
pneumatics wing1(Brain.ThreeWirePort.H);
pneumatics wing2(Brain.ThreeWirePort.F);
pneumatics climb(Brain.ThreeWirePort.D);

// ---- END VEXCODE CONFIGURED DEVICES ----
// useful stats printed to Brain screen
void autonBrain(double driveError) {
  Brain.Screen.printAt(20, 20, "DE: %f", driveError);
  Brain.Screen.printAt(20, 40, "C: %f", cata.velocity(rpm));
  Brain.Screen.printAt(200, 60, "I: %f", intake.velocity(rpm));
  Brain.Screen.printAt(20, 80, "FL: %f", frontLeft.velocity(rpm));
  Brain.Screen.printAt(20, 100, "BL: %f", backLeft.velocity(rpm));
  Brain.Screen.printAt(20, 120, "FR: %f", frontRight.velocity(rpm));
  Brain.Screen.printAt(20, 140, "BR: %f", backRight.velocity(rpm));
  Brain.Screen.printAt(200, 160, "ML: %f", middleLeft.velocity(rpm));
  Brain.Screen.printAt(200, 180, "MR: %f", middleRight.velocity(rpm));
}
void driveBrain() {
  Brain.Screen.printAt(20, 20, "C: %f %f", cata.velocity(rpm),
                       cata.temperature());
  Brain.Screen.printAt(20, 40, "I: %f %f", intake.velocity(rpm),
                       intake.temperature());
  Brain.Screen.printAt(20, 60, "FL: %f %f", frontLeft.velocity(rpm),
                       frontLeft.temperature());
  Brain.Screen.printAt(20, 80, "BL: %f %f", backLeft.velocity(rpm),
                       backLeft.temperature());
  Brain.Screen.printAt(20, 100, "FR: %f %f", frontRight.velocity(rpm),
                       frontRight.temperature());
  Brain.Screen.printAt(20, 120, "BR: %f %f", backRight.velocity(rpm),
                       backRight.temperature());
  Brain.Screen.printAt(20, 140, "ML: %f %f", middleLeft.velocity(rpm),
                       middleLeft.temperature());
  Brain.Screen.printAt(20, 160, "MR: %f %f", middleRight.velocity(rpm),
                       middleRight.temperature());
}

// All activities that occur before the competition starts
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // calibrates inertial sensor
  Inertial.calibrate();
  // reset motor positions
  par.resetPosition();
  perp.resetPosition();
  leftSide.resetPosition();
  rightSide.resetPosition();
  intake.resetPosition();
  cata.resetPosition();
  intake.resetRotation();
  leftSide.resetRotation();
  rightSide.resetRotation();
  rot.resetPosition();
  wait(2, sec);
}
// boolean to keep all controllers within auton
bool togglePID = 1;
// since PID is in a task, vectors were used to keep desired values in order and
// update at correct moments; time not used due to the inaccuracy of it
int i = 0;
vector<double> driving{0};
vector<double> turning{0};
vector<int> intakes{0};
vector<int> cataMode{0};
vector<bool> wing{0};
vector<bool> endgame{0};
vector<int> waiter{0};
vector<bool> ignoreT{0};
vector<double> maxV{12};
vector<int> oneSides{0};
vector<int> timing{2500};
vector<int> branch{0};
double turnAmount = 0;
int intakeSetting = 0;
int exitTime = 2500;
double voltageCap = 12;
bool ignoreTurn = 0;
bool activateWings = 0;
bool activateEndgame = 0;
int cataStage = 0;
// method to feed values into PID
void drivePID(double driveInches, int waitTime = 0,
              bool ignoreTurning = ignoreTurn, double maxVolts = voltageCap,
              int oneSide = 0, int timeThreshold = exitTime,
              int branchAmount = 0, double turnDegrees = turnAmount,
              int intake = intakeSetting, bool wings = activateWings,
              bool end = activateEndgame, int cata = cataStage) {
  // convert desired drive inches to degrees
  driveInches *= (360 / (2.75 * M_PI));
  if (maxVolts == 0) {
    maxVolts = 12;
  }
  // push parameters into respective vectors
  driving.push_back(driveInches);
  turning.push_back(turnDegrees);
  intakes.push_back(intake);
  cataMode.push_back(cata);
  wing.push_back(wings);
  endgame.push_back(end);
  waiter.push_back(waitTime);
  oneSides.push_back(oneSide);
  ignoreT.push_back(ignoreTurning);
  maxV.push_back(maxVolts);
  branch.push_back(branchAmount);
  timing.push_back(timeThreshold);
}
// int cataPI(){
//   double kP=0.01;
//   double kI=0.001;
//   double temp;
//   double cataFinalDistance;
//   double cataTotalError=0;
//   switch (cataMode.at(i)){
//     case 0:
//       //cata position 1
//       cataFinalDistance=10;
//       temp=cataFinalDistance;
//       break;
//     case 1:
//       cataFinalDistance=50;
//       temp=cataFinalDistance;
//       break;
//     case 2:
//       cataFinalDistance=0;
//       temp=cataFinalDistance;
//       break;
//     default:
//       Controller1.rumble(".-");
//       break;
//   } 
//   while(togglePID){
//     double cataPosition=dist.objectDistance(distanceUnits::cm);
//     if (cataPosition<cataFinalDistance){
//       cataFinalDistance=0;
//     }
//     else{
//       cataFinalDistance=temp;
//     }
  
//     //proportional
//     double cataError=cataFinalDistance-cataPosition;

//     //integral
//     if(fabs(cataError)<8){
//       cataTotalError+=cataError;
//     }
//     else{
//       cataTotalError=0;
//     }

//     //sets up the motor power for each side
//     double cataMotorPower=cataError*kP+cataTotalError*kI;
//     if(cataMotorPower>0){
//       cataMotorPower=0;
//     }
//     cata.spin(fwd,-cataMotorPower,volt);
    
//     //this helps you see any undershooting or overshooting
//     Brain.Screen.printAt(20,200,"%f",cataError);

//     //Sleep the PD for a short amount of time to prevent wasted resources.
//     wait(10,msec);
//   }
//   return 1;
// }

bool checkDriveErrors(double driveError, double driveMargin,
                      double driveFinalValue) {
  if (fabs(driveError) <= driveMargin || driveFinalValue == 0) {
    return true;
  }
  return false;
}
bool checkTurnErrors(double turnError, double turnMargin) {
  if (ignoreT.at(i) == 1) {
    return true;
  } else if (fabs(turnError) < turnMargin || ignoreT.at(i)) {
    return true;
  }
  return false;
}
void modDrivePower(double* driveMotorPower, double voltageCap,
                   double driveFinalValue) {
  // caps the max voltage of PID to allow for slower movements if needed
  if (*driveMotorPower >= voltageCap) {
    *driveMotorPower = voltageCap;
  } else if (*driveMotorPower <= -voltageCap) {
    *driveMotorPower = -voltageCap;
  }
  // keep drive still if it isn't supposed to move
  if (driveFinalValue == 0) {
    *driveMotorPower = 0;
  }
}
void powerSides(double *driveError, double driveMotorPower,
                double turnMotorPower) {
  // disable right side and arc by only powering left side
  if (oneSides.at(i) == 1) {
    *driveError = 0;
    leftSide.spin(fwd, turnMotorPower, volt);
    rightSide.stop(brake);
  }
  // disable left side and arc by only powering right side
  else if (oneSides.at(i) == 2) {
   *driveError = 0;
    rightSide.spin(fwd, -turnMotorPower, volt);
    leftSide.stop(brake);
  }
  // if both are false, move normally and give power to both sides
  else {
    rightSide.spin(fwd, driveMotorPower - turnMotorPower, volt);
    leftSide.spin(fwd, driveMotorPower + turnMotorPower, volt);
  }
}
void intakeMode() {
  // condition to spin intake forward to pick up discs and for rollers
  switch (intakes.at(i)){
    case 1:
      intake.spin(fwd, 12, volt);
      break;
    case 2:
      intake.spin(reverse, 12, volt);
      break;
    default:
      intake.stop();
      break;
  }
}
void branching(timer branchTimer, double branchThreshold) {
  // branch threshold is min time required to branch out
  // if we want to jump and there is enough time, jump amount we want to
  // 1 is signal
  //-1 is receiver
  // skip
  if (branch.at(i) == 1 && branchTimer.time(sec) <= branchThreshold) {
    for (int j = i; j < branch.size(); j++) {
      if (branch.at(j) == -1) {
        i = j;
        break;
      } else if (j == branch.size() - 1) {
        i++;
        break;
      }
    }
  }
  // otherwise, just continue normally
  else {
    i++;
  }
}
// if next value in drive and turn vectors exist, continue
void updatePIDConditions(double* driveFinalValue, double* turnFinalValue,
                         double* voltageCap, timer branchTimer) {
  // if next value in drive and turn vectors exist, continue
  if (i < driving.size()-1) {
    // wait specified amount of time at end
    wait(waiter.at(i), msec);
    // parameters to start/not branch
    //updates i
    branching(branchTimer, 54.5);

    *driveFinalValue = driving.at(i);
    *turnFinalValue = turning.at(i);
    // change intake power/direction
    intakeMode();
    // update voltage Cap
    *voltageCap = maxV.at(i);
    //wings
    wing1.set(wing.at(i));
    wing2.set(wing.at(i));
    climb.set(endgame.at(i));
  }
  // if there are no more commands, keep robot still
  else {
    driveFinalValue = 0;
  }
}
// PID put into task for constan updating and greater accuracy
int pid() {
  par.resetPosition();
  // drive and turn constants for PID
  double drivekP = .0152;
  double drivekD = .00023;
  double drivekI = 0.00015;
  double turnkP = .095;
  double turnkD = 0.003;
  double turnkI = 0.012;
  // initial drive and turn derivatives and integrals
  double driveFinalValue = driving.at(i);
  double drivePreviousError = 0;
  double totalError = 0;
  double turnFinalValue = turning.at(i);
  double turnPreviousError = 0;
  double totalTurnError = 0;
  // margins of error
  double driveMargin = 10;
  double turnMargin = 1;
  // default voltage cap on PID
  double voltageCap = maxV.at(i);
  timer t;
  timer branchTimer;
  // task CataPI(cataPI);
  while (togglePID) {
    // for longer distances, allow a larger margin
    if ((driveFinalValue * (2.75 * M_PI) / 360) > 80) {
      driveMargin = 20;
    }
    // store current rotation and inertial values
    double drivePosition = par.position(deg);
    double turnPosition = Inertial.rotation();
    // proportional
    double driveError = driveFinalValue - drivePosition;
    double turnError = turnFinalValue - turnPosition;

    // derivative
    double driveDerivative = driveError - drivePreviousError;
    double turnDerivative = turnError - turnPreviousError;

    // integral
    // drive bound=100
    // turn bound = 20
    // bounding integral to certain scope prevents integral wind-up
    if (fabs(driveError) <= 100) {
      totalError += driveError;
    } else {
      totalError = 0;
    }
    if (fabs(turnError) <= 20 && abs(turnError) > turnMargin) {
      totalTurnError += turnError;
    } else {
      totalTurnError = 0;
    }
    // update previous values for next iteration
    drivePreviousError = driveError;
    turnPreviousError = turnError;
    // sets up the linear motor power
    double driveMotorPower =
        driveError * drivekP + driveDerivative * drivekD + totalError * drivekI;
    // sets up turn motor power
    double turnMotorPower =
        turnError * turnkP + turnDerivative * turnkD + totalTurnError * turnkI;
    // cap or remove drive voltage
    modDrivePower(&driveMotorPower, voltageCap, driveFinalValue);
    // determine which sides to power, 1 is only left, 2 is only right
    powerSides(&driveError, driveMotorPower, turnMotorPower);
    // parameters to move onto next command
    // parameters are if drive and turn values are within margin, if optical is
    // activated and detects red, and if the time on command exceeds 3 seconds
    if ((checkDriveErrors(driveError, driveMargin, driveFinalValue) &&
         checkTurnErrors(turnError, turnMargin)) ||
        (t.time(msec) > timing.at(i))) {
      // reset timer for next command
      t.reset();
      // stop robot and reset rotation sensors
      driveTrain.stop(brake);
      par.resetPosition();
      perp.resetPosition();
      // updates drive and turn targets, intake state, branching, voltage cap
      updatePIDConditions(&driveFinalValue, &turnFinalValue, &voltageCap,
                          branchTimer);
      Brain.Screen.printAt(20, 200, "%f", driveError);
      autonBrain(driveError);
    }
    // wait to prevent wasted resources and allow sensors to update
    wait(10, msec);
  }
  return 1;
}
// ............................................................................
// ............................................................................
// ............................................................................

void rollerSide() {
  fourBar.open();
  driveTrain.driveFor(24, inches, 70,velocityUnits::pct);
}
void nonRoller() {
  driveTrain.driveFor(-36, inches, 50,velocityUnits::pct);
  intake.spin(reverse,100,pct);
  wait(1,sec);
  driveTrain.driveFor(-15, inches, 100,velocityUnits::pct);

}
void rollerSideAlt() {}
void skills() {}
void nonRollerAlt() {}
// auton method to tune PID
void pidTune() {}

// easy way to switch between different autons
double autoSelect = 0;
void auton() {
  if (autoSelect == 1) {
    rollerSide();
  } else if (autoSelect == 2) {
    nonRoller();
  } else if (autoSelect == 3) {
    rollerSideAlt();
  } else if (autoSelect == 4) {
    skills();
  } else if (autoSelect == 5) {
    nonRollerAlt();
  } else if (autoSelect == 6) {

  } else if (autoSelect == 7) {
    pidTune();
  }
}

void autonomous(void) {
  autoSelect = 2;
  auton();
}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*---------------------------------------------------------------------------*/
// task to have automatic intake control
// int catas;
// int driveCataPI(){
//   double cataFinalDistance;
//   double kP=5.8;
//   double kI=1;
//   double temp;
//   double cataTotalError=0;
//   switch (catas){
//     case 0:
//       //cata position 1
//       cataFinalDistance=0.9;
//       temp=cataFinalDistance;
//       break;
//     case 1:
//       cataFinalDistance=0.9;
//       temp=cataFinalDistance;
//       break;
//     case 2:
//       cataFinalDistance=-5;
//       temp=cataFinalDistance;
//       break;
//     default:
//       Controller1.rumble(".-");
//       cataFinalDistance=3.5;
//       break;
//   } 
//   while(true){
//     double cataPosition=dist.objectDistance(distanceUnits::cm);
//     if (cataPosition<cataFinalDistance){
//       cataFinalDistance=0;
//     }
//     else{
//       cataFinalDistance=temp;
//     }
  
//     //proportional
//     double cataError=cataFinalDistance-cataPosition;

//     //integral
//     if(fabs(cataError)<1){
//       cataTotalError+=cataError;
//     }
//     else{
//       cataTotalError=0;
//     }

//     //sets up the motor power for each side
//     double cataMotorPower=cataError*kP+cataTotalError*kI;
//     if(cataMotorPower>0){
//       cataMotorPower=0;
//     }
//     if (cataMotorPower==0||fabs(cataError)<0.1){
//       cata.stop(coast);
//     }

//     else{
//       cata.spin(fwd,-cataMotorPower,volt);
//     }
    
//     //this helps you see any undershooting or overshooting
//     Brain.Screen.printAt(120,200,"%f",cataError);
//     Brain.Screen.printAt(300,200,"%f",-cataMotorPower);

//     //Sleep the PD for a short amount of time to prevent wasted resources.
//     wait(10,msec);
//   }
//   return 1;
// }
// int prevCataPos=0;
// void cataPos1(){
//   catas=0;
//   prevCataPos=0;
// }
// void cataPos2(){
//   catas=1;
//   prevCataPos=1;
// }
// bool intakeToggle = true;
// directionType intakeDirection = directionType::fwd;
// int ballIntake() {
//   while (intakeToggle) {
//     if (fourBar.value()==0){
//       intake.stop(coast);
//     }
//     else if (intakeDirection == fwd) {
//       intake.spin(intakeDirection, 12, volt);
//     }
//     else if (intakeDirection == directionType::rev) {
//       intake.spin(intakeDirection, 12, volt);
//     }
//   }
//   intake.stop(coast);
//   return 1;
// }
void deadzone(double* moveStick, double* turnStick, double moveDZone = 0,
              double turnDZone = 0) {
  if (fabs(*moveStick) < moveDZone) {
    *moveStick = 0;
  }
  if (fabs(*turnStick) < turnDZone) {
    *turnStick = 0;
  }
}
void joystickControl(double* moveStick, double* turnStick) {
  double turnTune;
  if (*turnStick < 0) {
    turnTune = 18.1212;
  } else {
    turnTune = 5.12121;
  }
  rightSide.spin(
      fwd,
      ((*moveStick * 12 / 127) - (turnTune * pow(1.01, *turnStick) - turnTune)),
      volt);
  leftSide.spin(
      fwd,
      ((*moveStick * 12 / 127) + (turnTune * pow(1.01, *turnStick) - turnTune)),
      volt);
}
void intakePistons(){
  if (fourBar.value()==0){
    fourBar.set(1);
  }
  else if (fourBar.value()==1){
    fourBar.set(0);
  }
}
double cataPosition=0;
void changePos(){
  if (cataPosition==0){
        cataPosition=1;
      }
  else if (cataPosition==1){
        cataPosition=0;
      }
}
void flaps(){
  if (wing1.value()==1){
    wing1.set(0);
    wing2.set(0);
  }
  else if (wing1.value()==0){
    wing1.set(1);
    wing2.set(1);
  }
}
double bi=12;
void intakePower(){
  if (bi==12){
    bi=0;
  }
  else if(bi==0){
    bi=12;
  }
}
void usercontrol(void) {
  // code to run auton in driver.
  // catas=0;
  if (autoSelect == 4) {
    Inertial.resetRotation();
    wait(500, msec);
    auton();
    wait(200, sec);
  }
  Controller1.ButtonL1.pressed(intakePistons);
  Controller1.ButtonL2.pressed(flaps);
  Controller1.ButtonA.pressed(changePos);
  Controller1.ButtonB.pressed(intakePower);
  double cata0=3;
  double cata1=23;

  
  // Controller1.ButtonL2.pressed(cataPos2);
  // Controller1.ButtonR2.pressed(cataPos1);
  while (1) {
    // variables to control joystick sensitivity
    double moveStick = Controller1.Axis3.value();
    double turnStick = Controller1.Axis1.value();
    togglePID = 0;
    deadzone(&moveStick, &turnStick,10,10);
    joystickControl(&moveStick, &turnStick);

    if (Controller1.ButtonR2.pressing()){
      intake.stop(coast);
    }
    else if (rot.position(deg)>cata0+5&&cataPosition==0){
      intake.stop(coast);
    }
    else if (rot.position(deg)>cata1+5&&cataPosition==1){
      intake.stop(coast);
    }
    else if(fourBar.value()==1){
      intake.spin(fwd,bi,volt);
    }
    else if(fourBar.value()==0){
      intake.spin(reverse,bi,volt);
    }
    if (Controller1.ButtonR1.pressing()&&fourBar.value()==1){
      cata.spin(fwd,12,volt);
    }
    else if (cataPosition==0&&rot.position(deg)>cata0){
      cata.spin(fwd,12,volt);
    }
    else if (cataPosition==1&&rot.position(deg)>cata1){
      cata.spin(fwd,12,volt);
    }
    else{
      cata.stop(coast);
    }
    // initialize task for intake
    // task Intake(ballIntake);
    // task CATA(driveCataPI);
    //driveBrain();
    Brain.Screen.printAt(20,200,"%f",cata.voltage());
    wait(10, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

// Main will set up the competition functions and callbacks.
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
// odometry
// absolute position tracking system
// // working but not implemented due to time constraints
// double parDis = 1;
// double perpDis = 1;
// double rightDis = 1;
// double previousPar = 0;
// double previousPerp = 0;
// double previousTheta = 0;
// double previousX = 0;
// double previousY = 0;
// double totalPar = 0;
// double theta = 0;
// double xTranslation = 0;
// double yTranslation = 0;
// double xGlobal = 0;
// double yGlobal = 0;
// int odometry() {
//   while (1) {
//     double parPos = par.position(deg) * (M_PI * 2.75) / 360;
//     double perpPos = perp.position(deg) * (M_PI * 2.75) / 360;
//     double deltaPar = parPos - previousPar;
//     double deltaPerp = perpPos - previousPerp;
//     previousPar = parPos;
//     previousPerp = perpPos;
//     theta = Inertial.rotation() * M_PI / 180;
//     double deltaTheta = theta - previousTheta;
//     Brain.Screen.printAt(20, 20, "%f", deltaPar);
//     Brain.Screen.printAt(20, 40, "%f", yGlobal);

//     // double leftRadius=(deltaLeft/deltaTheta) + leftDis;
//     if (deltaTheta == 0) {
//       xTranslation = deltaPerp;
//       yTranslation = deltaPar;
//     } else {
//       xTranslation =
//           (2 * sin(theta / 2)) * ((deltaPerp / deltaTheta) + perpDis);
//       yTranslation = (2 * sin(theta / 2)) * ((deltaPar / deltaTheta) +
//       parDis);
//     }
//     double averageTheta = previousTheta + (deltaTheta / 2);
//     double polarRadius = sqrt((pow(xTranslation, 2) + pow(yTranslation, 2)));
//     // double polarTheta = (atan(yTranslation / xTranslation)) -
//     averageTheta; double xGlobalOffset = polarRadius * cos(averageTheta);
//     double yGlobalOffset = polarRadius * sin(averageTheta);
//     xGlobal = previousX + xGlobalOffset;
//     yGlobal = previousY + yGlobalOffset;
//     previousX = xGlobal;
//     previousY = yGlobal;
//     previousTheta = theta;

//     wait(20, msec);
//   }
//   return 1;
// }
