#include "api.h"
#include "auton.h"
#include "main.h"
#include "robot.h"
#include "pid.h"
#include "odometry.h"

using namespace std;
using namespace pros;

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or t
 * he VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */


void autonomous() {
  
  //INDEX
    //AUTON 0: SKILLS
    //AUTON 1: red ring
    //AUTON 2: blue ring
    //AUTON 3: red goal
    //AUTON 4: blue goal
    //AUTON 5: red solo
    //AUTON 6: blue solo
    //AUTON 7: start red right rush mogo ???
    //AUTON 8: red safe ring???

//111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111


  if (atn == 0) {
    imu.tare();

    HOOKS.tare_position();    
    LadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    LadyBrown.move(10);
    HOOKS.move(-127);
    delay(500);
    HOOKS.move(127);
    //LBMacro = 4;
    HOOKS.move(-127);
    driveStraight2(500);
  
    stallProtection = true;
    driveTurn2(-90);
    //mogoValues = true;
    driveClamp(-900, 100, 40);
    mogoValues = true;
    driveTurn2(0);
    driveStraightC(550); //500 //450 ///////////////////untested far
    driveArcRF(26, 800, 2000); //26
    driveStraight2(2200);
    // for(int i = 0; i <= 5000; i++){
    //  if (i % 150 == 0){
    //     con.print(2, 0, "imu: %f        ", float(imu.get_heading()));
    //   } 
    //   delay(1);
    // }

    delay(150); //450
    LBMacro = 1;
    stallProtection = false;
    HOOKS.move(-127);
  
    driveStraight2(475, 80);//500
    //LBMacro = 1;
    driveTurn2(20);//25
    driveStraight2(-1750); //-1800
    delay(150);
    LBMacro = 2;
    //HOOKS.move(40);
    hookControl2 = true;
    driveTurn2(90);
    hookControl2 = false;
    HOOKS.move(-127);
    // LBMacro = 0;
    // LadyBrown.move(0);
    driveStraight2(600); //500
    driveTurn2(90);
    //old hook control spot
    delay(400); //500
    HOOKS.move(0);
    delay(50); //100
    HOOKS.move(-127);
    delay(100); //200


    chasMove(100, 100);
    delay(400); //400
    HOOKS.tare_position();   
    hookControl2 = true;
    for(int i = 0; i <= 800; i++){
      LBMacro = 3;
      LadyBrownMacro();
      delay(1);
    }
    hookControl2 = false;
    // LadyBrown.move(-127);
    // delay(800);
    driveStraight2(-550); //-650
    LBMacro = 0;
    LadyBrown.move(127);
    //LBMacro = 4;

    //LadyBrown.move(127);
    // second part
    HOOKS.move(-127);
    stallProtection = true;
    // mogo.set_value(true);
    // delay(200);
    //second part
    driveTurn2(180);
    driveStraight2(2100, 40);
    LadyBrown.move(40);
    driveTurn2(60);
    driveStraight2(450);//650
    driveTurn2(-45);
    driveStraight2(-300);
    mogo.set_value(false);
    mogoValues = false;
    stallProtection = false;
    HOOKS.move(127);
    driveTurn2(-45);
    driveStraight2(150);
    driveTurn2(91);
    longValues = true;
    driveClamp(-3225, 100, 50);
    longValues = false;
    mogoValues = true;
    HOOKS.move(-127);
    stallProtection = true;
    driveStraight2(125);
    // driveTurn2(0);

    //////////////////////////////////////////////////////////////

    mogoValues = true;
    





    ////////////////old 2nd half
    driveTurn2(0);
    driveStraightC(475); //500 //450
    driveArcLF(26, 800, 2000); //26
    driveStraight2(2200);
    // for(int i = 0; i <= 5000; i++){
    //  if (i % 150 == 0){
    //     con.print(2, 0, "imu: %f        ", float(imu.get_heading()));
    //   } 
    //   delay(1);
    // }
    stallProtection = false;
    HOOKS.move(-127);

    delay(150); //450
    LBMacro = 1;
    driveStraight2(475, 80);//500
    //LBMacro = 1;
    driveTurn2(-20);//25
    driveStraight2(-1750); //-1800
    delay(150);
    LBMacro = 2;
    //HOOKS.move(40);
    hookControl2 = true;
    driveTurn2(-90);
    hookControl2 = false;
    HOOKS.move(-127);
    // LBMacro = 0;
    // LadyBrown.move(0);
    driveStraight2(575); //500
    driveTurn2(-90);
    //old hook control spot
    delay(400); //500
    HOOKS.move(0);
    delay(50); //100
    HOOKS.move(-127);
    delay(100); //200
    
    HOOKS.move(40);
    chasMove(100, 100);
    delay(400); //400
    hookControl2 = true;
    for(int i = 0; i <= 800; i++){
      LBMacro = 3;
      LadyBrownMacro();
      delay(1);
    }
    hookControl2 = false;
    // LadyBrown.move(-127);
    // delay(800);
    driveStraight2(-550); //-650
    LBMacro = 0;
    LadyBrown.move(127);
    //LBMacro = 4;

    //LadyBrown.move(127);
    // second part
    HOOKS.move(-127);
    stallProtection = true;
    // mogo.set_value(true);
    // delay(200);
    //second part
    driveTurn2(180);
    driveStraight2(2100, 40);
    LadyBrown.move(40);
    driveTurn2(-60);
    driveStraight2(450);//650
    driveTurn2(45);
    driveStraight2(-300);
    mogo.set_value(false);
    mogoValues = false;
//////////////old 2nd half



    mogoValues = false;
    HOOKS.move(127);
    driveTurn2(46);
    stallProtection = false;
    driveStraightR(3150, 70); //3000





    /*
    driveStraightC(300);
    driveArcLF(15, 800, 2000);
    driveStraight2(2200);
    delay(450);
    LBMacro = 1;
    driveStraight2(800, 80);
    driveTurn2(-5); //-5
    driveStraight2(-1700); //-1750
    delay(450);
    LBMacro = 2;
    //HOOKS.move(40);
    hookControl2 = true;
    driveTurn2(-90);
    hookControl2 = false;
    HOOKS.move(-127);
    LBMacro = 0;
    LadyBrown.move(0);
    driveStraight2(500);
    driveTurn2(-90);
    //old hook control spot
    delay(1200);
    HOOKS.move(0);
    delay(100);
    HOOKS.move(-127);
    delay(400);


    chasMove(70, 70);
    delay(400);
    hookControl2 = true;
    for(int i = 0; i <= 800; i++){
      LBMacro = 3;
      LadyBrownMacro();
      delay(1);
    }
    hookControl2 = false;
    // LadyBrown.move(-127);
    // delay(800);
    driveStraight2(-650);
    LadyBrown.move(127);
    // second part
    HOOKS.move(-127);
    // mogo.set_value(true);
    // delay(200);
    //second part
    driveTurn2(180);
    driveStraight2(2100, 40);
    LadyBrown.move(40);
    driveTurn2(-60);
    driveStraight2(650);
    driveTurn2(20);
    driveStraight2(-450);
    mogo.set_value(false);
    mogoValues = false;
    HOOKS.move(127);

    */

    



    
    ///// third part
    
    LBMacro = 0;
    LadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    LadyBrown.move(30);
    // driveTurn2(45);
    HOOKS.move_relative(350, 600);
    driveStraight2(1000);
    HOOKS.move_relative(-500, 300);
    driveStraight2(400);
    driveTurn2(115);
    driveClamp(-1500, 150, 50);
    color = 1;
    HOOKS.move(-127);
    stallProtection = true;
    driveTurn2(-140);
    driveStraight2(1300);
    driveTurn2(-40);
    driveStraightC(600);
    driveArcRF(40, 300, 1800);
    doinker.set_value(true);
    driveStraight2(250, 70);
    driveTurn2(-60);
    driveStraight2(550);
    delay(400);
   // driveTurn2(-170);
    driveTurn2(135); //120
    doinker.set_value(false);
    driveStraight2(-400);
    mogo.set_value(false);
    stallProtection = false;
    color = 0;
    HOOKS.move(127);
    driveTurn2(135);
    driveStraightC(700);
    HOOKS.move(-127);
    driveArcLF(90, 300, 2000);
    driveStraightC(700);
    driveArcRF(30, 300, 2000);
    HOOKS.move(0);
    driveStraight2(2500);
    intake.set_value(true);
    driveTurn2(40);
    LBMacro = 6;
    color = 0;
    driveStraight2(-2700, 65);
    LBMacro = 0;
    LadyBrown.move(127);



    





    // LBMacro = 0;
    // LadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    // LadyBrown.move(127);
    // driveTurn2(45);
    // LadyBrown.move(40);
    // HOOKS.move(0);
    // driveStraightC(3200);
    // driveArcLF(115, 250, 1800);
    // driveStraightC(1000);
    // driveArcRF(30, 250, 1900);
    // driveArcLF(30, 250, 1900);
    // driveStraight2(900);
    // driveStraight2(-1600);
    // driveTurn2(65);
    // driveStraight2(2400);



    // driveStraightC(1500);
    // HOOKS.move_relative(-400, 400);
    // driveStraight2(500);
    // driveStraight2(-300);
    // driveTurn2(135);
    // driveClamp(-1200, 150, 50);
    // HOOKS.move(-127);
    // driveTurn2(-105);
    // driveStraight2(-2600);
    // mogo.set_value(false);
    // driveStraight2(500);
    // driveTurn2(75);
    // driveStraight2(1500);




    // driveTurn2(-135);
    // driveStraight2(1200);
    // driveTurn2(-75);
    // driveStraightC(400);
    // driveArcLF(120, 90, 1800);
    // driveStraight2(500);


    
    



    /*
    // LadyBrown.move(-127);
    // delay(500);
    // driveClamp(-500, 100);
    

    LadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    //LadyBrown.move(40);
    HOOKS.move(-127);
    delay(500);
    HOOKS.move(127);
    LBMacro = 4;
    driveStraight2(500);
    HOOKS.move(-127);
    driveTurn2(-90);
    mogoValues = true;
    driveClamp(-900, 100, 40);
    driveTurn2(0);
    driveStraightC(300);
    driveArcRF(45, 800, 2000);
    driveStraightC(425); //425
    driveArcLF(45, 500, 2000);
    driveStraight2(800);
    delay(150); //450
    LBMacro = 1;
    driveStraight2(500, 80);
    driveTurn2(5);
    driveStraight2(-1550); //-1600
    delay(150);
    LBMacro = 2;
    //HOOKS.move(40);
    hookControl2 = true;
    driveTurn2(90);
    hookControl2 = false;
    HOOKS.move(-127);
    LBMacro = 0;
    LadyBrown.move(0);
    driveStraight2(500);
    driveTurn2(90);
    //old hook control spot
    delay(500); //1200
    HOOKS.move(0);
    delay(100);
    HOOKS.move(-127);
    delay(200); //400


    chasMove(70, 70);
    delay(400); 
    hookControl2 = true;
    for(int i = 0; i <= 800; i++){
      LBMacro = 3;
      LadyBrownMacro();
      delay(1);
    }
    hookControl2 = false;
    // LadyBrown.move(-127);
    // delay(800);
    driveStraight2(-650);
    LBMacro = 4;
    //LadyBrown.move(127);
    // second part
    HOOKS.move(-127);
    // mogo.set_value(true);
    // delay(200);
    //second part
    driveTurn2(180);
    driveStraight2(2100, 40);
    LadyBrown.move(40);
    driveTurn2(60);
    driveStraight2(650);
    driveTurn2(-45);
    driveStraight2(-500);
    mogo.set_value(false);
    mogoValues = false;
    HOOKS.move(127);

    driveStraight2(500);
    driveTurn2(90);
    driveClamp(-2700, 100, 40);
    HOOKS.move(-127);
    driveStraight2(225);
    driveTurn2(0);

    //////////////////////////////////////////////////////////////

    driveStraightC(300);
    driveArcLF(45, 800, 2000);
    driveStraightC(800); //425
    driveArcRF(45, 500, 2000);
    driveStraight2(900);
    delay(450);
    LBMacro = 1;
    driveStraight2(800, 80);
    driveTurn2(-5);
    driveStraight2(-1750); //-1600
    delay(450);
    LBMacro = 2;
    //HOOKS.move(40);
    hookControl2 = true;
    driveTurn2(-90);
    hookControl2 = false;
    HOOKS.move(-127);
    LBMacro = 0;
    LadyBrown.move(0);
    driveStraight2(500);
    driveTurn2(-90);
    //old hook control spot
    delay(1200);
    HOOKS.move(0);
    delay(100);
    HOOKS.move(-127);
    delay(400);


    chasMove(70, 70);
    delay(400);
    hookControl2 = true;
    for(int i = 0; i <= 800; i++){
      LBMacro = 3;
      LadyBrownMacro();
      delay(1);
    }
    hookControl2 = false;
    // LadyBrown.move(-127);
    // delay(800);
    driveStraight2(-650);
    LadyBrown.move(127);
    // second part
    HOOKS.move(-127);
    // mogo.set_value(true);
    // delay(200);
    //second part
    driveTurn2(180);
    driveStraight2(2100, 40);
    LadyBrown.move(40);
    driveTurn2(-60);
    driveStraight2(650);
    driveTurn2(20);
    driveStraight2(-450);
    mogo.set_value(false);
    mogoValues = false;
    HOOKS.move(127);



    
    ///// third part
    LBMacro = 0;
    LadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    LadyBrown.move(127);
    driveTurn2(45);
    LadyBrown.move(40);
    HOOKS.move(0);
    driveStraightC(3200);
    driveArcLF(135, 150, 1800);
    driveStraight2(2400);
    driveStraight2(-1500);
    driveTurn2(-65);
    driveStraight2(2400);



    // driveStraightC(1500);
    // HOOKS.move_relative(-400, 400);
    // driveStraight2(500);
    // driveStraight2(-300);
    // driveTurn2(135);
    // driveClamp(-1200, 150, 50);
    // HOOKS.move(-127);
    // driveTurn2(-105);
    // driveStraight2(-2600);
    // mogo.set_value(false);
    // driveStraight2(500);
    // driveTurn2(75);
    // driveStraight2(1500);




    // driveTurn2(-135);
    // driveStraight2(1200);
    // driveTurn2(-75);
    // driveStraightC(400);
    // driveArcLF(120, 90, 1800);
    // driveStraight2(500);
*/

    
    

  } else if (atn == 1) {
  imu.tare();
  ////Red Left Ring
  mogoToggle = true;
  color = 1;
  LadyBrown.move(-127);
  driveStraight2(250);
  delay(100);
 // LBMacro = 4;
  LadyBrown.move(0);
  driveClamp(-1625, 150, 60);//1550
  LadyBrown.move(127);
  HOOKS.move(-127);
  driveTurn2(160);
  stallProtection = true;
  driveStraightC(50); //150
  LadyBrown.move(0);
  driveArcLF(41, 700, 2500, 60);
  driveStraight2(375, 50);
  // driveStraightC(-200);
  // driveArcRF(-13, 700, 2500); 
  // driveStraight2(-50);
  driveStraight2(-200);
  driveTurn2(25);
  driveStraight2(1550, 75); //1300 //45
  
  driveTurn2(70);
  intake.set_value(true);
  driveStraight2(800);
  intake.set_value(false);
  driveStraight2(-100);
  // for(int i = 0; i <= 300; i+=10){
  //    ColorSort();
  //    delay(10);
  //   }
  driveStraight2(150);
  driveStraight2(-50);
  driveStraight2(50);
  driveStraight2(-600);
  driveTurn2(-65);
  intake.set_value(true);
  driveStraight2(1450, 85);
  intake.set_value(false);
    for(int i = 0; i <= 9900; i+=10){
     ColorSort();
     delay(10);
    }
  // driveStraight2(-350);
  // driveStraight2(150);
  // for(int i = 0; i <= 8800; i++){
  //   ColorSort();
  //   delay(10);
  //  }
  // delay(250);
  // mogo.set_value(false);
  // stallProtection = false;
  // HOOKS.move(0);
  // driveTurn2(50);//70
  // driveClamp(-1500, 150, 60);
  // HOOKS.move(-127);
  // driveTurn2(-48);
  // stallProtection = true;
  // driveStraight2(1000);
  // driveTurn2(120);
  // LBMacro = 6;
  // driveStraight2(2000, 50);
  //old red ring
  // mogoToggle = true;
  // color = 1;
  // LadyBrown.move(-127);
  // driveStraight2(250);
  // delay(100);
  // LBMacro = 4;
  // driveClamp(-1600, 150, 60);//1550
  // HOOKS.move(-127);
  // driveTurn2(160);
  // stallProtection = true;
  // driveStraightC(165); //275
  // driveArcLF(41, 700, 2500, 50);
  // driveStraight2(550);
  // driveStraight2(-700);
  // driveArcLF(130, 100, 2500);
  // driveStraight2(50);
  // driveStraight2(-100);
  // driveTurn2(40);

  // stallProtection = false;
  // HOOKS.move(127);
  
  // intake.set_value(true);
  // driveStraight2(2400);
  // intake.set_value(false);
  // HOOKS.move(-127);
  // driveStraight2(-500);
  // stallProtection = false;
  // driveTurn2(-62);
  // intake.set_value(true);
  // driveStraight2(1700, 80);
  // intake.set_value(false);
  // delay(400);
  // driveTurn2(-140);
  // LBMacro = 0;
  // driveStraight2(1200);
  // LadyBrown.move(-127);

// end old red ring
  // LadyBrown.move(-127);


  
  // mogoToggle = true;
  // color = 1;
  // LadyBrown.move(-127);
  // driveStraight2(250);
  // delay(100);
  // LBMacro = 4;
  // driveClamp(-1600, 150, 60);//1550
  // HOOKS.move(-127);
  // driveTurn2(160);
  // stallProtection = true;
  // driveStraightC(165); //275
  // driveArcLF(41, 700, 2500, 50);
  // driveStraight2(425, 50);
  // driveStraightC(-200);
  // driveArcLF(-15, 700, 2500);
  // driveStraight2(-50);
  // driveTurn2(49);
  // driveStraight2(1700, 45);
  // // for(int i = 0; i <= 500; i+=10){
  // //   ColorSort();
  // //   delay(10);
  // // }
  // driveStraight2(-500);
  // stallProtection = false;
  // HOOKS.move(127);
  
  // intake.set_value(true);
  // driveStraight2(1500);
  // intake.set_value(false);
  // HOOKS.move(-127);
  // driveStraight2(-500);
  // stallProtection = false;
  // driveTurn2(-62);
  // intake.set_value(true);
  // driveStraight2(1700, 80);
  // intake.set_value(false);
  // delay(400);
  // driveTurn2(-150);
  // LBMacro = 0;
  // LadyBrown.move(-127);
  // driveStraight2(1200);
  // // LadyBrown.move(-127);






  /*
  driveArcLF(35, 250, 2500);
  driveTurn2(0);
  driveStraight2(300);
  driveTurn2(15);
  */

  // intake.set_value(true);
  // driveStraight2(1950, 60);
  // intake.set_value(false);
  // delay(300);
  // driveStraight2(-300);
  // driveTurn2(-110);
  // driveStraight2(500);


    /*wallstake
    mogo.set_value(false);
    driveTurn2(0);
    driveStraight2(700);
    HOOKS.move(0);
    driveTurn2(120);
    driveStraight2(-500);
    driveTurn2(-150);
    driveStraight2(-200);
    */

    // driveTurn2(90);
 
    // driveStraight2(900);
    // driveStraight2(-300);
    // driveTurn2(-70);
    // driveStraight(600);
    // driveStraight2(-300);




    // ramp.move(-127);
    // driveStraight2(500);
    // driveTurn2(-90);
    // driveClamp(-500, 100);
    // driveTurn2(0);
    // driveStraight2(100);
    // driveTurn2(-90);
    // driveStraight2(125);
    // driveTurn2(35);
    // driveStraight2(350);
    // driveTurn2(180);
    // driveTurn2(-90);
    // driveStraight2(45);
    // driveStraight2()


  } else if (atn == 2) { 
    imu.tare();
    //blue ring
    mogoToggle = true;
    color = 2;
    LadyBrown.move(-127);
    driveStraight2(250);
    delay(100);
   // LBMacro = 4;
    LadyBrown.move(0);
    driveClamp(-1625, 150, 60);//1550
    LadyBrown.move(127);
    HOOKS.move(-127);
    driveTurn2(-160);
    stallProtection = true;
    driveStraightC(50); //150
    LadyBrown.move(0);
    driveArcRF(41, 700, 2500, 60);
    driveStraight2(375, 50);
    // driveStraightC(-200);
    // driveArcRF(-13, 700, 2500); 
    // driveStraight2(-50);
    driveStraight2(-200);
    driveTurn2(-32);
    driveStraight2(1550, 75); //1300 //45
    
    driveTurn2(-70);
    intake.set_value(true);
    driveStraight2(800);
    intake.set_value(false);
    driveStraight2(-100);
    // for(int i = 0; i <= 300; i+=10){
    //    ColorSort();
    //    delay(10);
    //   }
    driveStraight2(150);
    driveStraight2(-50);
    driveStraight2(50);
    driveStraight2(-600);
    driveTurn2(62);
    intake.set_value(true);
    driveStraight2(1500, 85);
    intake.set_value(false);
      for(int i = 0; i <= 800; i+=10){
       ColorSort();
       delay(10);
      }
      driveTurn2(130);
      LBMacro = 5;
      driveStraight2(600);

  // intake.set_value(true);
  // HOOKS.move(-127);
  // driveStraight2(400);
  // intake.set_value(false);//i try to arc at the end but didnt work
  // delay(200);
  // driveStraight2(-400);
  // driveArcLF(-35, 200, 9000);
  // driveClamp(400, 150);
  


    // doinker.set_value(true);
    // driveClampD(1600, 200);
    // driveStraight(-900);
    // doinkerClamp.set_value(false);
    // driveStraight2(-100);
    // doinker.set_value(false);
    // driveTurn2(-170);
    // driveClamp(-800, 150, 60);
    // HOOKS.move(-127);


    /*
    LBMacro = 4;
    driveStraightC(-875);
    driveArcLF(-30, 200, 2000);
    driveClamp(-910, 150);
    HOOKS.move(-127);
    driveStraight2(300);
    driveTurn2(65);
    mogo.set_value(false);
    driveStraight2(100);
    driveTurn2(-115);
    driveClamp(-700, 150, 60);
    driveTurn2(-65);
    driveStraight2(1000);
    driveTurn2(160);
    driveStraight2(-700);
    mogo.set_value(false);
    driveStraight2(700);
    driveTurn2(0);

    // driveTurn2(-40);
    // driveStraight2(1800);
    // intake.set_value(true);
    // driveArcRF(5, 1000, 1800);




    //old tworing
    // driveTurn2(-45);
    // driveStraight2(300);
    // driveTurn2(0);
    // delay(300);
    // mogo.set_value(false);
    // driveTurn2(-88);
    // driveClamp(-900, 150, 70);
    // intake.set_value(true);
    // driveTurn2(-45);
    // driveStraight2(1500);


    //old stuff
    // driveTurn2(41);
    // driveStraight2(975);
    // intake.set_value(false);
    // delay(300);
    // driveStraight2(-300);
    // driveTurn2(125);
    // delay(1000);
    // HOOKS.move(0);
    // driveStraight2(400);

    // driveClamp(-2000, 150);
    // HOOKS.move(-127);
    // driveTurn2(-15);
    // driveStraight2(500);
    // delay(300);
    // mogo.set_value(false);
    // driveTurn2(-105);




        // LadyBrown.move(-127);
    // delay(500);
    // driveClamp(-500, 100);

    // LadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    // LadyBrown.move(40);
    // HOOKS.move(-127);
    // delay(500);
    // HOOKS.move(127);
    // driveStraight2(500);
    // HOOKS.move(-127);
    // driveTurn2(-90);
    // mogoValues = true;
    // driveClamp(-900, 100, 40);
    // driveTurn2(0);
    // driveStraightC(300);
    // driveArcRF(45, 800, 2000);
    // driveStraightC(425);
    // driveArcLF(45, 500, 2000);
    // driveStraight2(800);
    // delay(250);
    // LBMacro = 1;
    // driveStraight2(500, 80);
    // //lb1 old one
    // driveTurn2(180);
    // //old spot 
    // driveStraightC(800);
    // LBMacro = 2;
    // //HOOKS.move(40);
    // hookControl2 = true;
    // driveArcLF(90, 200, 2000);
    // driveStraight2(60);
    // hookControl2 = false;
    // HOOKS.move(-127);
    // LBMacro = 0;
    // LadyBrown.move(0);
    // delay(1200);
    // HOOKS.move(0);
    // delay(100);
    // HOOKS.move(-127);
    // delay(400);


    // chasMove(70, 70);
    // delay(400);
    // HOOKS.move(127);
    // LadyBrown.move(-127);
    // delay(800);
    // driveStraight2(-500);
    // driveTurn2(180);

    */


    
  } else if (atn == 3) { 
    imu.tare();
    // red safe goal
    //elims
   
  mogoToggle = true;
    color = 0;
    LadyBrown.move(-127);
    driveStraight2(250);
    delay(100);
    // LBMacro = 4;
    LadyBrown.move(0);
    //LBMacro = 4;
    driveClamp(-1700, 150, 60);//1550
    LadyBrown.move(127);
    driveTurn2(-103);
    driveStraight2(-1050);
    LadyBrown.move(0);
    driveStraight2(450);
    driveTurn2(-72);
    color = 1;
    driveStraightI(2000, 100, 1500);
    stallProtection = true;
    intake.set_value(true);
    driveStraight2(800, 50);
    intake.set_value(false);
    driveStraight2(-100);
    for(int i = 0; i <= 300; i+=10){
      ColorSort();
      delay(10);
    }
  driveStraight2(150);
  driveStraight2(-50);
  driveStraight2(50);
  driveStraight2(-600);
  driveTurn2(150);
  driveStraight2(950);
  driveTurn2(70);
  driveStraight2(1200);
  LadyBrown.move(-127);
  HOOKS.move(0);

  



  } else if(atn == 4) {
    imu.tare();
    //blue safe goal
    mogoToggle = true;
    color = 0;
    LadyBrown.move(-127);
    driveStraight2(250);
    delay(100);
    // LBMacro = 4;
    LadyBrown.move(0);
    //LBMacro = 4;
    driveClamp(-1700, 150, 60);//1550
    LadyBrown.move(127);
    driveTurn2(103);
    driveStraight2(-1050);
    LadyBrown.move(0);
    driveStraight2(450);
    driveTurn2(72);
    stallProtection = true;
    color = 2;
    driveStraight2(2000);
    intake.set_value(true);
    driveStraight2(800);
    intake.set_value(false);
    driveStraight2(-100);
  for(int i = 0; i <= 300; i+=10){
     ColorSort();
     delay(10);
    }
  driveStraight2(150);
  driveStraight2(-50);
  driveStraight2(50);
  driveStraight2(-600);
  driveTurn2(-150);

  //SAFE SAFE SAFE
  // driveStraight2(950);
  // driveTurn2(-70);
  // driveStraight2(1000);
  // LadyBrown.move(-127);

  //ELIMS.
  // LBMacro = 1;
  // mogo.set_value(false);
  // driveStraight2(950);
  // driveTurn2(161);
  // driveStraight2(800);
  // HOOKS.move(0);
  // LadyBrown.move(-127);


    



  } else if (atn == 5){
    imu.tare();
    //red safe solo
    mogoToggle = true;
    color = 1;
    LadyBrown.move(-127);
    driveStraight2(195);
    delay(100);
   // LBMacro = 4;
    LadyBrown.move(0);
    driveClamp(-1650, 150, 60);//1550
    LadyBrown.move(127);
    HOOKS.move(-127);
    driveTurn2(160);
    stallProtection = true;
    driveStraightC(20); //150
    LadyBrown.move(0);
    driveArcLF(35, 700, 4000, 40);
    driveStraight2(500, 20);
    // driveStraightC(-200);
    // driveArcRF(-13, 700, 2500); 
    // driveStraight2(-50);
    driveStraight2(-225);
    driveTurn2(25);
    driveStraight2(400, 75); //1300 //45
    
    driveTurn2(-30);
    intake.set_value(true);
    driveStraight2(2200, 80);
    intake.set_value(false);
    delay(250);
    mogo.set_value(false);
    stallProtection = false;
    HOOKS.move(0);
    driveTurn2(63);
    driveClamp(-1400, 150, 60);
    HOOKS.move(-127);
    driveTurn2(-49);
    stallProtection = true;
    driveStraight2(800);
    driveTurn2(150);
    driveStraightC(200);
    LBMacro = 5;
    driveStraight2(600);

    //old red safe solo
    // mogoToggle = true;
    // color = 1;
    // LadyBrown.move(-127);
    // driveStraight2(250);
    // delay(100);
    // LBMacro = 4;
    // driveClamp(-1600, 150, 60);//1550
    // HOOKS.move(-127);
    // driveTurn2(160);
    // stallProtection = true;
    // driveStraightC(150); //275
    // driveArcLF(41, 700, 2500, 50);
    // driveStraight2(325, 50);
    // driveStraightC(-200);
    // driveArcLF(-15, 700, 2500);
    // driveStraight2(-50);
    // driveTurn2(49);
    // driveStraight2(700);
    // driveTurn2(-42);
    // intake.set_value(true);
    // driveStraight2(1600);
    // intake.set_value(false);
    // delay(200);
    // driveStraight2(-300);
    // mogo.set_value(false);
    // driveStraight2(100);
    // HOOKS.move(0);
    // driveTurn2(89);
    // driveClamp(-1700, 150, 60);
    // HOOKS.move(-127);
    // driveTurn2(-53);
    // driveStraight2(700);
    // driveTurn2(127);
    // LBMacro= 0;
    // LadyBrown.move(-127);
    // driveStraight2(2400);
    



    // //blue right ring rush
    // LBMacro = 4;
    // driveClamp(-1300, 150, 100);
    // HOOKS.move(-127);
    // driveTurn2(160);
    // driveStraightC(225); //275
    // driveArcLF(39, 700, 2500);
    // driveStraight2(300);
    // driveStraight2(-600);
    // driveArcLF(35, 250, 2500);
    // driveTurn2(0);
    // driveStraight2(300);
    // driveTurn2(-35);
    // intake.set_value(true);
    // driveStraight2(1800, 60);
    // intake.set_value(false);
    // delay(300);
    // driveStraight2(-300);
    // driveTurn2(-110);
    // driveStraight2(500);
    // mogoToggle = true;
    
  } else if (atn == 6){
    imu.tare();
    //blue safe solo
    mogoToggle = true;
    color = 2;
    LadyBrown.move(-127);
    driveStraight2(195);
    delay(100);
   // LBMacro = 4;
    LadyBrown.move(0);
    driveClamp(-1650, 150, 60);//1550
    LadyBrown.move(127);
    HOOKS.move(-127);
    driveTurn2(-160);
    stallProtection = true;
    driveStraightC(20); //150
    LadyBrown.move(0);
    driveArcRF(35, 700, 4000, 40);
    driveStraight2(500, 20);
    // driveStraightC(-200);
    // driveArcRF(-13, 700, 2500); 
    // driveStraight2(-50);
    driveStraight2(-225);
    driveTurn2(-25);
    driveStraight2(400, 75); //1300 //45
    
    driveTurn2(30);
    intake.set_value(true);
    driveStraight2(2200, 80);
    intake.set_value(false);
    delay(250);
    mogo.set_value(false);
    stallProtection = false;
    HOOKS.move(0);
    driveTurn2(-65);
    driveClamp(-1400, 150, 60);
    HOOKS.move(-127);
    driveTurn2(49);
    stallProtection = true;
    driveStraight2(800);
    driveTurn2(-150);
    driveStraightC(200);
    LBMacro = 5;
    driveStraight2(600);
    


    //old robot code
    // driveTurn2(-160);
    // stallProtection = true;
    // driveStraightC(150); //275
    // driveArcRF(41, 700, 2500, 50);
    // driveStraight2(325, 50);
    // driveStraightC(-200);
    // driveArcRF(-15, 700, 2500);
    // driveStraight2(-50);
    // driveTurn2(-49);
    // driveStraight2(700);
    // driveTurn2(42);
    // intake.set_value(true);
    // driveStraight2(1600);
    // intake.set_value(false);
    // delay(200);
    // driveStraight2(-300);
    // HOOKS.move(0);
    // mogo.set_value(false);
    // driveTurn2(-89);
    // driveClamp(-1700, 150, 80);
    // HOOKS.move(-127);
    // driveTurn2(53);
    // driveStraight2(700);
    // driveTurn2(-127);
    // driveStraight2(2400);
    



    // //blue right ring rush
    // LBMacro = 4;
    // driveClamp(-1300, 150, 100);
    // HOOKS.move(-127);
    // driveTurn2(-160);
    // driveStraightC(225); //275
    // driveArcRF(39, 700, 2500);
    // driveStraight2(300);
    // driveStraight2(-600); 
    // driveArcRF(35, 250, 2500);
    // driveTurn2(0);
    // driveStraight2(300);
    // driveTurn2(35);
    // intake.set_value(true);
    // driveStraight2(1800, 60);
    // intake.set_value(false);
    // delay(300);
    // driveStraight2(-300);
    // driveTurn2(110);
    // driveStraight2(500);
    // mogoToggle = true;

  } else if (atn == 7){
    imu.tare();
    
    //start red right rush mogo
    //   mogoToggle = true;
    //   //red right rush mogo
    //   // HOOKS.move(-127);
    //   // driveClampD(1600, 200);
    //   color = 1;
    //   LadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    //   LBMacro = 4;
    //   doinker.set_value(true);
    //   driveClampD(1675, 450, 1000);
    //   HOOKS.move(-20);
    //   driveStraight2(-900);
    //   doinkerClamp.set_value(false);
    //   driveStraight2(-100);
    //   driveTurnD(-160);
    //   doinker.set_value(false);
    //   driveClamp(-800, 150, 60);
    //   HOOKS.move(-127);
    //   driveTurn2(-155);
    //   intake.set_value(true);
    //   driveStraightC(100);
    //   driveArcLF(20, 3300, 2000);
    //   // driveStraight2(500);
      
    //   driveStraight2(700);
    //   intake.set_value(false);
    //   driveStraight2(-1500);
    //   driveTurn2(-115); //-117
    //   mogo.set_value(false);
    //   mogoValues = false;
    //   longValues = true;
    //   driveStraight2(1600);// why is this wiggly..//1675
    //   longValues = false;
    //   for(int i = 0; i <= 800; i++){
    //     LBMacro = 5;
    //     LadyBrownMacro();
    //     delay(1);
    //   }
    //   LBMacro = 4;
    // driveStraight2(-350);
    // driveTurn2(-143); //150
    // driveClamp(-1100, 150, 70);
    // driveTurn2(-120);
    // intake.set_value(true);
    // HOOKS.move(127);
    // driveStraight2(900);
    // HOOKS.move(-127);
    // intake.set_value(false);
    // delay(200);
    // driveTurn2(0);
    // LBMacro = 3;
    // driveStraight2(600);
    
  
  ///end red right rush mogo
    
  
    //   //blue ring
    //   mogoToggle = true;
    //   color = 2;
    //   LadyBrown.move(-127);
    //   driveStraight2(250);
    //   delay(100);
    //  // LBMacro = 4;
    //   LadyBrown.move(0);
    //   driveClamp(-1625, 150, 60);//1550
    //   LadyBrown.move(127);
    //   HOOKS.move(-127);
    //   driveTurn2(-160);
    //   stallProtection = true;
    //   driveStraightC(50); //150
    //   LadyBrown.move(0);
    //   driveArcRF(41, 700, 2500, 60);
    //   driveStraight2(375, 50);
    //   // driveStraightC(-200);
    //   // driveArcRF(-13, 700, 2500); 
    //   // driveStraight2(-50);
    //   driveStraight2(-200);
    //   driveTurn2(-25);
    //   driveStraight2(1550, 75); //1300 //45
      
    //   driveTurn2(-70);
    //   intake.set_value(true);
    //   driveStraight2(800);
    //   intake.set_value(false);
    //   driveStraight2(-100);
    //   // for(int i = 0; i <= 300; i+=10){
    //   //    ColorSort();
    //   //    delay(10);
    //   //   }
    //   driveStraight2(150);
    //   driveStraight2(-50);
    //   driveStraight2(50);
    //   driveStraight2(-600);
    //   driveTurn2(65);
    //   intake.set_value(true);
    //   driveStraight2(1450, 85);
    //   intake.set_value(false);
    //     for(int i = 0; i <= 9900; i+=10){
    //      ColorSort();
    //      delay(10);
    //     }
  /*
      // delay(1500);
      //blue right ring
      mogoToggle = true;
    color = 2;
    LadyBrown.move(-127);
    driveStraight2(250);
    delay(100);
   // LBMacro = 4;
    LadyBrown.move(0);
    driveClamp(-1600, 150, 60);//1550
    LadyBrown.move(127);
    HOOKS.move(-127);
    driveTurn2(-160);
    stallProtection = true;
    driveStraightC(150); //150
    LadyBrown.move(0);
    driveArcRF(41, 700, 2500, 60);
    driveStraight2(325, 50);
    // driveStraightC(-200);
    // driveArcRF(-13, 700, 2500); 
    // driveStraight2(-50);
    driveStraight2(-200);
    driveTurn2(-25);
    driveStraight2(1350, 55); //1300 //45
    HOOKS.move(127);
    // for(int i = 0; i <= 500; i+=10){
    //   ColorSort();
    //   delay(10);
    // }
    //stallProtection = false;
    // HOOKS.move(27);
    // driveStraight2(-500);
    driveTurn2(-66); //-65
    HOOKS.move(-127);
    intake.set_value(true);
    driveStraight2(750, 50);
    delay(150);
    intake.set_value(false);
    stallProtection = true;
    delay(700);
  
    driveStraight2(-500);
    driveTurn2(62);
    intake.set_value(true);
    driveStraight2(2000, 80); //1700
    intake.set_value(false);
    //delay(300);
    driveTurn2(150);
    LBMacro = 0;
    //LadyBrown.move(-127);
    driveStraight2(1200, 50);
    // LadyBrown.move(-127);
    */
  
  
  
      // //blue right ring
      // LadyBrown.move(-127);
      // driveStraight2(250);
      // delay(700);
      // LBMacro = 4;
      // driveClamp(-1600, 150, 80);//1550
      // HOOKS.move(-127);
      // driveTurn2(-160);
      // driveStraightC(225); //275
      // driveArcRF(39, 700, 2500);
      // driveStraight2(300);
      // driveStraight2(-600);
      // driveArcRF(35, 250, 2500);
      // driveTurn2(0);
      // driveStraight2(300);
      // driveTurn2(34);
      // intake.set_value(true);
      // driveStraight2(1550, 60);
      // intake.set_value(false);
      // delay(300);
      // driveStraight2(-300);
      // driveTurn2(110);
      // driveStraight2(900);
      // mogoToggle = true;
  
  
  
    


  } else if (atn == 8){
    

    imu.tare();
    //mogoToggle = true;

/*
    // red safe ring 
    mogoToggle = true;
  color = 1;
  LadyBrown.move(-127);
  driveStraight2(250);
  delay(100);
 // LBMacro = 4;
  LadyBrown.move(127);
  driveClamp(-1600, 150, 60);//1550
  HOOKS.move(-127);
  LadyBrown.move(0);
  driveTurn2(160);
  stallProtection = true;
  driveStraightC(150); //275
  driveArcLF(41, 700, 2500, 50);
  driveStraight2(325, 50);
  // driveStraightC(-200);
  // driveArcRF(-13, 700, 2500);
  // driveStraight2(-50);
  driveStraight2(-200);
  driveTurn2(25);
  driveStraight2(1300, 45);
  HOOKS.move(127); //new
  
  // for(int i = 0; i <= 500; i+=10){
  //   ColorSort();
  //   delay(10);
  // }
  stallProtection = false;
  // HOOKS.move(27);
  // driveStraight2(-500);
  driveTurn2(65);
  HOOKS.move(-127);
  intake.set_value(true);
  driveStraight2(700, 50);
  delay(450);
  intake.set_value(false);
  delay(500);

  driveStraight2(-500);
  driveTurn2(-62);
  intake.set_value(true);
  driveStraight2(1700, 80);
  intake.set_value(false);
  //delay(300);
  driveTurn2(-150);
  LBMacro = 0;
  //LadyBrown.move(-127);
  driveStraight2(1200, 30);
  // LadyBrown.move(-127);
*/

    //blu left goal
    // color = 2;
    // LadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    // LBMacro = 4;
    // doinker.set_value(true);
    // driveClampD(1675, 450, 1000);
    // HOOKS.move(-15);
    // driveStraight2(-900);
    // doinkerClamp.set_value(false);
    // driveStraight2(-100);
    // driveTurnD(-160);
    // doinker.set_value(false);
    // driveClamp(-800, 150, 60);
    // HOOKS.move(-127);
    // driveTurn2(-130);
    // intake.set_value(true);
    
    // doinker.set_value(true);
    // driveStraight2(2400, 90);
    // HOOKS.move(127);
    

    // driveTurn2(100);
    // intake.set_value(false);
    // driveStraight2(200);
    // mogo.set_value(false);
    // doinker.set_value(false);
    // driveTurn2(-110);
    // driveClamp(-1400, 150, 60);
    // LBMacro =0;
    // LadyBrown.move(60);
    // driveStraight2(400);
    // driveTurn2(60);



  //   driveStraightC(300);
  //   driveArcLF(30, 500, 2500);    
  //   mogoValues = false;
  //   longValues = true;
  //   driveStraight2(1400);// why is this wiggly..//1675
  //   longValues = false;
  //   for(int i = 0; i <= 800; i++){
  //     LBMacro = 5;
  //     LadyBrownMacro();
  //     delay(1);
  //   }
  //   LBMacro = 4;
  // driveStraight2(-350);
  // mogo.set_value(false);
  // driveTurn2(-110); //150
  // driveClamp(-1100, 150, 70);
  // driveTurn2(-120);
  // intake.set_value(true);
  // HOOKS.move(127);
  // driveStraight2(900);
  // HOOKS.move(-127);
  // intake.set_value(false);
  // delay(200);
  // driveTurn2(0);
  // LBMacro = 3;
  // driveStraight2(600);
 


    // LBMacro = 4;
    // driveStraightC(-875);
    // driveArcRF(-30, 200, 2000);
    // driveClamp(-910, 150);
    // HOOKS.move(-127);
    // driveStraight2(300);
    // driveTurn2(45);sssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssss
    // driveStraight2(300);
    // driveTurn2(0);
    // delay(300);
    // mogo.set_value(false);
    // driveTurn2(88);
    // driveClamp(-900, 150, 70);
    // intake.set_value(true);
    // driveTurn2(-41);
    // driveStraight2(975);
    // intake.set_value(false);
    // delay(300);
    // driveStraight2(-300);
    // driveTurn2(-125);
    // delay(1000);
    // HOOKS.move(0);
    // driveStraight2(400);
    // mogoToggle = true;
    

  }
}