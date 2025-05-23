#include "api.h"
// #include "auton.h"
#include "main.h"
#include "pid.h"
#include "robot.h"
#include "auton.h"
#include "odometry.h"
// #include "<valarray>"
// #include "<sstream>"
// #include "<string>"


using namespace pros;
using namespace c;
using namespace std;

double trueTarget = 0;

bool mogoValues = false;
bool longValues = false;
bool stallProtection = false;
// bool stalled = false;
// int stallTime = 0;
int direc;
int direc2;
int hookpos;
int prevhookpos;
float view;
int stallC = 0;

double vKpl;
double vKil;
double vKdl;
float errorl; //amount from target
double prevErrorl; 
//double h;
int integrall; 
int derivativel;
int time2l;
double powerl;

//constants used for calculating power/voltage
double vKp;
double vKi;
double vKd;
float error; //amount from target
double prevError; 
int integral; 
int derivative;
int time2;
double power; //voltage provided to motors at any given time to reach the target



//calc2
double vKp2;
double vKi2;
double vKd2;
float error2; //amount from target
double prevError2; 
double h2;
int integral2;
int derivative2;
int time22;
double power2;

//calc
double vKp3;
double vKi3;
double vKd3;
float error3; //amount from target
double prevError3; 
double h3;
int integral3;
int derivative3;
int time23;
double power3;

double vKp4;
double vKi4;
double vKd4;
float error4; //amount from target
double prevError4; 
double h4;
int integral4;
int derivative4;
int time24;
double power4;
int LBMacro = 0;
double LBPos = 0;

double vKp5;
double vKi5;
double vKd5;
float error5; //amount from target
double prevError5; 
double h5;
int integral5;
int derivative5;
int time25;
double power5;

bool hookControl2;


void hooks(int speed){
    direc = speed;
}

void intake2(int speed){
    direc2 = speed;
}



void stall(){


    //direc = hspeed;

    // if(sreverse){
    //     direc = -127;
    // } else {
    //     direc = 127;
    // }

    // if (reverse){
    //     direc = 127;
    // } else if(HOOKS.get_voltage() < -50) {
    //     direc = -127;
    // } else {
    //     direc = 0;
    // }

    // direc = HOOKS.get_voltage()/1000.0;

    if(stallProtection){
        prevhookpos = hookpos;
        hookpos = HOOKS.get_position();

        if((hookpos == prevhookpos)){
            stallC += 10;
        } else {
            stallC = 0;
        }

        if(stallC>200){
            stalled = true;
        }

        if (stalled){
           HOOKS.move(127);
           // INTAKE.move(-direc2);
            stallTime += 10;
            if(stallTime >= 100){
                stalled = false;
                stallTime = 0;
            }
            view = 1;
        } else {
            HOOKS.move(-127);
            //INTAKE.move(direc2);
            stallTime = 0;
            view = 0;
        }

        
    }
}


void setConstants(double kp, double ki, double kd) {
    vKp = kp;
    vKi = ki;
    vKd = kd;
} 

void setConstants2(double kp, double ki, double kd) {
    vKp2 = kp;
    vKi2 = ki;
    vKd2 = kd;
} 





void resetEncoders() { //reset the chassis motors every time a target is reached
    LF.tare_position(); //or set_zero_position(0) or set_zero_position(LF.get_position()); (sets current encoder position to 0)
    LB.tare_position();
	RF.tare_position();
	RB.tare_position();
    RM.tare_position();
	LM.tare_position();
}






//setting method for driving straight or turning (pos neg voltages change directions)
void chasMove(int left, int right) { //voltage to each chassis motor
    LF.move(left);
    LM.move(left);
    LB.move(left);
    RF.move(right);
    RM.move(right);
    RB.move(right);
}


double calcPIDlift(double target, double input, int integralKi, int maxIntegral, double bias) { //basically tuning i here
    int integrall;
    prevErrorl = errorl;
    errorl = target - input;
    
    if(std::abs(errorl) < integralKi) {
        integrall += errorl;
    } else {
        integrall = 0;
    }

    if(integrall >= 0) {
        integrall = std::min(integrall, maxIntegral); //min means take whichever value is smaller btwn integral and maxI
        //integral = integral until integral is greater than maxI (to keep integral limited to maxI)
    } else {
        integrall = std::max(integrall, -maxIntegral); //same thing but negative max
    }
    
    derivativel = errorl - prevErrorl;

    powerl = (vKp2 * errorl) + (vKi2 * integrall) + (vKd2 * derivativel);
    
    //multiply only on the way up  
    // if(error < 0){
    //     powerl = powerl*bias;
    // }
    //multiply on the way up divide on the way down
    if(errorl < 0){
        powerl =  powerl*0.65;
    } else {
        powerl = powerl/bias;
    }
    // straight add voltage to all scenarios 
    // powerl += bias;

    // if(powerl > 40){
    //     powerl = 40;
    // } else if (powerl < -40){
    //     powerl = -40;
    // }


    return powerl;
}

double calcPIDH(double target, double input, int integralKi, int maxIntegral, bool slewOn = false) { //basically tuning i here
    int integral4;
    prevError4 = error4;
    error4 = target - input;
    
    if(std::abs(error4) < integralKi) {
        integral4 += error4;
    } else {
        integral4 = 0;
    }

    if(integral4 >= 0) {
        integral4 = std::min(integral4, maxIntegral); //min means take whichever value is smaller btwn integral and maxI
        //integral = integral until integral is greater than maxI (to keep integral limited to maxI)
    } else {
        integral4 = std::max(integral4, -maxIntegral); //same thing but negative max
    }
    
    derivative4 = error4 - prevError4;

    

    power4 = (vKp2 * error4) + (vKi2 * integral4) + (vKd2 * derivative4);

    return power4;
}


void LadyBrownMacro(){
    LBPos = roto.get_angle();
    if(LBPos > 30000){
        LBPos -= 36000;
    }
    if(hookControl2){
        setConstants2(1, 0, 0);
        HOOKS.move(calcPIDH(175, HOOKS.get_position(), 0, 0, true));
        if(abs(175 - HOOKS.get_position()) < 10){
          hookControl2 = false;
        }
    } else {
        //HOOKS.tare_position();
    }
    setConstants2(0.03, 0, 0);
    if(LBMacro == 1){
       // setConstants2(0.04, 0, 100);
        LadyBrown.move(-calcPIDlift(3600, LBPos, 0, 0, 1.0));
    } else if(LBMacro == 2){
        //setConstants2(0.05, 0, 500);
        LadyBrown.move(-calcPIDlift(4900, LBPos, 0, 0, 1.0)); //5200
    } else if(LBMacro == 3){
        //setConstants2(0.03, 0, 0);
        LadyBrown.move(-calcPIDlift(18000, LBPos, 0, 0, 1.0));
    } else if(LBMacro == 4){
        setConstants2(0.03, 0, 0);
        LadyBrown.move(-calcPIDlift(10000, LBPos, 0, 0, 1.0));
    } else if(LBMacro == 5){
        LadyBrown.move(-calcPIDlift(22000, LBPos, 0, 0, 1.0));
    } else if (LBMacro == 6){
        LadyBrown.move(-calcPIDlift(14000, LBPos, 0, 0, 1.0));
    }
}

bool InitColor = false;
//bool InitCorrect = false;
int ColorCount;
bool stalled = false;
int hookCount = 0;
double hookPos = 0;
double prevHook = 0;
int stallTime = 0;

void ColorSort(){
    OpticalC.set_led_pwm(100);
    if(color == 1){ //sort out blue
        if((OpticalC.get_hue()<270 && OpticalC.get_hue()>170) && OpticalC.get_proximity() > 100){
            InitColor = true;
        }

        if(InitColor && ColorCount < 550){
            colorSorter.set_value(true);
            ColorCount += 10;
        } else {
            ColorCount = 0;
            InitColor = false;
            colorSorter.set_value(false);
        }
        

        // if(InitColor == false){
        //     prevHook = hookPos;
        //     hookPos = HOOKS.get_position();
        //     if(prevHook==hookPos){
        //         hookCount++;
        //     }

        //     if(hookCount>8){ 
        //         stalled = true;
        //     }

        //     if(stalled){
        //         HOOKS.move(-127);
        //         stallTime+=10;
        //         if(stallTime > 300){
        //             hookCount = 0;
        //             stalled = false;
        //             stallTime = 0;
        //         }
        //     } else {
        //         HOOKS.move(127);
        //     }
        // }

    } else if (color == 2){ //sort out red
        if((OpticalC.get_hue()<18|| OpticalC.get_hue()>350) && OpticalC.get_proximity() > 150){
            InitColor = true;
        }

        if(InitColor && ColorCount < 550){
            colorSorter.set_value(true);
            ColorCount += 10;
        } else {
            ColorCount = 0;
            InitColor = false;
            colorSorter.set_value(false);
        }

    //     if(InitColor == false){
    //         prevHook = hookPos;
    //         hookPos = HOOKS.get_position();
    //         if(prevHook==hookPos){
    //             hookCount++;
    //         }

    //         if(hookCount>8){ 
    //             stalled = true;
    //         }

    //         if(stalled){
    //             HOOKS.move(-127);
    //             stallTime+=10;
    //             if(stallTime > 300){
    //                 hookCount = 0;
    //                 stalled = false;
    //                 stallTime = 0;
    //             }
    //         } else {
    //             HOOKS.move(127);
    //         }
    // }
}
}




double calcPID(double target, double input, int integralKi, int maxIntegral, bool slewOn = false) { //basically tuning i here
    odometry2();
    stall();
    LadyBrownMacro();
    ColorSort();
    int integral;
    
    prevError = error;
    error = target - input;
    

    if(abs(error) < integralKi) {
        integral += error;
    } else {
        integral = 0;
    }

    if(integral >= 0) {
        integral = min(integral, maxIntegral); //min means take whichever value is smaller btwn integral and maxI
        //integral = integral until integral is greater than maxI (to keep integral limited to maxI)
    } else {
        integral = max(integral, -maxIntegral); //same thing but negative max
    }



    derivative = error - prevError;

    power = (vKp * error) + (vKi * integral) + (vKd * derivative);

    return power;
} 



double calcPID2(double target, double input, int integralKi, int maxIntegral, bool slewOn = false) { //basically tuning i here
    int integral2;
    prevError2 = error2;
    error2 = target - input;
    
    if(std::abs(error2) < integralKi) {
        integral2 += error2;
    } else {
        integral2 = 0;
    }

    if(integral2 >= 0) {
        integral2 = std::min(integral2, maxIntegral); //min means take whichever value is smaller btwn integral and maxI
        //integral = integral until integral is greater than maxI (to keep integral limited to maxI)
    } else {
        integral2 = std::max(integral2, -maxIntegral); //same thing but negative max
    }
    
    derivative2 = error2 - prevError2;

    

    power2 = (vKp * error2) + (vKi * integral2) + (vKd * derivative2);

    return power2;
}


double calcPID3(double target, double input, int integralKi, int maxIntegral, bool slewOn = false) { //basically tuning i here
    int integral3;
    prevError3 = error3;
    error3 = target - input;
    
    if(std::abs(error3) < integralKi) {
        integral3 += error3;
    } else {
        integral3 = 0;
    }

    if(integral3 >= 0) {
        integral3 = std::min(integral3, maxIntegral); //min means take whichever value is smaller btwn integral and maxI
        //integral = integral until integral is greater than maxI (to keep integral limited to maxI)
    } else {
        integral3 = std::max(integral3, -maxIntegral); //same thing but negative max
    }
    
    derivative3 = error3 - prevError3;

    

    power3 = (vKp * error3) + (vKi * integral3) + (vKd * derivative3);

    return power3;
}

double calcPIDT(double target, double input, int integralKi, int maxIntegral, bool slewOn = false) { //basically tuning i here
    int integral4;
    prevError4 = error4;
    error4 = target - input;
    
    if(std::abs(error4) < integralKi) {
        integral4 += error4;
    } else {
        integral4 = 0;
    }

    if(integral4 >= 0) {
        integral4 = std::min(integral4, maxIntegral); //min means take whichever value is smaller btwn integral and maxI
        //integral = integral until integral is greater than maxI (to keep integral limited to maxI)
    } else {
        integral4 = std::max(integral4, -maxIntegral); //same thing but negative max
    }

    if(error4 * prevError4 <= 0){
        integral4 = 0;
    }
    
    derivative4 = error4 - prevError4;

    

    power4 = (vKp * error4) + (vKi * integral4) + (vKd * derivative4);

    return power4;
}







// void ColorSort(int color){
//     //blue color rejection
//     if (color == 0){
//         if(OpticalC.get_hue()<240 && OpticalC.get_hue()>180){
//             InitColor = true;
//         }

//         if (InitColor){
//             ColorCount += 1;
//         }

//         if(ColorCount > 105 && ColorCount < 800){
//             HOOKS.move(0);
//         } else {
//             HOOKS.move(127);
//         }
//         if(ColorCount>=800){
//             InitColor = false;
//             ColorCount = 0;
//         }


//     } else if (color == 1) { //red color rejectiom
//         if(OpticalC.get_hue()>0 && OpticalC.get_hue()<40){
//             InitColor = true;
//         } 

//         if (InitColor){
//             ColorCount += 1;
//         }

//         if(ColorCount > 110 && ColorCount < 800){
//             HOOKS.move(-30);
//         } else {
//             HOOKS.move(127);
//         }
//         if(ColorCount>=800){
//             InitColor = false;
//             ColorCount = 0;
//         }



// }
// }

/*
bool Backwards = false;
bool InitColor = false;
bool stalled = false;
int stallTime = 0;
int hookCount = 0;
double hookPos = 0;
double prevHook = 0;

void ColorSort(){
    //blue color rejection
    if (color == 1){
        if((OpticalC.get_hue()<240 && OpticalC.get_hue()>180) && OpticalC.get_proximity()>100){
            InitColor = true;
            HOOKS.tare_position();
        }

        if (InitColor){
            if(Backwards == false){
                HOOKS.move(127);
                if(HOOKS.get_position() > 500){
                    Backwards = true; 
                }
            } else {
                HOOKS.move(-127);
                if(HOOKS.get_position() < 200){
                    Backwards = false;
                    InitColor = false;
                }
            }
        } else {
            
            prevHook = hookPos;
            hookPos = HOOKS.get_position();
            if(prevHook==hookPos){
                hookCount++
            }

            if(hookCount>8){
                stalled = true;
            }

            if(stalled){
                HOOKS.move(-127);
                stallTime+=10;
                if(stallTime > 300){
                    hookCount = 0;
                    stalled = false;
                    stallTime = 0;
                }
            } else {
                HOOKS.move(127);
            }
            
        }

    } else if (color == 2) { //red color rejectiom
        if((OpticalC.get_hue()>330 || OpticalC.get_hue()<30) && OpticalC.get_proximity()>100){
            InitColor = true;
            HOOKS.tare_position();
        } 

        if (InitColor){
            if(Backwards == false){
                HOOKS.move(127);
                if(HOOKS.get_position() > 200){
                    Backwards = true; 
                }
            } else {
                HOOKS.move(-127);
                if(HOOKS.get_position() < -200){
                    Backwards = false;
                    InitColor = false;
                }
            }
        } else {
             
            prevHook = hookPos;
            hookPos = HOOKS.get_position();
            if(prevHook==hookPos){
                hookCount++
            }

            if(hookCount>8){
                stalled = true;
            }

            if(stalled){
                HOOKS.move(-127);
                stallTime+=10;
                if(stallTime > 300){
                    hookCount = 0;
                    stalled = false;
                    stallTime = 0;
                }
            } else {
                HOOKS.move(127);
            }

        }



}
}
*/

/*

bool Backwards = false;
bool InitColor = false;
void ColorSort(){
    //blue color rejection
    if (color == 1){
        if(OpticalC.get_hue()<240 && OpticalC.get_hue()>180){
            InitColor = true;
        }

        if (InitColor){
            if(Backwards == false){
                HOOKS.move(127);
                if(HOOKS.get_position() > 500){
                    Backwards = true; 
                }
            } else {
                HOOKS.move(-127);
                if(HOOKS.get_position() < 200){
                    Backwards = false;
                    InitColor = false;
                }
            }
        } else {
            HOOKS.move(127);
            HOOKS.tare_position();
        }

    } else if (color == 2) { //red color rejectiom
        if(OpticalC.get_hue() < 30 && OpticalC.get_hue()>330){
            InitColor = true;
        } 
        if (InitColor){
            if(Backwards == false){
                HOOKS.move(-127);
                if(OpticalC.get_hue()<240 && OpticalC.get_hue()>180){
                } else {
                }
                if(HOOKS.get_position() < -3000){
                    Backwards = true; 
                }
            } else {
                HOOKS.move(127);
                Backwards = false;
                InitColor = false;
            }
        } else if(InitCorrect){
            if(Backwards == false){
                HOOKS.move(127);
                if(OpticalC.get_hue()>0 && OpticalC.get_hue()<30){
                } else {
                }
                if(HOOKS.get_position() > 4000){
                    Backwards = true; 
                }
            } else {
                HOOKS.move(-127);
                Backwards = false;
                InitColor = false;
            }

        } else {
            HOOKS.move(127);
            HOOKS.tare_position();
        }
}
}

*/



//driving straight
void driveStraight(int target) {

    imu.tare();
    double voltage;
    double encoderAvg;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0; // Controller Display Cycle
    time2 = 0;

    if (init_heading > 180){
        init_heading = init_heading - 360;
    }

    int timeout = 30000;
    double x = 0;
    x = double(abs(target));

    //timeout = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0 * pow(x, 3)) + (0 * pow(x, 2)) + (0 * x) + 0; //Tune with Desmos
    
    resetEncoders();
    while(true) {
        if(abs(target - encoderAvg)<25){
            setConstants(2.5, 0, 0);
        } else {
            setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        }

        encoderAvg = (LF.get_position() + RF.get_position()) / 2;
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        

        double position = imu.get_heading(); //this is where the units are set to be degrees

        if (position > 180){
            position = position - 360;
        }

        if((init_heading < 0) && (position > 0)){
            if((position - init_heading) >= 180){
                init_heading = init_heading + 360;
                position = imu.get_heading();
            } 
        } else if ((init_heading > 0) && (position < 0)){
            if((init_heading - position) >= 180){
            position = imu.get_heading();
            }
        } 



        // if(longValues){
        //     setConstants(HEADING_KP2, HEADING_KI2, HEADING_KD2);
        // } else {
        //     setConstants(HEADING_KP, HEADING_KI, HEADING_KD);
        // }
         setConstants(HEADING_KP, HEADING_KI, HEADING_KD);

        
        heading_error = calcPID2(init_heading, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);


        if(voltage > 127){
            voltage = 127;
        } else if (voltage < -127){
            voltage = -127;
        }
        errorp = abs(target - encoderAvg);
        heading_error = 0;
        chasMove((voltage + heading_error ), (voltage - heading_error));
        if (abs(target - encoderAvg) <= 2) count++;
        if (count >= 8 || time2 > timeout){
            break;
        } 

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "ERROR: %f           ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150 != 0){
            con.print(1, 0, "imu: %f           ", float(imu.get_heading()));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Time: %f        ", float(time2));
        } 


        delay(10);
        time2 += 10;
    }
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}


//driving straight BUT GLOBAL
void driveClamp(int target, int clampDistance, int speed) {
    int timeout = 30000;
    double x = 0;
    x = double(abs(target));
    timeout = ( 0.00000000000012321 * pow(x,5)) + (-0.000000000953264 * pow(x, 4)) + (0.00000271528 * pow(x, 3)) + (-0.00339918 * pow(x, 2)) + (2.12469 * x) + 409.43588; //Tune with Desmos

    bool over = false;
    double voltage;
    double encoderAvg;
    int count = 0;
    double heading_error = 0;
    int cycle = 0; // Controller Display Cycle
    time2 = 0;

    if(trueTarget > 180){
        trueTarget = trueTarget - 360;
    }

    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    resetEncoders();
   
    timeout = timeout*(0.375/((100.0-double(speed))/100.0));
    while(true) {

        if(abs(target - encoderAvg)<25){
            setConstants(2.5, 0, 0);
        } else {
            setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        }


    encoderAvg = (LF.get_position() + RF.get_position()) / 2;
    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);


    double position = imu.get_heading(); //this is where the units are set to be degrees

    if (position > 180){
        position = position - 360;
    }

    if((trueTarget < 0) && (position > 0)){
        if((position - trueTarget) >= 180){
            trueTarget = trueTarget + 360;
            position = imu.get_heading();
        } 
    } else if ((trueTarget > 0) && (position < 0)){
        if((trueTarget - position) >= 180){
           position = imu.get_heading();
        }
    } 


        // if(trueTarget > 180) {
        //     trueTarget = (360 - trueTarget);
        // }

        // if(imu.get_heading() < 180) {
        //     heading_error = trueTarget - imu.get_heading();
        // }
        // else {
        //     heading_error = ((360 - imu.get_heading()) - trueTarget);
        // }

        // heading_error = heading_error * HEADING_CORRECTION_KP;

        if(longValues){
            setConstants(HEADING_KP2, HEADING_KI2, HEADING_KD2);
        } else {
            setConstants(HEADING_KP, HEADING_KI, HEADING_KD);
        }

        heading_error = calcPID2(trueTarget, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);


        if(abs(error) < clampDistance){
            mogo.set_value(true);
        }

        if(voltage > 127 * double(speed)/100.0){
            voltage = 127 * double(speed)/100.0;
        } else if (voltage < -127 * double(speed)/100.0){
            voltage = -127 * double(speed)/100.0;
        }



        chasMove( (voltage + heading_error ), (voltage - heading_error));
        if (abs(target - encoderAvg) <= 4) count++;
        if (count >= 20 || time2 > timeout){
            break;
        } 


        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "ERROR: %f           ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150 != 0){
            con.print(1, 0, "IMU: %f           ", float(imu.get_heading()));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Time: %f        ", float(timeout));
        } 

        delay(10);
        time2 += 10;
        //hi
    }
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}

void driveClampD(int target, int clampDistance, int intakeDistance, int speed) {
    int timeout = 30000;
    double x = 0;
    x = double(abs(target));
    timeout = ( 0.00000000000012321 * pow(x,5)) + (-0.000000000953264 * pow(x, 4)) + (0.00000271528 * pow(x, 3)) + (-0.00339918 * pow(x, 2)) + (2.12469 * x) + 9.43588; //Tune with Desmos

    double voltage;
    double encoderAvg;
    int count = 0;
    double heading_error = 0;
    time2 = 0;

    timeout = timeout*(1.0/((100.0-double(speed))/100.0));

    if(trueTarget > 180){
        trueTarget = trueTarget - 360;
    }

    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    resetEncoders();
   

    while(true) {

        if(abs(error)< intakeDistance){
            HOOKS.move(-127);
        }

        if(abs(target - encoderAvg)<25){
            setConstants(2.5, 0, 0);
        } else {
            setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        }


    encoderAvg = (LF.get_position() + RF.get_position()) / 2;
    voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);


    double position = imu.get_heading(); //this is where the units are set to be degrees

    if (position > 180){
        position = position - 360;
    }

    if((trueTarget < 0) && (position > 0)){
        if((position - trueTarget) >= 180){
            trueTarget = trueTarget + 360;
            position = imu.get_heading();
        } 
    } else if ((trueTarget > 0) && (position < 0)){
        if((trueTarget - position) >= 180){
           position = imu.get_heading();
        }
    } 


        // if(trueTarget > 180) {
        //     trueTarget = (360 - trueTarget);
        // }

        // if(imu.get_heading() < 180) {
        //     heading_error = trueTarget - imu.get_heading();
        // }
        // else {
        //     heading_error = ((360 - imu.get_heading()) - trueTarget);
        // }

        // heading_error = heading_error * HEADING_CORRECTION_KP;

        if(longValues){
            setConstants(HEADING_KP2, HEADING_KI2, HEADING_KD2);
        } else {
            setConstants(HEADING_KP, HEADING_KI, HEADING_KD);
        }



        heading_error = calcPID2(trueTarget, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);


        if(abs(error) < clampDistance){
            doinkerClamp.set_value(true);
        }

        if(voltage > 127 * double(speed)/100.0){
            voltage = 127 * double(speed)/100.0;
        } else if (voltage < -127 * double(speed)/100.0){
            voltage = -127 * double(speed)/100.0;
        }



        chasMove( (voltage + heading_error ), (voltage - heading_error));
        if (abs(target - encoderAvg) <= 4) count++;
        if (count >= 20 || time2 > timeout){
            break;
        } 


        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "ERROR: %f           ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150 != 0){
            con.print(1, 0, "EncoderAvg: %f           ", float(encoderAvg));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Time: %f        ", float(time2));
        } 

        delay(10);
        time2 += 10;
        //hi
    }
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}


void driveStraight2(int target, int speed) {
    int timeout = 5000;
    double x = 0;
    x = double(abs(target));
    timeout = ( 0.00000000000012321 * pow(x,5)) + (-0.000000000953264 * pow(x, 4)) + (0.00000271528 * pow(x, 3)) + (-0.00339918 * pow(x, 2)) + (2.12469 * x) + 409.43588; //Tune with Desmos

    double voltage;
    double encoderAvg;
    int count = 0;
    double heading_error = 0;
    int cycle = 0; // Controller Display Cycle
    time2 = 0;


    if(trueTarget > 180){
        trueTarget = trueTarget - 360;
    }

    if(mogoValues == true){
        setConstants(STRAIGHT_KPM, STRAIGHT_KIM, STRAIGHT_KDM);
    } else {
        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    }

    timeout = timeout * (2.0 - double(speed)/100.0);
    //timeout = timeout*(0.35/((100.0-double(speed))/100.0));
    
    resetEncoders();
   

    while(true) {

    if(abs(target - encoderAvg)<25){
        setConstants(2.5, 0, 0);
    } else {
        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    }
  

    encoderAvg = (LF.get_position() + RF.get_position()) / 2;
    voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);


    double position = imu.get_heading(); //this is where the units are set to be degrees

    if (position > 180){
        position = position - 360;
    }

    if((trueTarget < 0) && (position > 0)){
        if((position - trueTarget) >= 180){
            trueTarget = trueTarget + 360;
            position = imu.get_heading();
        } 
    } else if ((trueTarget > 0) && (position < 0)){
        if((trueTarget - position) >= 180){
           position = imu.get_heading();
        }
    } 


        // if(trueTarget > 180) {
        //     trueTarget = (360 - trueTarget);
        // }

        // if(imu.get_heading() < 180) {
        //     heading_error = trueTarget - imu.get_heading();
        // }
        // else {
        //     heading_error = ((360 - imu.get_heading()) - trueTarget);
        // }

        // heading_error = heading_error * HEADING_CORRECTION_KP;

        if(longValues){
            setConstants(HEADING_KP2, HEADING_KI2, HEADING_KD2);
        } else {
            setConstants(HEADING_KP, HEADING_KI, HEADING_KD);
        }

        heading_error = calcPID2(trueTarget, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);
        

        if(voltage > 127 * double(speed)/100.0){
            voltage = 127 * double(speed)/100.0;
        } else if (voltage < -127 * double(speed)/100.0){
            voltage = -127 * double(speed)/100.0;
        }

        chasMove( (voltage + heading_error ), (voltage - heading_error));
        if (abs(target - encoderAvg) <= 4) count++;
        if (count >= 8 || time2 > timeout){
            break;
        } 


        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "ERROR: %f           ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150 != 0){
            con.print(1, 0, "true: %f           ", float(trueTarget));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Time: %f        ", float(time2));
        } 

        delay(10);
        time2 += 10;
        //hi
    }
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}


void driveStraightI(int target, int speed, int dist) {
    int timeout = 5000;
    double x = 0;
    x = double(abs(target));
    timeout = ( 0.00000000000012321 * pow(x,5)) + (-0.000000000953264 * pow(x, 4)) + (0.00000271528 * pow(x, 3)) + (-0.00339918 * pow(x, 2)) + (2.12469 * x) + 409.43588; //Tune with Desmos

    double voltage;
    double encoderAvg;
    int count = 0;
    double heading_error = 0;
    int cycle = 0; // Controller Display Cycle
    time2 = 0;


    if(trueTarget > 180){
        trueTarget = trueTarget - 360;
    }

    if(mogoValues == true){
        setConstants(STRAIGHT_KPM, STRAIGHT_KIM, STRAIGHT_KDM);
    } else {
        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    }

    timeout = timeout * (2.0 - double(speed)/100.0);
    //timeout = timeout*(0.35/((100.0-double(speed))/100.0));
    
    resetEncoders();
   

    while(true) {

    if(abs(error) < dist){
        HOOKS.move(-127);
    } else {
        HOOKS.move(0);
    }
    if(abs(target - encoderAvg)<25){
        setConstants(2.5, 0, 0);
    } else {
        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    }
  

    encoderAvg = (LF.get_position() + RF.get_position()) / 2;
    voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);


    double position = imu.get_heading(); //this is where the units are set to be degrees

    if (position > 180){
        position = position - 360;
    }

    if((trueTarget < 0) && (position > 0)){
        if((position - trueTarget) >= 180){
            trueTarget = trueTarget + 360;
            position = imu.get_heading();
        } 
    } else if ((trueTarget > 0) && (position < 0)){
        if((trueTarget - position) >= 180){
           position = imu.get_heading();
        }
    } 


        // if(trueTarget > 180) {
        //     trueTarget = (360 - trueTarget);
        // }

        // if(imu.get_heading() < 180) {
        //     heading_error = trueTarget - imu.get_heading();
        // }
        // else {
        //     heading_error = ((360 - imu.get_heading()) - trueTarget);
        // }

        // heading_error = heading_error * HEADING_CORRECTION_KP;

        if(longValues){
            setConstants(HEADING_KP2, HEADING_KI2, HEADING_KD2);
        } else {
            setConstants(HEADING_KP, HEADING_KI, HEADING_KD);
        }

        heading_error = calcPID2(trueTarget, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);
        

        if(voltage > 127 * double(speed)/100.0){
            voltage = 127 * double(speed)/100.0;
        } else if (voltage < -127 * double(speed)/100.0){
            voltage = -127 * double(speed)/100.0;
        }

        chasMove( (voltage + heading_error ), (voltage - heading_error));
        if (abs(target - encoderAvg) <= 4) count++;
        if (count >= 8 || time2 > timeout){
            break;
        } 


        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "ERROR: %f           ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150 != 0){
            con.print(1, 0, "true: %f           ", float(trueTarget));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Time: %f        ", float(time2));
        } 

        delay(10);
        time2 += 10;
        //hi
    }
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}


void driveStraightR(int target, int speed) {
    int timeout = 5000;
    double x = 0;
    x = double(abs(target));
    timeout = ( 0.00000000000012321 * pow(x,5)) + (-0.000000000953264 * pow(x, 4)) + (0.00000271528 * pow(x, 3)) + (-0.00339918 * pow(x, 2)) + (2.12469 * x) + 209.43588; //Tune with Desmos

    bool over = false;
    double voltage;
    double encoderAvg;
    int count = 0;
    double heading_error = 0;
    int cycle = 0; // Controller Display Cycle
    time2 = 0;

    if(trueTarget > 180){
        trueTarget = trueTarget - 360;
    }

    if(mogoValues == true){
        setConstants(STRAIGHT_KPM, STRAIGHT_KIM, STRAIGHT_KDM);
    } else {
        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    }

    timeout = timeout*(1.0/((100.0-double(speed))/100.0));
    
    resetEncoders();
   

    while(true) {

    if(abs(target - encoderAvg)<25){
        setConstants(2.5, 0, 0);
    } else {
        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    }

    // if(OpticalC.get_hue()<16 || OpticalC.get_hue()>350){
    //     HOOKS.move(0);
    // } else {
    //     HOOKS.move(-127);
    // }

    if(OpticalC.get_proximity()>100){
        HOOKS.move(0);
    } else {
        HOOKS.move(-127);
    }
  

    encoderAvg = (LF.get_position() + RF.get_position()) / 2;
    


    voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);


    double position = imu.get_heading(); //this is where the units are set to be degrees

    if (position > 180){
        position = position - 360;
    }

    if((trueTarget < 0) && (position > 0)){
        if((position - trueTarget) >= 180){
            trueTarget = trueTarget + 360;
            position = imu.get_heading();
        } 
    } else if ((trueTarget > 0) && (position < 0)){
        if((trueTarget - position) >= 180){
           position = imu.get_heading();
        }
    } 


        // if(trueTarget > 180) {
        //     trueTarget = (360 - trueTarget);
        // }

        // if(imu.get_heading() < 180) {
        //     heading_error = trueTarget - imu.get_heading();
        // }
        // else {
        //     heading_error = ((360 - imu.get_heading()) - trueTarget);
        // }

        // heading_error = heading_error * HEADING_CORRECTION_KP;

        if(longValues){
            setConstants(HEADING_KP2, HEADING_KI2, HEADING_KD2);
        } else {
            setConstants(HEADING_KP, HEADING_KI, HEADING_KD);
        }

        heading_error = calcPID2(trueTarget, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);
        

        if(voltage > 127 * double(speed)/100.0){
            voltage = 127 * double(speed)/100.0;
        } else if (voltage < -127 * double(speed)/100.0){
            voltage = -127 * double(speed)/100.0;
        }

        chasMove( (voltage + heading_error ), (voltage - heading_error));
        if (abs(target - encoderAvg) <= 4) count++;
        if (count >= 8 || time2 > timeout){
            break;
        } 


        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "ERROR: %f           ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150 != 0){
            con.print(1, 0, "EncoderAvg: %f           ", float(encoderAvg));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Time: %f        ", float(time2));
        } 

        delay(10);
        time2 += 10;
        //hi
    }
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}

// drive stright BUT GLOBAL AND WITH CHAINING (ends with velocity)
void driveStraightC(int target) {
    bool over = false;
    int timeout = 30000;
    double x = 0;
    x = double(abs(target));
    timeout = ( 0.00000000000012321 * pow(x,5)) + (-0.000000000953264 * pow(x, 4)) + (0.00000271528 * pow(x, 3)) + (-0.00339918 * pow(x, 2)) + (2.12469 * x) + 409.43588; //Tune with Desmos

    if(target > 0){
        target = target + 500;
    } else {
        target = target - 500;
    }

    double voltage;
    double encoderAvg;
    int count = 0;
    double heading_error = 0;
    int cycle = 0; // Controller Display Cycle
    time2 = 0;

    if(trueTarget > 180){
        trueTarget = trueTarget - 360;
    }

    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    resetEncoders();

    // timeout = timeout*(1.0/(100.0-double(speed)));

    while(true) {
        encoderAvg = (LF.get_position() + RF.get_position()) / 2;
        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

    
        double position = imu.get_heading(); //this is where the units are set to be degrees

        if (position > 180){
            position = position - 360;
        }

        if((trueTarget < 0) && (position > 0)){
            if((position - trueTarget) >= 180){
                trueTarget = trueTarget + 360;
                position = imu.get_heading();
            } 
        } else if ((trueTarget > 0) && (position < 0)){
            if((trueTarget - position) >= 180){
            position = imu.get_heading();
            }
        } 
    

        // if(trueTarget > 180) {
        //     trueTarget = (360 - trueTarget);
        // }

        // if(imu.get_heading() < 180) {
        //     heading_error = trueTarget - imu.get_heading();
        // }
        // else {
        //     heading_error = ((360 - imu.get_heading()) - trueTarget);
        // }

        setConstants(HEADING_KP, HEADING_KI, HEADING_KD);
        heading_error = calcPID2(trueTarget, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

        if(voltage > 127){
            voltage = 127;
        } else if (voltage < -127){
            voltage = -127;
        }

        chasMove((voltage + heading_error ), (voltage - heading_error));
        if (target > 0){
            if ((encoderAvg - (target-500)) > 0){
                over = true;
            }
        } else {
             if (((target+500) - encoderAvg) > 0){
                over = true;
            }
        }

        if (over || time2 > timeout){
          break;
        } 

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "ERROR: %f           ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150 != 0){
            con.print(1, 0, "EncoderAvg: %f           ", float(encoderAvg));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Time: %f        ", float(time2));
        } 

        delay(10);
        time2 += 10;
        //hi
    }
}



//Turning
void driveTurn(int target) { //target is inputted in autons
    double voltage;
    double position;
    int count = 0;
    time2 = 0;


    setConstants(TURN_KP, TURN_KI, TURN_KD);

    int timeout = 30000;
    double variKP = 0;
    double variKD = 0;
    double x = 0;

    x = double(abs(target));
   // variKP = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0 * pow(x, 3)) + (0 * pow(x, 2)) + (0 * x) + 0; // Use Desmos to tune
   variKD = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0 * pow(x, 3)) + (0 * pow(x, 2)) + (0 * x) + 0; // Use Desmos to tune
   if(mogoValues){
    //variKD =(-0.0000000042528 * pow(x,5)) + (0.00000209186 * pow(x, 4)) + (-0.000381218 * pow(x, 3)) + (0.0314888 * pow(x, 2)) + (-0.951821 * x) + 87.7549; // Use Desmos to tune
    variKD =(0 * pow(x,5)) + (0 * pow(x, 4)) + (0 * pow(x, 3)) + (0 * pow(x, 2)) + (0 * x) + 0; // Use Desmos to tune
   } 
    timeout = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0 * pow(x, 3)) + (0 * pow(x, 2)) + (0 * x) + 0; // Use Desmos to tune
    // if(abs(target>=25)){
    // setConstants(TURN_KP, TURN_KI, variKD); 
    // } else if(mogoValues == false) {
    // setConstants(5, TURN_KI, 90); 
    // }

    //setConstants(variKP, TURN_KI, variKD);
    setConstants(TURN_KP, TURN_KI, TURN_KD); 
    imu.tare_heading();

    while(true) {
        position = imu.get_heading(); //this is where the units are set to be degrees

        if (position > 180) {
            position = position - 360;
        }

        if(abs(error)<= 2){
            setConstants(11, 0, 0);
        } else {
            setConstants(TURN_KP, TURN_KI, TURN_KD); 
        }

        voltage = calcPID(target, position, TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL);
        
        chasMove(voltage, -voltage);
        if (fabs(target - position) <= 0.5) count++; 
        if (count >= 20 || time2 > timeout) {
          break; 
        }

        
        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "ERROR: %f           ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150 != 0){
            con.print(1, 0, "IMU: %f           ", float(imu.get_heading()));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Time: %f        ", float(time2));
        } 

        time2 += 10;
        delay(10);
    }
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}



//Turning BUT GLOBAL
void driveTurn2(int target) { //target is inputted in autons

    trueTarget = target;
    double voltage;
    double position;
    int count = 0;
    time2 = 0;
    int cycle = 0;
    int turnv = 0;


    position = imu.get_heading(); //this is where the units are set to be degrees

    if (position > 180){
        position = position - 360;
    }

    if((target < 0) && (position > 0)){
        if((position - target) >= 180){
            target = target + 360;
            position = imu.get_heading();
            turnv = (target - position); // target + position
        } else {
             turnv = (abs(position) + abs(target));
        }
    } else if ((target > 0) && (position < 0)){
        if((target - position) >= 180){
            position = imu.get_heading();
            turnv = abs(abs(position) - abs(target));
        } else {
            turnv = (abs(position) + target);
        }
    } else {
         turnv = abs(abs(position) - abs(target));
    }

    //fortnite - derrick

    double variKP = 0;
    double x = 0;
    double variKD = 0;
    int timeout = 5000;

    x = double(abs(turnv));
   // variKP = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0 * pow(x, 3)) + (0 * pow(x, 2)) + (0 * x) + 0; // Use Desmos to tune
   //variKD = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0 * pow(x, 3)) + (0 * pow(x, 2)) + (0 * x) + 0; // Use Desmos to tune
   //if(mogoValues){
    //variKD =(-0.0000000042528 * pow(x,5)) + (0.00000209186 * pow(x, 4)) + (-0.000381218 * pow(x, 3)) + (0.0314888 * pow(x, 2)) + (-0.951821 * x) + 87.7549; // Use Desmos to tune
    variKD =(0.0000000033996 * pow(x,5)) + (-0.00000144663 * pow(x, 4)) + (0.000207591 * pow(x, 3)) + (-0.0111654 * pow(x, 2)) + (0.209467 * x) + 53.04069; // Use Desmos to tune
   //} 
    timeout = (0.00000000392961 * pow(x,5)) + (0.0000057915 * pow(x, 4)) + (-0.00321553 * pow(x, 3)) + (0.502982 * pow(x, 2)) + (-22.36692 * x) + 766.53481; //866 // Use Desmos to tune 
    if(atn == 8){
        timeout -= 300;
    }
    if(atn == 6 || atn == 5){
        timeout -= 200;
    }
    if(atn == 1 || atn == 2){
        timeout -= 350;
    }
    // if(abs(target>=25)){
    // setConstants(TURN_KP, TURN_KI, variKD); 
    // } else if(mogoValues == false) {
    // setConstants(5, TURN_KI, 90); 
    // }

   // variKD = (0*pow(x,5)) +(0*pow(x,4)) + (0*pow(x,3)) + (0*pow(x,2)) + (0*x) + 0;

   
    //setConstants(variKP, TURN_KI, variKD);
    setConstants(TURNT_KP, TURN_KI, variKD); 



    while(true) {
        position = imu.get_heading(); 

        if (position > 180){
            position = ((360 - position) * -1 );
        }

        if((target < 0) && (position > 0)){
            if((position - target) >= 180){
                target = target + 360;
                position = imu.get_heading();
                turnv = (target - position); 
            } else {
                turnv = (abs(position) + abs(target));
            }
        } else if ((target > 0) && (position < 0)){
            if((target - position) >= 180){
            position = imu.get_heading();
                turnv = abs(abs(position) - abs(target));
            } else {
                turnv = (abs(position) + target);
            }
        } else {
            turnv = abs(abs(position) - abs(target));
        }

        if(abs(error)<= 1){
            setConstants(15, 0, 0);
        } else if(abs(error)<=2){
            setConstants(9, 0, 0);
        } else {
            setConstants(TURN_KP, TURN_KI, variKD); 
        }


     

        voltage = calcPID(target, position, TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL);

        
        chasMove(voltage, -voltage);
        
        if (abs(target - position) <= 0.5) count++; //0.35
        if (count >= 20 || time2 > timeout) {
           break; 
        }

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "turnv: %f           ", float(turnv));
        } else if (time2 % 100 == 0 && time2 % 150 != 0){
            con.print(1, 0, "IMU: %f           ", float(imu.get_heading()));
        } else if (time2 % 150 == 0){
         con.print(2, 0, "error %f        ", float(error));
        } 

        time2 += 10;
        delay(10);
    }
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}

void driveTurnD(int target) { //target is inputted in autons

    trueTarget = target;
    double voltage;
    double position;
    int count = 0;
    time2 = 0;
    int cycle = 0;
    int turnv = 0;



    position = imu.get_heading(); //this is where the units are set to be degrees

    if (position > 180){
        position = position - 360;
    }

    if((target < 0) && (position > 0)){
        if((position - target) >= 180){
            target = target + 360;
            position = imu.get_heading();
            turnv = (target - position); // target + position
        } else {
             turnv = (abs(position) + abs(target));
        }
    } else if ((target > 0) && (position < 0)){
        if((target - position) >= 180){
            position = imu.get_heading();
            turnv = abs(abs(position) - abs(target));
        } else {
            turnv = (abs(position) + target);
        }
    } else {
         turnv = abs(abs(position) - abs(target));
    }

    //fortnite - derrick

    double variKP = 0;
    double x = 0;
    double variKD = 0;
    int timeout = 5000;

    x = double(abs(turnv));
   // variKP = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0 * pow(x, 3)) + (0 * pow(x, 2)) + (0 * x) + 0; // Use Desmos to tune
   //variKD = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0 * pow(x, 3)) + (0 * pow(x, 2)) + (0 * x) + 0; // Use Desmos to tune
   //if(mogoValues){
    //variKD =(-0.0000000042528 * pow(x,5)) + (0.00000209186 * pow(x, 4)) + (-0.000381218 * pow(x, 3)) + (0.0314888 * pow(x, 2)) + (-0.951821 * x) + 87.7549; // Use Desmos to tune
    variKD =(0.0000000033996 * pow(x,5)) + (-0.00000144663 * pow(x, 4)) + (0.000207591 * pow(x, 3)) + (-0.0111654 * pow(x, 2)) + (0.209467 * x) + 53.04069; // Use Desmos to tune
   //} 
    timeout = (0.00000000392961 * pow(x,5)) + (0.0000057915 * pow(x, 4)) + (-0.00321553 * pow(x, 3)) + (0.502982 * pow(x, 2)) + (-22.36692 * x) + 466.53481; // Use Desmos to tune
    // if(abs(target>=25)){
    // setConstants(TURN_KP, TURN_KI, variKD); 
    // } else if(mogoValues == false) {
    // setConstants(5, TURN_KI, 90); 
    // }

    //setConstants(variKP, TURN_KI, variKD);
    setConstants(TURNT_KP, TURN_KI, variKD); 



    while(true) {
        position = imu.get_heading(); 

        if (position > 180){
            position = ((360 - position) * -1 );
        }

        if((target < 0) && (position > 0)){
            if((position - target) >= 180){
                target = target + 360;
                position = imu.get_heading();
                turnv = (target - position); 
            } else {
                turnv = (abs(position) + abs(target));
            }
        } else if ((target > 0) && (position < 0)){
            if((target - position) >= 180){
            position = imu.get_heading();
                turnv = abs(abs(position) - abs(target));
            } else {
                turnv = (abs(position) + target);
            }
        } else {
            turnv = abs(abs(position) - abs(target));
        }

        if(abs(error)<= 1){
            setConstants(15, 0, 0);
        } else if(abs(error)<=2){
            setConstants(9, 0, 0);
        } else {
            setConstants(TURN_KP, TURN_KI, variKD); 
        }

        if(position < -50){
            doinker.set_value(false);
        }


     

        voltage = calcPID(target, position, TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL);

        
        chasMove(voltage, -voltage);
        
        if (abs(target - position) <= 0.5) count++; //0.35
        if (count >= 20 || time2 > timeout) {
           break; 
        }

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "turnv: %f           ", float(turnv));
        } else if (time2 % 100 == 0 && time2 % 150 != 0){
            con.print(1, 0, "IMU: %f           ", float(imu.get_heading()));
        } else if (time2 % 150 == 0){
         con.print(2, 0, "vari %f        ", float(variKD));
        } 

        time2 += 10;
        delay(10);
    }
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}
//Turning BUT GLOBAL
void driveTurnT(int target) { //target is inputted in autons
    trueTarget = target;
    double voltage;
    double position;
    int count = 0;
    time2 = 0;
    int cycle = 0;
    int turnv = 0;

    position = imu.get_heading(); //this is where the units are set to be degrees

    if (position > 180){
        position = position - 360;
    }

    if((target < 0) && (position > 0)){
        if((position - target) >= 180){
            target = target + 360;
            position = imu.get_heading();
            turnv = (target - position); // target + position
        } else {
             turnv = (abs(position) + abs(target));
        }
    } else if ((target > 0) && (position < 0)){
        if((target - position) >= 180){
            position = imu.get_heading();
            turnv = abs(abs(position) - abs(target));
        } else {
            turnv = (abs(position) + target);
        }

    } else {
         turnv = abs(abs(position) - abs(target));
    }

    //fortnite - derrick

    double variKP = 0;
    double x = 0;
    double variKD = 0;
    int timeout = 5000;


    x = double(abs(target));
   // variKP = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0 * pow(x, 3)) + (0 * pow(x, 2)) + (0 * x) + 0; // Use Desmos to tune
   variKD = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0 * pow(x, 3)) + (0 * pow(x, 2)) + (0 * x) + 0; // Use Desmos to tune
   if(mogoValues){
    //variKD =(-0.0000000042528 * pow(x,5)) + (0.00000209186 * pow(x, 4)) + (-0.000381218 * pow(x, 3)) + (0.0314888 * pow(x, 2)) + (-0.951821 * x) + 87.7549; // Use Desmos to tune
    variKD =(0 * pow(x,5)) + (0 * pow(x, 4)) + (0 * pow(x, 3)) + (0 * pow(x, 2)) + (0 * x) + 0; // Use Desmos to tune
   } 
    //timeout = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0 * pow(x, 3)) + (0 * pow(x, 2)) + (0 * x) + 0; // Use Desmos to tune
    // if(abs(target>=25)){
    // setConstants(TURN_KP, TURN_KI, variKD); 
    // } else if(mogoValues == false) {
    // setConstants(5, TURN_KI, 90); 
    // }

    //setConstants(variKP, TURN_KI, variKD);
    setConstants(TURNT_KP, TURNT_KI, TURNT_KD); 

    while(true) {
        position = imu.get_heading(); 

        if (position > 180){
            position = ((360 - position) * -1 );
        }

        if((target < 0) && (position > 0)){
            if((position - target) >= 180){
                target = target + 360;
                position = imu.get_heading();
                turnv = (target - position); 
            } else {
                turnv = (abs(position) + abs(target));
            }
        } else if ((target > 0) && (position < 0)){
            if((target - position) >= 180){
            position = imu.get_heading();
                turnv = abs(abs(position) - abs(target));
            } else {
                turnv = (abs(position) + target);
            }
        } else {
            turnv = abs(abs(position) - abs(target));
        }

        if(abs(error4)<= 2){
            setConstants(9.5, 0, 0);
        } else {
            setConstants(TURNT_KP, TURNT_KI, TURNT_KD); 
        }

        voltage = calcPIDT(target, position, TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL);

        
        chasMove(voltage, -voltage);
        
        if (abs(target - position) <= 0.5) count++; //0.35
        if (count >= 20 || time2 > timeout) {
           break; 
        }

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "ERROR: %f           ", float(error4));
        } else if (time2 % 100 == 0 && time2 % 150 != 0){
            con.print(1, 0, "IMU: %f           ", float(imu.get_heading()));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "mogoValues: %f        ", float(mogoValues));
        } 

        time2 += 10;
        delay(10);
    }
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}

void driveArcL(double theta, double radius, int timeout, int speed){
    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);


   // int timeout = 30000;

    double totalError = 0;

    double ltarget = 0;
    double rtarget = 0;
    double pi = 3.14159265359;
    int count = 0;
    time2 = 0;
    resetEncoders();
    con.clear();
    //int timeout = 5000;
    ltarget = double((theta / 360) * 2 * pi * radius); 
    rtarget = double((theta / 360) * 2 * pi * (radius + 455));
    while (true){
        double encoderAvgL = (LF.get_position() + LB.get_position()) / 2;
        double encoderAvgR = (RF.get_position() +  RB.get_position()) / 2;
        double leftcorrect = -(encoderAvgL * 360) / (2 * pi * radius);

        if(trueTarget > 180){
            trueTarget = trueTarget - 360;
        }

        double position = imu.get_heading(); //this is where the units are set to be degrees W

        if (position > 180){
            position = position - 360;
        }

        if(((trueTarget + leftcorrect)< 0) && (position > 0)){
            if((position - (trueTarget + leftcorrect)) >= 180){
                leftcorrect = leftcorrect + 360;
                position = imu.get_heading();
            } 
        } else if (((trueTarget + leftcorrect) > 0) && (position < 0)){
            if(((trueTarget + leftcorrect) - position) >= 180){
            position = imu.get_heading();
            }
        } 
    
        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        if(voltageL > 127 * double(speed)/100.0){
            voltageL = 127 * double(speed)/100.0;
        } else if (voltageL < -127 * double(speed)/100.0){
            voltageL = -127 * double(speed)/100.0;
        }


        int voltageR = calcPID2(rtarget, encoderAvgR, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        if(voltageR > 127 * double(speed)/100.0){
            voltageR = 127 * double(speed)/100.0;
        } else if (voltageR < -127 * double(speed)/100.0){
            voltageR = -127 * double(speed)/100.0;
        }

        //cole is just better. 8838D is king! :}
  

        setConstants(ARC_HEADING_KP, ARC_HEADING_KI, ARC_HEADING_KD);
        int fix = calcPID3((trueTarget + leftcorrect), position, ARC_HEADING_INTEGRAL_KI, ARC_HEADING_MAX_INTEGRAL);
        totalError += error3;
    
        chasMove((voltageL + fix), (voltageR - fix));
        if ((abs(ltarget - encoderAvgL) <= 4) && (abs(rtarget - encoderAvgR) <= 4)) count++;
        if (count >= 20 || time2 > timeout){
            trueTarget -= theta;
            break;
        } 

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "ERROR: %f           ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150 != 0){
            con.print(1, 0, "fix: %f           ", float(fix));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "imu: %f        ", float(imu.get_heading()));
        } 

        time2 += 10;
        delay(10);

    }
    
}


void driveArcLF(double theta, double radius, int timeout, int speed){
    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    int trueTheta = theta;
    double ltarget = 0;
    double rtarget = 0;
    double ltargetFinal = 0;
    double rtargetFinal = 0;
    double pi =  3.14159265359;
    bool over = false;
    int count = 0;
    int time = 0;
    resetEncoders();
    con.clear();



    //int timeout = 5000;
    ltargetFinal = double((theta / 360) * 2 * pi * radius); // * double(2) * pi * double(radius));
    rtargetFinal = double((theta / 360) * 2 * pi * (radius + 390));
    if(theta > 0){
        theta = theta + 45;
    } else {
        theta = theta - 45; 
    }
    ltarget = double((theta / 360) * 2 * pi * radius); // * double(2) * pi * double(radius));
    rtarget = double((theta / 360) * 2 * pi * (radius + 390));
    while (true){

        double encoderAvgL = (LF.get_position() + LM.get_position()) / 2;
        double encoderAvgR = (RB.get_position() +  RM.get_position()) / 2;
        double leftcorrect = -(encoderAvgL * 360) / (2 * pi * radius);

        if(trueTarget > 180){
            trueTarget = trueTarget - 360;
        }

        double position = imu.get_heading(); //this is where the units are set to be degrees W

        if (position > 180){
            position = position - 360;
        }

        if(((trueTarget + leftcorrect)< 0) && (position > 0)){
            if((position - (trueTarget + leftcorrect)) >= 180){
                leftcorrect = leftcorrect + 360;
                position = imu.get_heading();
            } 
        } else if (((trueTarget + leftcorrect) > 0) && (position < 0)){
            if(((trueTarget + leftcorrect) - position) >= 180){
            position = imu.get_heading();
            }
        } 
    


        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        // if(voltageL > 70){ //set left limit
        //     voltageL = 70;
        // } else if (voltageL < -70){
        //     voltageL = -70;
        // }

        if(voltageL > 127 * double(speed)/100.0){
            voltageL = 127 * double(speed)/100.0;
        } else if (voltageL < -127 * double(speed)/100.0){
            voltageL = -127 * double(speed)/100.0;
        }


        int voltageR = calcPID2(rtarget, encoderAvgR, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        if(voltageR > 127 * double(speed)/100.0){
            voltageR = 127 * double(speed)/100.0;
        } else if (voltageR < -127 * double(speed)/100.0){
            voltageR = -127 * double(speed)/100.0;
        }


        


        
    

        setConstants(ARC_HEADING_KP, ARC_HEADING_KI, ARC_HEADING_KD);
        int fix = calcPID3((trueTarget + leftcorrect), position, ARC_HEADING_INTEGRAL_KI, ARC_HEADING_MAX_INTEGRAL);

        chasMove((voltageL + fix), (voltageR - fix));

        // if (theta > 0){
        //     if ((encoderAvgL - ltargetFinal) > 0){
        //         over = true;
        //     }
        // } else {
        //     if ((ltargetFinal - encoderAvgL) > 0){
        //         over = true;
        //     }
        // }
        if(theta>0){
            if(abs((trueTarget - position)) > trueTheta){
                over = true;
            }
        } else {
            if((position-trueTarget) < -trueTheta){
                over = true;
            }
        }

        if (over || time > timeout){
            trueTarget -= trueTheta;
            break;
        } 

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "ERROR: %f           ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150 != 0){
            con.print(1, 0, "imu: %f           ", float(imu.get_heading()));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "l: %f        ", float( ltarget));
        } 

        time += 10;
        delay(10);

}
}

void driveArcR(double theta, double radius, int timeout, int speed){
    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    double ltarget = 0;
    double rtarget = 0;
    double pi =  3.14159265359;
    // if (trueTarget > 180){
    //     trueTarget = trueTarget - 360;
    // }

    int count = 0;
    time2 = 0;
    resetEncoders();
    con.clear();
    //int timeout = 5000;
    ltarget = double((theta / 360) * 2 * pi * (radius + 390)); // * double(2) * pi * double(radius));
    rtarget = double((theta / 360) * 2 * pi * (radius));
    while (true){
        double encoderAvgL = (LF.get_position() + LB.get_position()) / 2;
        double encoderAvgR = (RF.get_position() + RB.get_position()) / 2;
        double rightcorrect = (encoderAvgR * 360) / (2 * pi * radius);

        if(trueTarget > 180){
            trueTarget = trueTarget - 360;
        }

        double position = imu.get_heading(); //this is where the units are set to be degrees W

        if (position > 180){
            position = position - 360;
        }

        if(((trueTarget + rightcorrect) < 0) && (position > 0)){
            if((position - (trueTarget + rightcorrect)) >= 180){
                trueTarget = trueTarget + 360;
                position = imu.get_heading();
            } 
        } else if (((trueTarget + rightcorrect)> 0) && (position < 0)){
            if(((trueTarget + rightcorrect) - position) >= 180){
            position = imu.get_heading();
            }
        } 


        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        // if(voltageL > 100){ //set left limit
        //     voltageL = 100;
        // } else if (voltageL < -100){
        //     voltageL = -100;
        // }
        
        if(voltageL > 127 * double(speed)/100.0){
            voltageL = 127 * double(speed)/100.0;
        } else if (voltageL < -127 * double(speed)/100.0){
            voltageL = -127 * double(speed)/100.0;
        }

        int voltageR = calcPID2(rtarget, encoderAvgR, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        // if(voltageR > 70){ //set right limit
        //     voltageR = 70;
        // } else if (voltageR < -70){
        //     voltageR = -70;
        // }
        
        if(voltageR > 127 * double(speed)/100.0){
            voltageR = 127 * double(speed)/100.0;
        } else if (voltageR < -127 * double(speed)/100.0){
            voltageR = -127 * double(speed)/100.0;
        }



        setConstants(ARC_HEADING_KP, ARC_HEADING_KI, ARC_HEADING_KD);
        int fix = calcPID3((trueTarget + rightcorrect), position, ARC_HEADING_INTEGRAL_KI, ARC_HEADING_MAX_INTEGRAL);

        chasMove((voltageL + fix), (voltageR - fix));
        if ((abs(ltarget - encoderAvgL) <= 4) && (abs(rtarget - encoderAvgR) <= 4)) count++;
        if (count >= 20 || time2 > timeout){
            trueTarget += theta;
            break;
        } 

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "ERROR: %f           ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150 != 0){
            con.print(1, 0, "EncoderR: %f           ", float(encoderAvgR));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Time: %f        ", float(time2));
        } 

        time2 += 10;
        delay(10);
    }
}



void driveArcRF(double theta, double radius, int timeout, int speed){
    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    bool over = false;
    int trueTheta = theta;
    double ltarget = 0;
    double rtarget = 0;
    double ltargetFinal = 0;
    double rtargetFinal = 0;
    double pi =  3.14159265359;
    // if (trueTarget > 180){
    //     trueTarget farc= trueTarget - 360;
    // }
    int count = 0;
    int time = 0;
    double rightcorrect = 0;
    resetEncoders();
    con.clear();
    //int timeout = 5000;
    ltargetFinal = double((theta / 360) * 2 * pi * (radius+390)); // * double(2) * pi * double(radius));
    rtargetFinal = double((theta / 360) * 2 * pi * (radius));
    if(theta > 0){
        theta = theta + 45;
    } else{
        theta = theta - 45;
    }
    ltarget = double((theta / 360) * 2 * pi * (radius + 390)); // * double(2) * pi * double(radius));
    rtarget = double((theta / 360) * 2 * pi * (radius));

    while (true){
        if(trueTarget > 180){
            trueTarget = trueTarget - 360;
        }

        double position = imu.get_heading(); //this is where the units are set to be degrees W

        if (position > 180){
            position = position - 360;
        }
        double encoderAvgR = (RB.get_position() +  RM.get_position()) / 2;
        rightcorrect = (encoderAvgR * 360) / (2 * pi * radius);

        if(((trueTarget + rightcorrect) < 0) && (position > 0)){
            if((position - (trueTarget + rightcorrect)) >= 180){
                trueTarget = trueTarget + 360;
                position = imu.get_heading();
            } 
        } else if (((trueTarget + rightcorrect)> 0) && (position < 0)){
            if(((trueTarget + rightcorrect) - position) >= 180){
            position = imu.get_heading();
            }
        } 

        double encoderAvgL = LF.get_position();
        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if(voltageL > 127 * double(speed)/100.0){
            voltageL = 127 * double(speed)/100.0;
        } else if (voltageL < -127 * double(speed)/100.0){
            voltageL = -127 * double(speed)/100.0;
        }

        int voltageR = calcPID2(rtarget, encoderAvgR, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if(voltageR > 127 * double(speed)/100.0){
            voltageR = 127 * double(speed)/100.0;
        } else if (voltageR < -127 * double(speed)/100.0){
            voltageR = -127 * double(speed)/100.0;
        }


        
        setConstants(ARC_HEADING_KP, ARC_HEADING_KI, ARC_HEADING_KD);
        int fix = calcPID3((trueTarget + rightcorrect), position, ARC_HEADING_INTEGRAL_KI, ARC_HEADING_MAX_INTEGRAL);



        chasMove((voltageL + fix), (voltageR - fix));

        // if (theta > 0){
        //     if ((encoderAvgR - rtargetFinal) > 0){
        //         over = true;
        //     }
        // } else {
        //     if ((rtargetFinal - encoderAvgR) > 0){
        //         over = true;
        //     }
        // }

        if(theta>0){
            if(abs((trueTarget - position)) > trueTheta){
                over = true;
            }
        } else {
            if(abs((position-trueTarget)) > -trueTheta){
                over = true;
            }
        }


        if (over || time > timeout){
            trueTarget += trueTheta;
            break;
        } 

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "ERROR: %f           ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150 != 0){
            con.print(1, 0, "EncoderR: %f           ", float(encoderAvgR));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Time: %f        ", float(time2));
        } 

        time += 10;
        delay(10);

}
}

