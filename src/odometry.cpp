#include "api.h"
#include "main.h"
#include "pid.h"
#include "robot.h"
#include "auton.h"
#include "odometry.h"

using namespace pros;
using namespace std;

int turnv = 0;

double absoluteAngleToTarget = 0;
double position = 0;

double deltaX;
double deltaY;

double startingX;
double startingY;
double startingHeading;

double r0;
double r1;

double delta_left_encoder_pos;
double delta_right_encoder_pos;
double delta_center_encoder_pos;

double prev_left_encoder_pos;
double prev_right_encoder_pos;
double prev_center_encoder_pos;

double left_encoder_pos;
double right_encoder_pos;
double center_encoder_pos;

double localX;
double localY;

double local_polar_angle;
double local_polar_length;
double global_polar_angle;



double phi;

double prev_imu_pos;
double imu_pos;
double imu_pos_radians;

double x_pos;
double y_pos;

double pi = 3.14159265359;

int odo_time = 0;

void setPosition(float xcoord, float ycoord, float heading){
    startingX = xcoord;
    startingY = ycoord;
    
    startingHeading = heading;
  
    x_pos = startingX;
    y_pos = startingY;
}

void odometry (){

        prev_imu_pos = imu_pos;
        imu_pos = imu.get_rotation() + startingHeading;

        prev_left_encoder_pos   = left_encoder_pos;
        prev_right_encoder_pos  = right_encoder_pos;
        prev_center_encoder_pos = center_encoder_pos;

        left_encoder_pos   = LF.get_position();   // motor degrees
        right_encoder_pos  = RF.get_position();   // motor degrees
        center_encoder_pos = 0; // if no lateral wheel, keep 0

        delta_left_encoder_pos   = left_encoder_pos   - prev_left_encoder_pos;
        delta_right_encoder_pos  = right_encoder_pos  - prev_right_encoder_pos;
        delta_center_encoder_pos = center_encoder_pos - prev_center_encoder_pos;

        // phi in degrees (keep for debug), and in radians for math
        phi = imu_pos - prev_imu_pos; // degrees
        const double dtheta_rad = (pi * phi) / 180.0;
        const double th_rad     = (pi * imu_pos) / 180.0;

     const double EPS = (pi * IMU_THRESHOLD) / 180.0;       // small-angle threshold (convert degree threshold to radians)
    

        // compute local delta (robot frame)
        if (fabs(dtheta_rad) < EPS) {
          // straight-ish: use linearized form
          localX = (delta_left_encoder_pos + delta_right_encoder_pos) / 2.0;
          // NOTE: dtheta now radians
          localY = delta_center_encoder_pos - FORWARD_OFFSET * dtheta_rad;
        } else {
          // turning: use exact arc formulas (use radians!)
          const double avg_lr = (delta_left_encoder_pos + delta_right_encoder_pos) / 2.0;
          r0 = avg_lr / dtheta_rad;                    // forward radius
          r1 =  (delta_center_encoder_pos) / dtheta_rad; // lateral radius

          const double s = sin(dtheta_rad);
          const double c = cos(dtheta_rad);

          localX = r0 * s - r1 * (1.0 - c);
          localY = r1 * s + r0 * (1.0 - c);
        }

        // rotate into global frame using absolute heading (radians)
        deltaY =  localX * cos(th_rad) - localY * sin(th_rad);
        deltaX =  localX * sin(th_rad) + localY * cos(th_rad);

        x_pos += deltaX;
        y_pos += deltaY;

        if (odo_time % 50 == 0 && odo_time % 100 != 0 && odo_time % 150 != 0){
          con.print(0, 0, "x_pos: %f           ", float(x_pos));
        } else if (odo_time % 100 == 0 && odo_time % 150 != 0){
          con.print(1, 0, "y_pos: %f           ", float(y_pos));
        } else if (odo_time % 150 == 0){
          con.print(2, 0, "Pos: %f        ", float(position)); // FYI: 'position' never updates
        }

        odo_time += 10; // assumes caller delays ~10ms per loop
}

// void odometry2(){
//   prev_imu_pos = imu_pos;
//   imu_pos = imu.get_rotation() + startingHeading;
//   // if(abs(imu_pos - prev_imu_pos)<IMU_THRESHOLD){
//   //   imu_pos = prev_imu_pos;
//   // }
//   imu_pos_radians = (imu_pos*pi)/180;

        
//   prev_left_encoder_pos = left_encoder_pos;
//   prev_right_encoder_pos = right_encoder_pos;
//   prev_center_encoder_pos = center_encoder_pos;


//   left_encoder_pos = (ODOMY.get_position()/36000.0)*(2*pi);
//   right_encoder_pos =(ODOMY.get_position()/36000.0)*(2*pi);
//   center_encoder_pos = 0;
// //((ODOMX.get_position()/36000.0)*(2*pi) )
//  // + (imu.get_rotation()/360.0)*(2*pi*4.6)

//   delta_left_encoder_pos = left_encoder_pos - prev_left_encoder_pos;
//   delta_right_encoder_pos = right_encoder_pos - prev_right_encoder_pos;
//   delta_center_encoder_pos = center_encoder_pos - prev_center_encoder_pos;

//   // phi = (delta_left_encoder_pos - delta_right_encoder_pos) / trackwidth;

//   phi = imu_pos - prev_imu_pos;
//   phi = (pi*phi)/180;

//   if (phi <= IMU_THRESHOLD) {
//     localX = delta_center_encoder_pos;
//     localY = delta_right_encoder_pos;
//   } else {
//     localX = (2*sin(phi/2))*((delta_center_encoder_pos/phi)+FORWARD_OFFSET); 
//     localY = (2*sin(phi/2))*((delta_right_encoder_pos/phi)+SIDEWAYS_OFFSET);
//   }

  

//   if (localX == 0 && localY == 0){
//     local_polar_angle = 0;
//     local_polar_length = 0;
//   } else {
//     local_polar_angle = atan2(localY, localX); 
//     local_polar_length = sqrt(pow(localX, 2) + pow(localY, 2)); 
//   }

//   global_polar_angle = local_polar_angle - ((pi*prev_imu_pos)/180) - (phi/2);

//   deltaX = local_polar_length*cos(global_polar_angle); 
//   deltaY = local_polar_length*sin(global_polar_angle);

//   x_pos += deltaX;
//   y_pos += deltaY;

//   //  if (odo_time % 50 == 0 && odo_time % 100 != 0 && odo_time % 150 != 0){
//   //       con.print(0, 0, "x_pos: %f           ", float(x_pos));
//   //     } else if (odo_time % 100 == 0 && odo_time % 150 != 0){
//   //       con.print(1, 0, "y_pos: %f           ", float(y_pos));
//   //     } else if (odo_time % 150 == 0){
//   //       con.print(2, 0, "Pos: %f        ", float(center_encoder_pos));
//   //     } 

      

//   odo_time += 10;


   
// }

void odometry2() {
  // --- Tunables: set these for your hardware (units must match x_pos/y_pos) ---
  constexpr double TICKS_PER_REV_X = 36000.0;   // rotation sensor returns centidegrees -> 360 deg = 36000
  constexpr double TICKS_PER_REV_Y = 36000.0;
  constexpr double TRACK_RADIUS_X  = 1.0/* forward wheel radius */ ;
  constexpr double TRACK_RADIUS_Y  = 1.0/* lateral wheel radius */ ;
  constexpr double GEAR_X          = 1.0;       // wheel rev per sensor rev (change if geared)
  constexpr double GEAR_Y          = 1.0;

  const double DEG2RAD = pi / 180.0;
  const double THRESH  = IMU_THRESHOLD * DEG2RAD;   // keep IMU_THRESHOLD in degrees elsewhere

  // --- Heading change (IMU) ---
  prev_imu_pos = imu_pos;                                   // degrees
  imu_pos      = imu.get_rotation() + startingHeading;      // degrees
  double phi   = (imu_pos - prev_imu_pos) * DEG2RAD;        // radians

  // --- Previous wheel linear positions (we reuse your variable names) ---
  prev_left_encoder_pos   = left_encoder_pos;    // will represent X-wheel distance
  prev_right_encoder_pos  = right_encoder_pos;   // will represent Y-wheel distance
  prev_center_encoder_pos = 0;                   // unused in 2-wheel setup

  // --- Read rotation sensors -> linear distance (keep units consistent) ---
  double revX = (ODOMX.get_position() / TICKS_PER_REV_X) * GEAR_X; // rev
  double revY = (ODOMY.get_position() / TICKS_PER_REV_Y) * GEAR_Y; // rev
  left_encoder_pos  = revX * (2.0 * pi * TRACK_RADIUS_X);          // distance along body X
  right_encoder_pos = revY * (2.0 * pi * TRACK_RADIUS_Y);          // distance along body Y
  center_encoder_pos = 0.0;

  // --- Deltas measured by the tracking wheels ---
  double dXwheel = left_encoder_pos  - prev_left_encoder_pos;      // forward wheel delta (body X)
  double dYwheel = right_encoder_pos - prev_right_encoder_pos;     // lateral wheel delta (body Y)
  // If your Y wheel increases to the RIGHT instead of LEFT, do: dYwheel = -dYwheel;

  // --- Local (body-frame) displacement of the robot center ---
  double localX, localY;  // body X (forward), body Y (left)
  if (fabs(phi) < THRESH) {
    // Small-angle (linearized) update: add rotation-induced terms from offsets
    localX = dXwheel + FORWARD_OFFSET  * phi;   // FORWARD_OFFSET is +x offset of X wheel
    localY = dYwheel + SIDEWAYS_OFFSET * phi;   // SIDEWAYS_OFFSET is +y offset of Y wheel
  } else {
    // Curved update using mid-arc geometry
    double s = 2.0 * sin(phi / 2.0);
    localX = s * ((dXwheel / phi) + FORWARD_OFFSET);
    localY = s * ((dYwheel / phi) + SIDEWAYS_OFFSET);
  }

  // --- Rotate to global frame using mid-heading (better on turns) ---
  double theta_mid = (prev_imu_pos * DEG2RAD) + (phi / 2.0);
  double dx =  localX * cos(theta_mid) - localY * sin(theta_mid);
  double dy =  localX * sin(theta_mid) + localY * cos(theta_mid);

  x_pos += dx;
  y_pos += dy;

  // Optional: prints (kept minimal)
  // con.print(0, 0, "x_pos: %f   ", float(x_pos));
  // con.print(1, 0, "y_pos: %f   ", float(y_pos));

  odo_time += 10;
}



void driveToPoint (double xTarget, double yTarget, double perferredHeading){

  while(true){
    double turnv = 0;
    odometry2();
      double distanceToTarget = sqrt(pow((x_pos - xTarget),2) + pow((y_pos - yTarget),2));
      double absoluteAngleToTarget = atan2(pow((x_pos - xTarget),2), pow((y_pos - yTarget),2));

      double angleToTarget = absoluteAngleToTarget - (imu_pos - 90);

      while (angleToTarget >= 360) {
        angleToTarget = angleToTarget - 360;
      }

      while (angleToTarget <= -360){
        angleToTarget = angleToTarget + 360;
      }

      double relativeXToPoint = cos((pi*angleToTarget)/180) * distanceToTarget;
      double relativeYToPoint = sin((pi*angleToTarget)/180) * distanceToTarget;
      double relativeTurnAngle = angleToTarget - 180 + perferredHeading;

      double movementXPower = relativeXToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
      double movementYPower = relativeYToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
      
      //double movementTurn = clamp(((angleToTarget)/30), 1, -1);
      //I havent finished this yet 




  }


}



void boomerang(double xTarget, double yTarget){
  double hypot = 0;
  double voltage = 0;
  double heading_correction = 0;
  int btime = 0;
  int timeout = 30000;
  int count = 0;


  while(true){
    odometry2();
    hypot = sqrt(pow((x_pos - xTarget),2) + pow((y_pos - yTarget),2));
    absoluteAngleToTarget = atan2((xTarget - x_pos),(yTarget - y_pos)) * (180/pi);

    if (absoluteAngleToTarget > 180){
      absoluteAngleToTarget = absoluteAngleToTarget - 360;
    }

    position = imu.get_heading(); //this is where the units are set to be degrees

    if (position > 180){
            position = position - 360;
    }

    if((absoluteAngleToTarget < 0) && (position > 0)){
        if((position - absoluteAngleToTarget) >= 180){
            absoluteAngleToTarget = absoluteAngleToTarget + 360;
            position = imu.get_heading();
            turnv = (absoluteAngleToTarget - position); // target + position
        } else {
             turnv = (abs(position) + abs(absoluteAngleToTarget));
        }
    } else if ((absoluteAngleToTarget > 0) && (position < 0)){

        if((absoluteAngleToTarget - position) >= 180){
           position = imu.get_heading();
            turnv = abs(abs(position) - abs(absoluteAngleToTarget));
        } else {
            turnv = (abs(position) + absoluteAngleToTarget);
        }

    } else {
         turnv = abs(abs(position) - abs(absoluteAngleToTarget));
    }


        if(abs(turnv) > 90){
          absoluteAngleToTarget = absoluteAngleToTarget + 180;
          hypot = -hypot;
        }
        if (absoluteAngleToTarget >= 359){
            absoluteAngleToTarget = absoluteAngleToTarget - 360;
        }

        if((absoluteAngleToTarget < 0) && (position > 0)){
            if((position - absoluteAngleToTarget) >= 180){
                absoluteAngleToTarget = absoluteAngleToTarget + 360;
                position = imu.get_heading();
            } 
        } else if ((absoluteAngleToTarget > 0) && (position < 0)){
            if((absoluteAngleToTarget - position) >= 180){
            position = imu.get_heading();
            }
        } 
    setConstants(TURN_KP, TURN_KI, TURN_KD);
    heading_correction = calcPID(absoluteAngleToTarget, position, TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL, true);

    setConstants(STRAIGHT_KP*5, STRAIGHT_KI*5, STRAIGHT_KD*5);
    // if(abs(position - absoluteAngleToTarget) < 90){
    //   voltage = -calcPID2(0, hypot, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, true);
    // } else {
    //   voltage = calcPID2(0, hypot, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, true);
    // }
    voltage = -calcPID2(0, hypot, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, true);

    if(voltage > 127){
      voltage = 127;
    } else if(voltage < -127) {
      voltage = -127;
    }

    if(abs(hypot) < HEADING_CUTOFF){
      heading_correction = 0;
    }

    chasMove((voltage + heading_correction), (voltage - heading_correction));
    if(abs(hypot) < 1) count++;
    if((count > 20) || (btime > timeout)){
      break;
    }

       if (btime % 50 == 0 && btime % 100 != 0 && btime % 150 != 0){
        con.print(0, 0, "x_pos: %f           ", float(x_pos));
      } else if (btime % 100 == 0 && btime % 150 != 0){
        con.print(1, 0, "y_pos: %f           ", float(y_pos));
      } else if (btime % 150 == 0){
        con.print(2, 0, "Pos: %f        ", float(hypot));
      } 

    


    

  btime += 10;
  delay(10);

  }  

}