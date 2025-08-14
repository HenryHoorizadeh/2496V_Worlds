// /src/stanley.cpp  (add to PROS project and #include "stanley.hpp" where needed)
#include "main.h"
#include <algorithm>
#include <cmath>
#include <vector>

constexpr double kGain      = 500;      // Stanley gain (mm/s) – tune this
constexpr double wheelTrack = 300;     // Robot track width W (mm) – measure yours
constexpr double cg2axle    = wheelTrack / 2.0;  // center of the robot
constexpr double maxSpeed   = 1200;      // mm/s – cap for safety
constexpr double eps        = 1e-3;     // avoid divide‑by‑zero

struct Pose {
  double x;       // mm
  double y;
  double theta;   // rad, 0 = +x axis CCW
};

// Simple waypoint
struct Waypoint { double x, y; };

//basic motors for example code
pros::Motor leftMotors (1, pros::E_MOTOR_GEAR_200, true);
pros::Motor rightMotors(2, pros::E_MOTOR_GEAR_200, false);

Pose getPose() {
  // TODO: use odom and mcl here in order to get the robots position
  static Pose p{0,0,0};
  // update p here
  return p;
}

double getForwardVelocity() {
  // TODO: compute from wheel encoders (mm/s).
  //here you calcualte the speed of the bot in the units you would prefer, ex: mm/s, m/s, ft/s, in/s
  return 0.5;
}

// ****  STANLEY HELPERS  ****
double normAngle(double a) {
  while (a >  M_PI) a -= 2*M_PI;
  while (a < -M_PI) a += 2*M_PI;
  return a;
}

// Returns nearest point *on the segment* and the index of the segment start
std::pair<Pose,size_t> nearestPoint(const Pose& p, const std::vector<Waypoint>& path) {
  double bestDist2 = 1e9;
  Pose   bestPt;
  size_t idx = 0;
  for (size_t i=0;i+1<path.size();++i) {
    // project p onto segment (A,B)
    auto A = path[i]; auto B = path[i+1];
    double vx = B.x - A.x, vy = B.y - A.y;
    double wx = p.x - A.x, wy = p.y - A.y;
    double c1 = vx*wx + vy*wy;
    double c2 = vx*vx + vy*vy;
    double t  = std::clamp(c1/c2, 0.0, 1.0);
    Pose proj{A.x + t*vx, A.y + t*vy, 0};
    double d2 = std::pow(p.x-proj.x,2)+std::pow(p.y-proj.y,2);
    if (d2 < bestDist2) { bestDist2 = d2; bestPt = proj; idx = i; }
  }
  // Heading of path at nearest segment
  double dx = path[idx+1].x - path[idx].x;
  double dy = path[idx+1].y - path[idx].y;
  bestPt.theta = std::atan2(dy,dx);
  return {bestPt, idx};
}

// Main control step – returns left & right wheel linear velocities (m/s)
std::pair<double,double> stanleyStep(const Pose& robot, const std::vector<Waypoint>& path) {
  Pose target; size_t seg;
  std::tie(target, seg) = nearestPoint(robot, path);

  // Cross‑track sign: +ve if robot is left of path heading
  double headingVecX = std::cos(target.theta);
  double headingVecY = std::sin(target.theta);
  double dx = robot.x - target.x;
  double dy = robot.y - target.y;
  double cross = headingVecX*dy - headingVecY*dx; // 2D cross product z‑comp
  double ec = cross;  // already in meters

  // Heading error
  double theta_e = normAngle(target.theta - robot.theta);

  // Stanley law
  double v = std::clamp(getForwardVelocity(), 0.05, maxSpeed);
  double delta = theta_e + std::atan2(kGain * ec, v + eps);

  // Convert steering angle to curvature then wheel speeds
  double kappa = std::tan(delta) / cg2axle;
  double vLeft  = v * (1.0 - 0.5 * kappa * wheelTrack);
  double vRight = v * (1.0 + 0.5 * kappa * wheelTrack);

  // Clamp wheel speeds
  double maxW = std::max(std::fabs(vLeft), std::fabs(vRight));
  if (maxW > maxSpeed) {
    vLeft  *= maxSpeed / maxW;
    vRight *= maxSpeed / maxW;
  }
  return {vLeft, vRight};
}

// ****  TASK LOOP  ****
void stanleyTask(void*) {
  // Example 8‑ft straight then 90° turn (in meters)
  std::vector<Waypoint> path = {
    {0.0, 0.0}, {2.44, 0.0}, {2.44, 1.22}
  };

  while (true) {
    Pose pose = getPose();
    auto [vl, vr] = stanleyStep(pose, path);

    // Convert m/s to rpm  (rpm = (v / (2πR)) * 60)
    constexpr double wheelR = 0.0508;  // 2‑inch radius (4‑inch dia) in meters
    double rpmLeft  = vl / (2*M_PI*wheelR) * 60.0;
    double rpmRight = vr / (2*M_PI*wheelR) * 60.0;

    // leftMotors.move_velocity(rpmLeft);
    // rightMotors.move_velocity(rpmRight);

    pros::delay(10); // 10 ms
  }
}
