#include "main.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include "robot.h"

// ----- Constants and Globals -----
const int N_PARTICLES = 300;
const double FIELD_WIDTH = 3650.0; // mm (example field width between two side walls)
const double FIELD_LENGTH = 3650.0; // mm (example field length)
const double SENSOR_NOISE_STD = 50.0; // mm, sensor noise standard deviation
const double ODOMETRY_NOISE_STD = 5.0; // mm, translational noise per cycle
const double ROTATION_NOISE_STD = 1.0; // deg, rotational noise per cycle

struct Particle {
    double x;
    double y;
    double theta;
    double weight;
};

std::vector<Particle> particles;
std::default_random_engine rng;

// Sensors/Actuators objects (placeholder - create with actual ports)
pros::Imu imu_sensor(1);
pros::Distance dist_left(2);
pros::Distance dist_right(3);
// Assume motors or rotation sensors for tracking wheels:
pros::Motor drive_left(4, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor drive_right(5, pros::E_MOTOR_ENCODER_DEGREES);
// (If using separate tracking wheels with rotation sensors, use pros::Rotation)

double prev_left_enc = 0.0;
double prev_right_enc = 0.0;
double prev_imu_heading = 0.0;


// Initialize particles
void initializeParticles(double x0=0, double y0=0, double theta0=0, bool useGuess=false) {
    std::uniform_real_distribution<double> distX(0, FIELD_WIDTH);
    std::uniform_real_distribution<double> distY(0, FIELD_LENGTH);
    std::uniform_real_distribution<double> distTheta(0, 360);
    // Initialize around given guess
    std::normal_distribution<double> gaussX(x0, FIELD_WIDTH/10.0);
    std::normal_distribution<double> gaussY(y0, FIELD_LENGTH/10.0);
    std::normal_distribution<double> gaussTheta(theta0, 15.0);
    particles.resize(N_PARTICLES);
    for (int i = 0; i < N_PARTICLES; ++i) {
        if (useGuess) {  
            particles[i].x = gaussX(rng);
            particles[i].y = gaussY(rng);
            particles[i].theta = gaussTheta(rng);
        } else {
            // Global random initialization
            particles[i].x = distX(rng);
            particles[i].y = distY(rng);
            particles[i].theta = distTheta(rng);
        }
        // Bound positions within field
        if (particles[i].x < 0) particles[i].x = 0;
        if (particles[i].x > FIELD_WIDTH) particles[i].x = FIELD_WIDTH;
        if (particles[i].y < 0) particles[i].y = 0;
        if (particles[i].y > FIELD_LENGTH) particles[i].y = FIELD_LENGTH;
        particles[i].weight = 1.0 / N_PARTICLES;
    }
}

// Motion update: update particles based on odometry and IMU
double left_enc_deg = 0;// in degrees
double right_enc_deg = 0; // in degrees
double left_enc_mm = 0;
double right_enc_mm = 0;
double imu_heading = 0; // in degrees
void updateParticlesWithMotion() {
    // Update stored previous values
    prev_left_enc = left_enc_mm;
    prev_right_enc = right_enc_mm;
    prev_imu_heading = imu_heading;
    
    // Read encoders (convert degrees to mm traveled)
    left_enc_deg = drive_left.get_position(); // in degrees
    right_enc_deg = drive_right.get_position(); // in degrees
    left_enc_mm = left_enc_deg / 360.0 * (/*gear ratio*/36.0/48.0) * ( /* wheel circumference mm */ 320.0);
    right_enc_mm = right_enc_deg / 360.0 * (/*gear ratio*/36.0/48.0) * ( /* wheel circumference mm */ 320.0);
    imu_heading = imu.get_rotation(); // in degrees

    // Calculate changes
    double delta_left = left_enc_mm - prev_left_enc;
    double delta_right = right_enc_mm - prev_right_enc;
    double delta_heading = imu_heading - prev_imu_heading;
    // Wrap delta_heading to [-180, 180]
    if (delta_heading > 180) delta_heading -= 360;
    if (delta_heading < -180) delta_heading += 360;
    double delta_d = (delta_left + delta_right) / 2.0;



    // Prepare noise distributions
    std::normal_distribution<double> transNoise(0.0, ODOMETRY_NOISE_STD);
    std::normal_distribution<double> rotNoise(0.0, ROTATION_NOISE_STD);

    // Update each particle
    for (Particle& p : particles) {
        double noisy_d = delta_d + transNoise(rng);
        double noisy_theta = delta_heading + rotNoise(rng);
        // Update orientation
        p.theta += noisy_theta;
        if (p.theta < 0) p.theta += 360;
        if (p.theta >= 360) p.theta -= 360;
        // Update position
        double theta_rad = p.theta * M_PI / 180.0;
        p.x += noisy_d * cos(theta_rad);
        p.y += noisy_d * sin(theta_rad);
        // Keep particles within bounds (optional: you can reflect or clamp them)
        if (p.x < 0) p.x = 0;
        if (p.x > FIELD_WIDTH) p.x = FIELD_WIDTH;
        if (p.y < 0) p.y = 0;
        if (p.y > FIELD_LENGTH) p.y = FIELD_LENGTH;
    }
}

// Sensor update: update weights based on distance sensor readings
void updateParticlesWithSensor() {
    // Read distance sensors
    int32_t z_left = dist_left.get(); // mm
    int32_t z_right = dist_right.get(); // mm
    if (z_left < 0) z_left = FIELD_WIDTH; // if out of range, assume max
    if (z_right < 0) z_right = FIELD_WIDTH;

    // Update weights
    double weight_sum = 0.0;
    double var = SENSOR_NOISE_STD * SENSOR_NOISE_STD;
    for (Particle& p : particles) {
        // expected distances to walls based on particle position
        double pred_left = p.x;
        double pred_right = FIELD_WIDTH - p.x;
        // If using orientation, adjust predicted distances (omitted here for simplicity)
        double err_left = z_left - pred_left;
        double err_right = z_right - pred_right;
        // Likelihood assuming Gaussian noise
        double w_left = exp(-(err_left * err_left) / (2 * var));
        double w_right = exp(-(err_right * err_right) / (2 * var));
        p.weight = w_left * w_right;
        weight_sum += p.weight;
    }
    // Normalize weights
    if (weight_sum > 1e-9) {
        for (Particle& p : particles) {
            p.weight /= weight_sum;
        } 
    } else {
        // If total weight is zero (all particles have zero likelihood), reinitialize or assign equal weights
        for (Particle& p : particles) {
            p.weight = 1.0 / N_PARTICLES;
        }
    }
}

// Resample particles based on their weights (low-variance resampling)
void resampleParticles() {
    std::vector<Particle> newParticles;
    newParticles.resize(N_PARTICLES);
    // Calculate cumulative distribution of weights
    std::vector<double> cumw(N_PARTICLES);
    double cum = 0;
    for (int i = 0; i < N_PARTICLES; ++i) {
        cum += particles[i].weight;
        cumw[i] = cum;
    }
    // Ensure last cumw = 1 (normalized)
    if (cumw.back() == 0) {
        // All weights zero (shouldn't happen after normalization unless weight_sum was 0)
        for (int i = 0; i < N_PARTICLES; ++i) {
            cumw[i] = (i+1) / (double)N_PARTICLES;
        }
    }

    std::uniform_real_distribution<double> dist(0.0, 1.0 / N_PARTICLES);
    double start = dist(rng);
    double step = 1.0 / N_PARTICLES;
    double ptr = start;
    int j = 0;
    for (int i = 0; i < N_PARTICLES; ++i) {
        while (j < N_PARTICLES && cumw[j] < ptr) {
            j++;
        }
        if (j == N_PARTICLES) j = N_PARTICLES - 1;
        newParticles[i] = particles[j];
        newParticles[i].weight = 1.0 / N_PARTICLES; // reset weight
        ptr += step;
    }
    particles = newParticles;

    // (Optional) add random particles or noise to particles for robustness
    // e.g., add one random particle each time for kidnapped robot recovery:
    std::uniform_real_distribution<double> distX(0, FIELD_WIDTH);
    std::uniform_real_distribution<double> distY(0, FIELD_LENGTH);
    std::uniform_real_distribution<double> distTheta(0, 360);
    particles[0].x = distX(rng);
    particles[0].y = distY(rng);
    particles[0].theta = distTheta(rng);
    particles[0].weight = 1.0 / N_PARTICLES; 
}

// Compute the estimated pose (weighted average)
Particle getEstimatedPose() {
    Particle estimate;
    double mean_x = 0;
    double mean_y = 0;
    double mean_sin = 0;
    double mean_cos = 0;
    for (const Particle& p : particles) {
        mean_x += p.x * p.weight;
        mean_y += p.y * p.weight;
        mean_cos += cos(p.theta * M_PI / 180.0) * p.weight;
        mean_sin += sin(p.theta * M_PI / 180.0) * p.weight;
    }
    estimate.x = mean_x;
    estimate.y = mean_y;
    estimate.theta = atan2(mean_sin, mean_cos) * 180.0 / M_PI;
    if (estimate.theta < 0) estimate.theta += 360;
    estimate.weight = 1.0;
    return estimate;
}

void opcontrol() {
    // Calibrate IMU
    imu_sensor.reset(true); // blocking calibration
    // Reset encoders
    drive_left.tare_position();
    drive_right.tare_position();
    prev_left_enc = 0;
    prev_right_enc = 0;
    prev_imu_heading = imu_sensor.get_rotation();

    // Initialize particles with no prior (uniform)
    initializeParticles(/* x0,y0,theta0 */ 0, 0, 0, /*useGuess*/ false);

    while (true) {
        // 1. Motion update
        updateParticlesWithMotion();
        // 2. Sensor update (distance sensors)
        updateParticlesWithSensor();
        // 3. Resample based on new weights
        resampleParticles();
        // 4. Get current pose estimate
        Particle est = getEstimatedPose();

        // Use or display the estimate (for debugging, print to console or controller)
        printf("Estimated pose: x=%.1f mm, y=%.1f mm, θ=%.1f deg\n", est.x, est.y, est.theta);

        pros::delay(20); // 20 ms loop delay (~50 Hz update rate)
    }
}