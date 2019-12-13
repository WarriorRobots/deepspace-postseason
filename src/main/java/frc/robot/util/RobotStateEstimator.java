package frc.robot.util;

import frc.robot.Robot;
import frc.robot.util.Kinematics;
import frc.robot.util.RobotState;
// import frc.robot.loops.ILooper;
// import frc.robot.loops.Loop;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Twist2d;

public class RobotStateEstimator {
    static RobotStateEstimator mInstance = new RobotStateEstimator();
    private RobotState mRobotState = RobotState.getInstance();
    private double left_encoder_prev_distance_ = 0.0;
    private double right_encoder_prev_distance_ = 0.0;
    private double prev_timestamp_ = -1.0;
    private Rotation2d prev_heading_ = null;

    public static RobotStateEstimator getInstance() {
        if (mInstance == null) {
            mInstance = new RobotStateEstimator();
        }

        return mInstance;
    }

    private RobotStateEstimator() {}

    
    public synchronized void onStart(double timestamp) {
        left_encoder_prev_distance_ = Robot.drivetrain.getLeftEncoderClicks();
        right_encoder_prev_distance_ = Robot.drivetrain.getLeftEncoderClicks();
        prev_timestamp_ = timestamp;
    }

    public synchronized void onLoop(double timestamp) {
        if (prev_heading_ == null) {
            prev_heading_ = mRobotState.getLatestFieldToVehicle().getValue().getRotation();
        }
        final double dt = timestamp - prev_timestamp_;
        final double left_distance = Robot.drivetrain.getLeftEncoderInches(); // inches
        final double right_distance = Robot.drivetrain.getRightEncoderInches(); // inches
        final double delta_left = left_distance - left_encoder_prev_distance_;
        final double delta_right = right_distance - right_encoder_prev_distance_;
        final Rotation2d gyro_angle = Rotation2d.fromDegrees(Robot.drivetrain.getAngleDegrees());
        Twist2d odometry_twist;
        synchronized (mRobotState) {
            final Pose2d last_measurement = mRobotState.getLatestFieldToVehicle().getValue();
            odometry_twist = Kinematics.forwardKinematics(last_measurement.getRotation(), delta_left,
                    delta_right, gyro_angle);
        }
        final Twist2d measured_velocity = Kinematics.forwardKinematics(
                delta_left, delta_right, prev_heading_.inverse().rotateBy(gyro_angle).getRadians()).scaled(1.0 / dt);
        final Twist2d predicted_velocity = Kinematics.forwardKinematics(Robot.drivetrain.getLeftLinearVelocity(),
                Robot.drivetrain.getRightLinearVelocity()).scaled(dt);
        // mRobotState.addVehicleToTurretObservation(timestamp,
        //         Rotation2d.fromDegrees(Turret.getInstance().getAngle()));
        mRobotState.addObservations(timestamp, odometry_twist, measured_velocity,
                predicted_velocity);
        left_encoder_prev_distance_ = left_distance;
        right_encoder_prev_distance_ = right_distance;
        prev_heading_ = gyro_angle;
        prev_timestamp_ = timestamp;
    }

    public void outputTelemetry() { // writes to the dashboard using the robotstate
         mRobotState.outputToSmartDashboard();
    }
}