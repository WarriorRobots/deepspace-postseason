package frc.robot.commands.autonomous.deprecated;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.util.SynchronousPIDF;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.QuickAccessVars;
import frc.robot.Robot;
import frc.robot.subsystems.CameraSubsystem;

/**
 * ApproachCurve approachs the target keeping it to the left or right (depending
 * on the direction of approach) and when it is aligned up (by the aspect ratio
 * being 2.3 or above), it approach the rest of the distance with it in front.
 */
@Deprecated
public class ApproachCurve extends Command {

	/** PID used for approaching the wall. */
	private SynchronousPIDF PIDapproach;
	/** PID for keeping the target centered */
	private SynchronousPIDF PIDcenter;

	private Timer timer;
	/** Calculated PID output from {@link #PIDapproach} should stored in value. */
	private double valueapproach;
	/** Calculated PID output from {@link #PIDcenter} should stored in value. */
	private double valuecenter;

	/** Height of the left target, overall target, and right target
	 * (in that order as left as 0, and right as 2)
	 */
	private double[] target_height;

	/** Horizontal postion of the left target, overall target, and right target
	 * (in that order as left as 0, and right as 2)
	*/
	private double[] target_x;

	private double target_distance;

	/** Denotes if the drivers are in control of the robot during the duration of the command. */
	private boolean driver_control;

	/** Used for {@link #target_height} and {@link #target_x} */
	private final int LEFT = 0;
	/** Used for {@link #target_height} and {@link #target_x} */
	private final int OVERALL = 1;
	/** Used for {@link #target_height} and {@link #target_x} */
	private final int RIGHT = 2;

	/** Shorthand for pipeline */
	private final int PIPELEFT = CameraSubsystem.PIPELINE_TARGETLEFT;
	/** Shorthand for pipeline */
	private final int PIPEOVERALL = CameraSubsystem.PIPELINE_CENTER;
	/** Shorthand for pipeline */
	private final int PIPERIGHT = CameraSubsystem.PIPELINE_TARGETRIGHT;

	/** Keeps the id of the pipeline that is currently being asked to give values to the data arrays */
	private int intendedPipe;

	/** Denotes if the camera is missing data in order to start driving */
	private boolean data_missing;

	/** Left:Right height ratio of targets */
	private double heightRatio;

	@Deprecated
	public ApproachCurve() {
		requires(Robot.drivetrain);
		requires(Robot.camera);

		valueapproach = 0;
		valuecenter = 0;

		PIDapproach = new SynchronousPIDF(QuickAccessVars.KP_APPROACH, QuickAccessVars.KI_APPROACH,
				QuickAccessVars.KD_APPROACH);
		PIDcenter = new SynchronousPIDF(QuickAccessVars.KP_CENTER, QuickAccessVars.KI_CENTER,
				QuickAccessVars.KD_CENTER);

		timer = new Timer();
	}

	@Override
	protected void initialize() {
		PIDapproach.setSetpoint(QuickAccessVars.SETPOINT_APPROACH); // Robot should aim to be be <setpoint> in away from the target
		try {
			PIDapproach.setOutputRange(-0.7, 0.7);
		} catch (Exception e) {
		}

		PIDcenter.setSetpoint(QuickAccessVars.SETPOINT_CENTER); // Robot should aim to keep the target centered on the crosshair

		timer.start();

		intendedPipe = PIPERIGHT;
		Robot.camera.setPipeline(intendedPipe);

		target_height = new double[3];
		target_x = new double[3];
		target_distance = 0;

		driver_control = false;
		data_missing = false;
	}

	@Override
	protected void execute() {

		updateTargetData();

		if (Math.abs(Robot.input.getRightY()) > QuickAccessVars.CAMERA_DRIVE_THRESHOLD
				|| Math.abs(Robot.input.getRightX()) > QuickAccessVars.CAMERA_DRIVE_THRESHOLD) {
			driver_control = true;
		}

		if (!driver_control) { // if the drivers are not controlling it do the auto part
			auto();
		} else { // if the drivers are controlling it then let them do arcade drive
			drive();
		}
	}

	/** Moves the robot autonomously to the target. */
	private void auto() {
		if (target_height[LEFT] == 0 || target_height[OVERALL] == 0 || target_height[RIGHT] == 0 || target_x[LEFT] == 0
				|| target_x[OVERALL] == 0 || target_x[RIGHT] == 0 || target_distance == 0) {
			data_missing = true;
			return; // if any part of the data is unwritten to, then return out of function to avoid the
			// the robot driving before it has data
		} else {
			data_missing = false;
		}

		if (Robot.camera.canSeeObject()) {

			shiftCenter();

			valueapproach = PIDapproach.calculate(target_distance, timer.get());
			valuecenter = PIDcenter.calculate(target_x[OVERALL], timer.get());

			// helps to keep the robot to drive in a missile approach curve
		} else {
			// Set value to zero if the target can not be seen so robot does not go crazy
			valueapproach = 0;
			// Don't 0 valuecenter because it should "remember" what direction it's attempting to turn.
			valuecenter = 0;
		}
		Robot.drivetrain.arcadeDriveRaw(-valueapproach, -valuecenter);
	}

	private void drive() {
		Robot.drivetrain.arcadeDriveTeleop(Robot.input.getRightY(QuickAccessVars.ARCADE_FORWARD_MODIFIER),
				Robot.input.getRightX(QuickAccessVars.ARCADE_TURN_MODIFIER));
	}

	private void updateTargetData() {

		if (Robot.camera.getPipeline() == intendedPipe) { // wait for pipeline to be the intened one
			switch (intendedPipe) {

			case PIPELEFT: // Left
				target_height[LEFT] = Robot.camera.getTargetHeight();
				target_x[LEFT] = Robot.camera.getObjectX();
				Robot.camera.setPipeline(PIPEOVERALL);
				intendedPipe = PIPEOVERALL;
				break;
			case PIPEOVERALL: // Overall
				target_height[OVERALL] = Robot.camera.getTargetHeight();
				target_x[OVERALL] = Robot.camera.getObjectX();
				target_distance = Robot.camera.getTargetDistance();
				// don't move off the center pipeline if the targets become centered
				if (!(1 / 1.1 < heightRatio && heightRatio < 1.1 / 1) || data_missing) {
					Robot.camera.setPipeline(PIPERIGHT);
					intendedPipe = PIPERIGHT;
				} else {
					// set the targets to be the same height
					// so it doesn't try to turn to fix it
					heightRatio = 1;
				}
				break;
			case PIPERIGHT: // Right
				target_height[RIGHT] = Robot.camera.getTargetHeight();
				target_x[RIGHT] = Robot.camera.getObjectX();
				Robot.camera.setPipeline(PIPELEFT);
				intendedPipe = PIPELEFT;
				break;
			default:
				break;
			}
		}

		try { // try statement makes the robot doesn't crash when target_height isn't set
			if (heightRatio != 1) {
				// if the height ratio doesn't say its straight on...
				// find how off it is
				heightRatio = target_height[LEFT] / target_height[RIGHT];
			}
		} catch (Exception e) {
			heightRatio = 0;
		}

	}

	/**
	 * Shift the setpoint of the {@link #PIDcenter}
	 * with the intent of making the robot turn to compensate the angle the robot is facing the target at.
	 * This should make the turn be an arc to make the robot perpendicular to the target.
	 */
	private void shiftCenter() {
		// don't move the setpoint if there is an issue
		if (heightRatio == 0)
			return;

		// bound the height ratio within 2 to 0.5
		// this is so the turn is not to large
		if (heightRatio > 2)
			heightRatio = 2;
		if (heightRatio < 0.5)
			heightRatio = 0.5;

		// turn the ratio into a screenspace
		// by reversing a function that looks similar to the output from the ratio
		// (the function is the exponential R(x)=2^(-x) )
		// (so when x is -1 (full left), the ratio is 2)
		// (so when x is 1 (full right), the ratio is 0.5)

		// solving for x we get -log_2(R(x)) = x
		// there is no log base in java so we must use the property log_b(a) = log(a)/log(b)
		double x = -Math.log(heightRatio) / Math.log(2);

		// if the x is within tolerable range, ignore it
		if (Math.abs(x) < 0.1)
			x = 0;

		// now with x being between -1 and 1 we can multiply by the degrees we want the camera to turn
		// at most (25 degrees)
		double point = x * 25;
		PIDcenter.setSetpoint(point);
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void end() {
		timer.stop();
		PIDapproach.reset();
		PIDcenter.reset();
		valueapproach = 0;
		valuecenter = 0;
		Robot.camera.setPipeline(CameraSubsystem.PIPELINE_CENTER);
		Robot.drivetrain.stopDrive();
	}

}