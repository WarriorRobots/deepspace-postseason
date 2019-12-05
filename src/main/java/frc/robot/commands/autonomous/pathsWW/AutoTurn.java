package frc.robot.commands.autonomous.pathsWW;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.QuickAccessVars;
import frc.robot.Robot;
import frc.robot.util.SynchronousPIDF;

/**
 * When run, the robot will turn to the provided angle,
 * using a PID loop to maintain accuracy and control.
 */
public class AutoTurn extends Command {
	
	private double angleTarget, output;
	
	private boolean stopsAtSetpoint = true;
	
	private SynchronousPIDF pidLoop;
	private Timer timer;
	
	/**
	 * Create a new instance of {@link TurnAuto}.
	 * @param angle  What angle in degrees to turn towards. (Right is positive)
	 */
	public AutoTurn(double angle) {
		requires(Robot.drivetrain);

		angleTarget = angle;
		
		pidLoop = new SynchronousPIDF(
			QuickAccessVars.AUTO_TURN_P,
			QuickAccessVars.AUTO_TURN_I,
			QuickAccessVars.AUTO_TURN_D);
		
		timer = new Timer();
		//System.out.println("angleTarget " + Double.toString(this.angleTarget));
	}

	/**
	 * Set the internal PID constants to new values
	 * @param p  P gain
	 * @param i  I gain
	 * @param d  D gain
	 */
	public void setPID(double p, double i, double d) {
		pidLoop.setPID(p, i, d);
	}

	@Override
	protected void initialize() {
    Robot.drivetrain.resetAngle();
    try {
      pidLoop.setIzone(-QuickAccessVars.AUTO_TURN_TOLERANCE, QuickAccessVars.AUTO_TURN_TOLERANCE);
      pidLoop.setOutputRange(-1, 1);
    } catch (Exception e) {}
		pidLoop.setSetpoint(angleTarget);
		timer.start();
	}
	
	@Override
	protected void execute() {
		output = pidLoop.calculate(Robot.drivetrain.getAngleDegrees(), timer.get());
		Robot.drivetrain.arcadeDriveRaw(0, output);
	}

	@Override
	protected boolean isFinished() {
		if (pidLoop.onTarget(QuickAccessVars.AUTO_TURN_TOLERANCE) && stopsAtSetpoint) {
			return true;
		} else {
			return false;
		}
	}
	
	@Override
	protected void end() {
		timer.stop();
		pidLoop.reset();
		Robot.drivetrain.stopDrive();
	}
}