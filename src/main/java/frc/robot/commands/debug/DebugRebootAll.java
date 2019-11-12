package frc.robot.commands.debug;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Robot;
import frc.robot.subsystems.CameraSubsystem;

public class DebugRebootAll extends InstantCommand {

	/**
	 * In one loop: sets all motor speeds to zero, resets all solenoids, and stops
	 * all commands. This effectively reboots every component on the robot except
	 * encoders. Use with care!
	 * 
	 * <p>
	 * The static method <code>rebootAll()</code> is also made available for running
	 * this outside of a command.
	 * 
	 * @see #rebootAll()
	 */
	public DebugRebootAll() {
		requires(Robot.camera);
		requires(Robot.drivetrain);
		requires(Robot.pneumatics);
	}

	@Override
	protected void execute() {
		rebootAll();
	}

	/**
	 * In one loop: sets all motor speeds to zero, resets all solenoids, and stops
	 * all commands. This effectively reboots every component on the robot except
	 * encoders. Use with care!
	 */
	public static void rebootAll() {
		System.out.println("Debug: Running DebugRebootAll"); // can't use this.getClass().getSimpleName() because it's static
		Robot.camera.setPipeline(CameraSubsystem.PIPELINE_DRIVER);
		Robot.drivetrain.stopDrive();
		Robot.pneumatics.enableCompressor();
	}

	@Override
	protected void end() {
		// kills all currently-running commands
		Scheduler.getInstance().removeAll();
	}

}