package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.debug.DebugRebootAll;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.LedControllerSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.util.AutoHandler;
import frc.robot.util.DashboardHandler;

/**
 * Main class of the Robot.
 */
public class Robot extends TimedRobot {

	public static final CameraSubsystem camera = new CameraSubsystem();
	public static final LedControllerSubsystem leds = new LedControllerSubsystem();
	public static final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
	public static final PneumaticSubsystem pneumatics = new PneumaticSubsystem();

	/** Reference this to get input from the joysticks and Xbox controller. */
	public static ControlHandler input;

	@Override
	public void robotInit() {
		input = new ControlHandler();
		SmartDashboard.putData(camera);
		SmartDashboard.putData(leds);
		SmartDashboard.putData(drivetrain);
		SmartDashboard.putData(pneumatics);
	}

	@Override
	public void robotPeriodic() {
		drivetrain.onLoop(Timer.getFPGATimestamp());
	}

	@Override
	public void disabledInit() {
		DebugRebootAll.rebootAll();
		Scheduler.getInstance().removeAll();
		AutoHandler.getInstance().reset();
		DashboardHandler.getInstance().init();
	}

	@Override
	public void disabledPeriodic() {
		camera.setPipeline(CameraSubsystem.PIPELINE_DRIVER);
	}

	@Override
	public void autonomousInit() {
		Scheduler.getInstance().removeAll();
		AutoHandler.getInstance().selectCase();		
		Scheduler.getInstance().add(AutoHandler.getInstance().getCase());
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		Scheduler.getInstance().removeAll();
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void testInit() {
		Scheduler.getInstance().removeAll();
	}

	@Override
	public void testPeriodic() {
		Scheduler.getInstance().run();
	}

}