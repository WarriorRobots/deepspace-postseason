package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.Constants;

/**
 * Contains all pneumatics on the robot.
 */
public class PneumaticSubsystem extends Subsystem {

	private Compressor compressor;

	/**
	 * Instantiates new subsystem; make ONLY ONE.
	 * <p>
	 * <code> public static final HatchPlacerSubsystem hatchPlacer = new
	 * HatchPlacerSubsystem();
	 */
	public PneumaticSubsystem() {
		compressor = new Compressor(Constants.PCM_1);
		compressor.start();
	}

	/**
	 * Allows the compressor to pump air at low pressures (not all the time).
	 */
	public void enableCompressor() {
		compressor.start();
	}

	/**
	 * Prevents the compressor from pumping air, at any time.
	 */
	public void disableCompressor() {
		compressor.stop();
	}

	@Override
	protected void initDefaultCommand() {
		// none
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		// builder.addBooleanProperty("low pressure?", () -> !compressor.getPressureSwitchValue(), null);
		// builder.addBooleanProperty("compressor?", () -> compressor.enabled(), null);
	}

}