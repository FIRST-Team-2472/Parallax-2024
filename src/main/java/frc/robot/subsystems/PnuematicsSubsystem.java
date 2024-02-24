package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PnuematicsConstants;

public class PnuematicsSubsystem extends SubsystemBase {

	Compressor compressor = new Compressor(PnuematicsConstants.kCompressorid, PneumaticsModuleType.REVPH);

	boolean bigLeftExtended;
	DoubleSolenoid doubleSolenoidBigLeft = new DoubleSolenoid(PneumaticsModuleType.REVPH,
			PnuematicsConstants.kBigLeftPnuematicInflateChannel,
			PnuematicsConstants.kBigLeftPnuematicDeflateChannel);

	boolean smallLeftExtended;
	DoubleSolenoid doubleSolenoidSmallLeft = new DoubleSolenoid(PneumaticsModuleType.REVPH,
			PnuematicsConstants.kSmallLeftPnuematicInflateChannel,
			PnuematicsConstants.kSmallLeftPnuematicDeflateChannel);

	boolean bigRightExtended;
	DoubleSolenoid doubleSolenoidBigRight = new DoubleSolenoid(PneumaticsModuleType.REVPH,
			PnuematicsConstants.kBigRightPnuematicInflateChannel,
			PnuematicsConstants.kBigRightPnuematicDeflateChannel);

	boolean smallRightExtended;
	DoubleSolenoid doubleSolenoidSmallRight = new DoubleSolenoid(PneumaticsModuleType.REVPH,
			PnuematicsConstants.kSmallRightPnuematicInflateChannel,
			PnuematicsConstants.kSmallRightPnuematicDeflateChannel);

	public PnuematicsSubsystem() {
		doubleSolenoidBigLeft.set(DoubleSolenoid.Value.kReverse);
		doubleSolenoidBigRight.set(DoubleSolenoid.Value.kReverse);
		doubleSolenoidSmallLeft.set(DoubleSolenoid.Value.kReverse);
		doubleSolenoidSmallRight.set(DoubleSolenoid.Value.kReverse);

		bigLeftExtended = false;
		smallLeftExtended = false;
		bigRightExtended = false;
		smallLeftExtended = false;
	}

	public void toggleSmallpnuematics() {
		if (!bigLeftExtended) {
			toggleSmallLeftPneumatic();
		}
		if (!bigRightExtended) {
			toggleSmallRightPneumatic();
		}
	}

	public void toggleBigpnuematics() {
		if (smallLeftExtended) {
			toggleBigLeftPneumatic();
		}
		if (smallRightExtended) {
			toggleBigRightPneumatic();
		}
	}

	public void toggleBigLeftPneumatic() {
		doubleSolenoidBigLeft.toggle();
		bigLeftExtended = bigLeftExtended ? false : true;
	}

	public void toggleSmallLeftPneumatic() {
		doubleSolenoidSmallLeft.toggle();
		smallLeftExtended = smallLeftExtended ? false : true;
	}

	public void toggleBigRightPneumatic() {
		doubleSolenoidBigRight.toggle();
		bigRightExtended = bigRightExtended ? false : true;
	}

	public void toggleSmallRightPneumatic() {
		doubleSolenoidSmallRight.toggle();
		smallRightExtended = smallRightExtended ? false : true;
	}

	public boolean getBigLeftPnuematic() {
		return bigLeftExtended;
	}

	public boolean getSmallLeftPnuematic() {
		return smallLeftExtended;
	}

	public boolean getBigRightPnuematic() {
		return bigRightExtended;
	}

	public boolean getSmallRightPnuematic() {
		return smallRightExtended;
	}
}
