
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDTest extends SubsystemBase {

  CANSparkMax test = new CANSparkMax(0, MotorType.kBrushless);
  SparkMaxPIDController testPID = test.getPIDController();

  /* Note to self: the encoder getPosition() method automatically returns in terms
  *of revolutions, but it can also perform automatic conversion via the 
  *setPositionConversionFactor() method. The PID controllers when set to position mode
  *should use revolutions, but the documentation does not say for certain. Would be wise to
  *test before attaching to robot
  */

  /** Creates a new PIDTest. */
  public PIDTest() {
    testPID.setP(0.1);
    testPID.setI(0.1);
    testPID.setD(0.1);
    testPID.setFF(0.1);
    test.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPosition(double position) {
    testPID.setReference(position, ControlType.kPosition);
  }
}
