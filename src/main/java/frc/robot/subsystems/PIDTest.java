
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDTest extends SubsystemBase {

  CANSparkMax test = new CANSparkMax(0, MotorType.kBrushless);
  SparkMaxPIDController testPID = test.getPIDController();

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
    // Sets the position for the PID Controller (units are revolutions)
    testPID.setReference(position, ControlType.kPosition);
  }
}
