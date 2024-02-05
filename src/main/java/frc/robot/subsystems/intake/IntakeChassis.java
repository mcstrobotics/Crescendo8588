package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;

public class IntakeChassis {
  private CANSparkMax intake;

  public IntakeChassis(CANSparkMax intake) {
    this.intake = intake;
  }

  public CANSparkMax getIntake() {
    return intake;
  }
}