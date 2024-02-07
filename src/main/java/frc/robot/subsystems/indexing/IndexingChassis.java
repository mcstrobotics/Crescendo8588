package frc.robot.subsystems.indexing;

import com.revrobotics.CANSparkMax;

public class IndexingChassis {
    private CANSparkMax rightmotor;
    private CANSparkMax leftmotor;

    public IndexingChassis(CANSparkMax rightmotor, CANSparkMax leftmotor) {
        this.rightmotor = rightmotor;
        this.leftmotor = leftmotor;
    }

    public CANSparkMax getRightMotor() {
        return rightmotor;
    }

    public CANSparkMax getLeftMotor() {
        return leftmotor;
    }
}
