package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;

public class ShooterChassis {
    private CANSparkMax shooterAim;
    private CANSparkMax shooterBottom;
    private CANSparkMax shooterTop;

    public ShooterChassis(CANSparkMax shooterAim, CANSparkMax shooterBottom, CANSparkMax shooterTop) {
        this.shooterAim = shooterAim;
        this.shooterBottom = shooterBottom;
        this.shooterTop = shooterTop;
    }

    public CANSparkMax getShooterAim() {
        return shooterAim;
    }

    public CANSparkMax getShooterBottom() {
        return shooterBottom;
    }

    public CANSparkMax getShooterTop() {
        return shooterTop;
    }
}
