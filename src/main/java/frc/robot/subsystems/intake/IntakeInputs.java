
package frc.robot.subsystems.intake;

import java.util.function.Supplier;

public class IntakeInputs {

    public enum MoveStatus {
        FRONT, BACK, STOP
    }

    public final Supplier<Boolean> intakeIn;
    public final Supplier<Boolean> intakeOut;

    public IntakeInputs(Supplier<Boolean> intakeIn, Supplier<Boolean> intakeOut) {
        this.intakeIn = intakeIn;
        this.intakeOut = intakeOut;
    }
}
