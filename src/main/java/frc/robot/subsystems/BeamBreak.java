package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreak {
    private DigitalInput beamBreaker;
    public BeamBreak(int channel) {
        beamBreaker = new DigitalInput(channel);
    }
    public boolean isBeamBroken() {
        return beamBreaker.get();
    }
}