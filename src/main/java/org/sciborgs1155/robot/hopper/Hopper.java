package org.sciborgs1155.robot.hopper;

import org.sciborgs1155.robot.Robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Hopper extends SubsystemBase implements AutoCloseable {
    public static Hopper create() {
        // HopperIO hardware = Robot.isReal() ? new RealHopper() : (Robot.isSimulation() ? new SimHopper() : new NoHopper());
        // return new Hopper(hardware);

        return Robot.isReal() ? new Hopper(new RealHopper()) : Hopper.none();
    }

    public static Hopper none() {
        return new Hopper(new NoHopper());
    }

    private final HopperIO hardware;
    private final Trigger beambreakTrigger;
    
    public Hopper(HopperIO hardware) {
        this.hardware = hardware;
        this.beambreakTrigger = new Trigger(hardware::beambreak);
    }

    public Command runHopper(double power) {
        return runOnce(() -> hardware.setPower(power));
    }

    public Command intake() {
        return runHopper(HopperConstants.INTAKE_POWER);
    }

    public Command outtake() {
        return runHopper(-HopperConstants.INTAKE_POWER);
    }

    @Override
    public void close() throws Exception {
        hardware.close();
    }
    
}
