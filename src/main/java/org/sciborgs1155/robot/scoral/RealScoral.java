package org.sciborgs1155.robot.scoral;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.lib.FaultLogger.check;
import static org.sciborgs1155.robot.Ports.Scoral.*;
import static org.sciborgs1155.robot.scoral.ScoralConstants.CURRENT_LIMI;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class RealScoral implements ScoralIO {

  private final SparkMax topMotor;
  private final SparkMax bottomMotor;

  private final DigitalInput beambreak;

  public RealScoral() {
    topMotor = new SparkMax(TOP_ROLLER, MotorType.kBrushed);
    bottomMotor = new SparkMax(BOTTOM_ROLLER, MotorType.kBrushed);

    beambreak = new DigitalInput(BEAMBREAK);

    SparkMaxConfig topConfig = new SparkMaxConfig();
    check(
        topMotor,
        topMotor.configure(
            topConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));

    topConfig.apply(
        topConfig.idleMode(IdleMode.kBrake).smartCurrentLimit((int) CURRENT_LIMI.in(Amps)));

    check(
        topMotor,
        topMotor.configure(
            topConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));

    SparkMaxConfig bottomConfig = new SparkMaxConfig();
    check(
        bottomMotor,
        bottomMotor.configure(
            bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));

    bottomConfig.apply(bottomConfig.apply(topConfig).follow(TOP_ROLLER, true));

    check(
        bottomMotor,
        bottomMotor.configure(
            bottomConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void setPower(double power) {
    topMotor.set(power);
  }

  @Override
  public boolean beambreak() {
    return beambreak.get();
  }
}
