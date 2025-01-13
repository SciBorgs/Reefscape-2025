package org.sciborgs1155.robot.scoral;

import static org.sciborgs1155.robot.Ports.Scoral.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;

public class RealScoral implements ScoralIO {

  private final SparkMax topMotor;
  private final SparkMax bottomMotor;

  private final DigitalInput beambreak;

  public RealScoral() {
    topMotor = new SparkMax(TOP_ROLLER, MotorType.kBrushed);
    bottomMotor = new SparkMax(BOTTOM_ROLLER, MotorType.kBrushed);

    beambreak = new DigitalInput(BEAMBREAK);
  }

  @Override
  public void setPower(double power) {}

  @Override
  public boolean beambreak() {
    return beambreak.get();
  }
}
