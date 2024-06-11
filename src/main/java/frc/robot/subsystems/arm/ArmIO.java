package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double armAbsolutePositionDeg = 0.0;
    public double armRelativePositionDeg = 0.0;
    public double armVelocityRotPerSec = 0.0;
    public double[] armCurrentAmps = new double[] {};
    public double[] armTempCelcius = new double[] {};

    public double wristAbsolutePositionDeg = 0.0;
    public double wristRelativePositionDeg = 0.0;
    public double wristVelocityRotPerSec = 0.0;
    public double[] wristCurrentAmps = new double[] {};
    public double[] wristTempCelcius = new double[] {};
  }

  public default void setArmTarget(double target) {}

  public default void setWristTarget(double target) {}

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setBrakeMode(boolean armBrake, boolean wristBrake) {}

  public default void stop() {}

    /** Set velocity PID constants. */
    public default void configurePID(double armKP, double armKI, double armKD, double wristKP, double wristKI, double wristKD) {}
}