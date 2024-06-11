package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public final class ArmConstants {
    public static final ArmPositions stow = new ArmPositions(Rotation2d.fromDegrees(23),
            Rotation2d.fromDegrees(65));
    public static final ArmPositions intakeFloor = new ArmPositions(Rotation2d.fromDegrees(23),
            Rotation2d.fromDegrees(65));
    public static final ArmPositions intakeSource = new ArmPositions(Rotation2d.fromDegrees(23),
            Rotation2d.fromDegrees(65));
    public static final ArmPositions bottomGrid = new ArmPositions(Rotation2d.fromDegrees(23),
            Rotation2d.fromDegrees(65));
    public static final ArmPositions midGrid = new ArmPositions(Rotation2d.fromDegrees(23),
            Rotation2d.fromDegrees(65));
    public static final ArmPositions topGrid = new ArmPositions(Rotation2d.fromDegrees(23),
            Rotation2d.fromDegrees(65));

    public static final int ARM_GEAR_RATIO = 50;
    public static final int WRIST_GEAR_RATIO = 50;

    public record ArmPositions(Rotation2d arm, Rotation2d wrist) {
    }

    public record MotorFeedbackController(double kP, double kI, double kD, double kG) {
    }
}
