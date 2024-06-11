package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private final SimpleMotorFeedforward ffModel;
    // private final SysIdRoutine sysId;

    ArmVisualizer visualizerMeasured;
    ArmVisualizer visualizerSetpoint;

    private double armTarget = ArmConstants.stow.arm().getDegrees();
    private double wristTarget = ArmConstants.stow.wrist().getDegrees();

    public Arm(ArmIO io) {
        this.io = io;

        visualizerMeasured = new ArmVisualizer("ArmMeasured", null);
        visualizerSetpoint = new ArmVisualizer("ArmSetpoint", new Color8Bit(Color.kOrange));

        // Switch constants based on mode (the physics simulator is treated as a
        // separate robot with different tuning)
        switch (Constants.getMode()) {
            case REAL:
            case REPLAY:
                ffModel = new SimpleMotorFeedforward(0.1, 0.05);
                io.configurePID(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
                break;
            case SIM:
                ffModel = new SimpleMotorFeedforward(0.0, 0.03);
                io.configurePID(0.5, 0.0, 0.0, 0.0, 0.0, 0.0);
                break;
            default:
                ffModel = new SimpleMotorFeedforward(0.0, 0.0);
                break;
        }

        // // Configure SysId
        // sysId = new SysIdRoutine(
        // new SysIdRoutine.Config(
        // null,
        // null,
        // null,
        // (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
        // new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null,
        // this));
        // }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
        visualizerMeasured.update(inputs.armRelativePositionDeg, inputs.wristRelativePositionDeg);
    }

    public void setArmTarget(double armTarget) {
        this.armTarget = armTarget;
        io.setArmTarget(armTarget);
        Logger.recordOutput("Arm/ArmTargetPositionDeg", armTarget);
        visualizerSetpoint.update(this.armTarget, this.wristTarget);
    }

    public void setWristTarget(double wristTarget) {
        this.wristTarget = wristTarget;
        io.setWristTarget(wristTarget);
        Logger.recordOutput("Arm/WristTargetPositionDeg", wristTarget);
        visualizerSetpoint.update(this.armTarget, this.wristTarget);
    }

    public void setArmAndWristTarget(double armTarget, double wristTarget) {
        this.wristTarget = wristTarget;
        this.armTarget = armTarget;

        io.setWristTarget(wristTarget);
        Logger.recordOutput("Arm/WristTargetPositionDeg", wristTarget);

        io.setArmTarget(armTarget);
        Logger.recordOutput("Arm/ArmTargetPositionDeg", armTarget);

        visualizerSetpoint.update(this.armTarget, this.wristTarget);
    }

    public void stop() {
        io.stop();
    }

    public double getWristAngleRelative() {
        return inputs.wristRelativePositionDeg;
    }

    public double getWristAngleAbsolute() {
        return inputs.wristAbsolutePositionDeg;
    }

    public double getArmAngleAbsolute() {
        return inputs.armAbsolutePositionDeg;
    }

    public double getArmAngleRelative() {
        return inputs.armRelativePositionDeg;
    }

    public double getRelativeWristTarget() {
        return wristTarget;
    }

    public double getRelativeArmTarget() {
        return armTarget;
    }

    @AutoLogOutput(key = "Arm/isArmWristInStowPosition")
    public boolean isArmWristInStowPosition() {
        return (Math.abs(ArmConstants.stow.arm().getRadians() - getArmAngleRelative()) < (Units.degreesToRadians(1)))
                && (Math.abs(ArmConstants.stow.wrist().getRadians() - getWristAngleRelative()) < (Units
                        .degreesToRadians(1)));
    }

    @AutoLogOutput(key = "Arm/isArmWristInIntakeFloorPosition")
    public boolean isArmWristInIntakePosition() {
        return (Math
                .abs(ArmConstants.intakeFloor.arm().getRadians() - getArmAngleRelative()) < (Units.degreesToRadians(1)))
                && (Math.abs(ArmConstants.intakeFloor.wrist().getRadians() - getWristAngleRelative()) < (Units
                        .degreesToRadians(1)));
    }

    @AutoLogOutput(key = "Arm/isArmWristInIntakeSourcePosition")
    public boolean isArmWristInSourcePosition() {
        return (Math
                .abs(ArmConstants.intakeFloor.arm().getRadians() - getArmAngleRelative()) < (Units.degreesToRadians(1)))
                && (Math.abs(ArmConstants.intakeFloor.wrist().getRadians() - getWristAngleRelative()) < (Units
                        .degreesToRadians(1)));
    }

    @AutoLogOutput(key = "Arm/isArmWristInBottomGridPosition")
    public boolean isArmWristInBottomGridPosition() {
        return (Math
                .abs(ArmConstants.bottomGrid.arm().getRadians() - getArmAngleRelative()) < (Units.degreesToRadians(1)))
                && (Math.abs(ArmConstants.bottomGrid.wrist().getRadians() - getWristAngleRelative()) < (Units
                        .degreesToRadians(1)));
    }

    @AutoLogOutput(key = "Arm/isArmWristInMidGridPosition")
    public boolean isArmWristInMidGridPosition() {
        return (Math.abs(ArmConstants.midGrid.arm().getRadians() - getArmAngleRelative()) < (Units.degreesToRadians(1)))
                && (Math.abs(ArmConstants.midGrid.wrist().getRadians() - getWristAngleRelative()) < (Units
                        .degreesToRadians(1)));
    }

    @AutoLogOutput(key = "Arm/isArmWristInTopGridPosition")
    public boolean isArmWristInTopGridPosition() {
        return (Math.abs(ArmConstants.topGrid.arm().getRadians() - getArmAngleRelative()) < (Units.degreesToRadians(1)))
                && (Math.abs(ArmConstants.topGrid.wrist().getRadians() - getWristAngleRelative()) < (Units
                        .degreesToRadians(1)));
    }
}