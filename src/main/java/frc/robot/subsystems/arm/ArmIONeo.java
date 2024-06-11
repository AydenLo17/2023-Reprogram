package frc.robot.subsystems.arm;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.util.Units;
import frc.robot.util.VendorWrappers.Neo;

import static frc.robot.subsystems.arm.ArmConstants.*;

public class ArmIONeo implements ArmIO {
    private final Neo armLMotor = new Neo(0);
    private final Neo armRMotor = new Neo(1);
    private final Neo wristMotor = new Neo(2);

    private final AbsoluteEncoder armEncoder = armLMotor.getAbsoluteEncoder();
    private final AbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder();

    private final SparkPIDController armPID = armLMotor.getPIDController();
    private final SparkPIDController wristPID = wristMotor.getPIDController();

    public ArmIONeo() {

        armLMotor.restoreFactoryDefaults();
        armRMotor.restoreFactoryDefaults();
        wristMotor.restoreFactoryDefaults();

        armLMotor.setCANTimeout(250);
        armRMotor.setCANTimeout(250);
        wristMotor.setCANTimeout(250);

        armLMotor.setInverted(false);
        armRMotor.follow(armLMotor, false);
        wristMotor.setInverted(false);

        armLMotor.enableVoltageCompensation(12.0);
        wristMotor.enableVoltageCompensation(12.0);

        armLMotor.setSmartCurrentLimit(30);
        wristMotor.setSmartCurrentLimit(30);

        armLMotor.burnFlash();
        armRMotor.burnFlash();
        wristMotor.burnFlash();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armAbsolutePositionDeg = armEncoder.getPosition();
        inputs.armRelativePositionDeg = Units.rotationsToDegrees(armLMotor.getPosition() / ARM_GEAR_RATIO);
        inputs.armVelocityRotPerSec = armEncoder.getVelocity();
        inputs.armCurrentAmps = new double[] { armLMotor.getOutputCurrent(), armRMotor.getOutputCurrent() };

        inputs.wristAbsolutePositionDeg = wristEncoder.getPosition();
        inputs.wristRelativePositionDeg = Units.rotationsToDegrees(wristMotor.getPosition() / WRIST_GEAR_RATIO);
        inputs.wristVelocityRotPerSec = wristEncoder.getVelocity();
        inputs.wristCurrentAmps = new double[] { wristMotor.getOutputCurrent() };
    }

    public void setArmTarget(double armTargetAngleDeg) {
        armPID.setReference(armTargetAngleDeg, ControlType.kPosition);
    }

    public void setWristTarget(double wrustTargetAngleDeg) {
        wristPID.setReference(wrustTargetAngleDeg, ControlType.kPosition);
    }

    @Override
    public void setBrakeMode(boolean armBrake, boolean wristBrake) {
        armLMotor.setIdleMode(armBrake ? IdleMode.kBrake : IdleMode.kBrake.kCoast);
        wristMotor.setIdleMode(wristBrake ? IdleMode.kBrake : IdleMode.kBrake.kCoast);
    }

    @Override
    public void stop() {
        armLMotor.stopMotor();
        wristMotor.stopMotor();
    }

    public void configurePID(double armKP, double armKI, double armKD, double wristKP, double wristKI, double wristKD) {
        armPID.setP(armKP, 0);
        armPID.setI(armKI, 0);
        armPID.setD(armKD, 0);
        armPID.setFF(0, 0);

        wristPID.setP(wristKP, 0);
        wristPID.setI(wristKI, 0);
        wristPID.setD(wristKD, 0);
        wristPID.setFF(0, 0);
    }
}
