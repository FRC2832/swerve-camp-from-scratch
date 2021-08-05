// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class SwerveModule {
    //private static final double kWheelRadius = 0.0508;
    //private static final int kEncoderResolution = 4096;

    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    private final WPI_TalonFX driveMotor;
    private final CANSparkMax turningMotor;

    private final CANCoder turningEncoder;
    //private double turnMotorAngle;

    private final PIDController drivePIDController = new PIDController(1.0, 0.0, 0.0);

    private final ProfiledPIDController turningPIDController = new ProfiledPIDController(1.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(1.0, 0.5);
    private final SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(1.0, 0.5);

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel   ID for the drive motor.
     * @param turningMotorChannel ID for the turning motor.
     * @param canCoderID          ID for the CANCoder (mounted to turning motor.)
     */
    public SwerveModule(int driveMotorChannel, int turningMotorChannel, int canCoderID) {
        driveMotor = new WPI_TalonFX(driveMotorChannel);
        //driveMotor.configClosedloopRamp(1.0);
        turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
        //turningMotor.setClosedLoopRampRate(2.0);
        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        //driveMotor
        //driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
        // Set the distance (in this case, angle) per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * wpi::math::pi)
        // divided by the encoder resolution.
        //turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);
        turningEncoder = new CANCoder(canCoderID);
        turningEncoder.setPositionToAbsolute();
        //turnMotorAngle = turningEncoder.getAbsolutePosition();
        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getSelectedSensorVelocity(), new Rotation2d(turningEncoder.getAbsolutePosition() * Math.PI / 180));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turningEncoder.getAbsolutePosition() * Math.PI / 180));
        // Calculate the drive output from the drive PID controller.
        final double driveOutput = drivePIDController.calculate(driveMotor.getSelectedSensorVelocity(), state.speedMetersPerSecond);

        final double driveFeedforward = driveFeedForward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = turningPIDController.calculate(turningEncoder.getAbsolutePosition() * Math.PI / 180, state.angle.getRadians());

        final double turnFeedforward = turnFeedForward.calculate(turningPIDController.getSetpoint().velocity);


        driveMotor.setVoltage(driveOutput + driveFeedforward);
        turningMotor.setVoltage(turnOutput + turnFeedforward);
    }

    public String getMotorOutputString() {
        return "Drive output: " + driveMotor.get() + "\nTurn output: " + turningMotor.get();
    }

    public void setTurnMagnetOffset(double angle) {
        turningEncoder.configGetMagnetOffset();
    }

    public double getTurnMotorValue() {
        return turningEncoder.getAbsolutePosition();
    }
}
