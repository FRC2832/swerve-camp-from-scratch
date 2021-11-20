// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class SwerveModule {
    // private static final double kWheelRadius = 0.0508;
    // private static final int kEncoderResolution = 4096;

    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    private final WPI_TalonFX driveMotor;
    private final CANSparkMax turningMotor;

    private final CANCoder absEncoder;
    private final CANEncoder turningEncoder;
    //private double turnMotorAngle;

    private final PIDController drivePIDController = new PIDController(1.0, 0.0, 0.0);

    private final ProfiledPIDController turningPIDController = new ProfiledPIDController(1.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(1.0, 0.5);
    private final SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(1.0, 0.5);
    private double turnVoltCommand;
    private double driveVoltCommand;
    private SwerveConstants constants;

    // Using FlywheelSim as a stand-in for a simple motor
    private FlywheelSim m_turnMotorSim;
    private FlywheelSim m_driveMotorSim;

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel   ID for the drive motor.
     * @param turningMotorChannel ID for the turning motor.
     * @param canCoderID          ID for the CANCoder (mounted to turning motor.)
     */
    public SwerveModule(SwerveConstants cornerConstants) {
        constants = cornerConstants;
        driveMotor = new WPI_TalonFX(constants.DriveMotorId);
        turningMotor = new CANSparkMax(constants.TurnMotorId, MotorType.kBrushless);
        turningEncoder = turningMotor.getEncoder();
        /* TODO: Turn on brake mode for motors
        driveMotor.setNeutralMode(NeutralMode.Brake);
        turningMotor.setIdleMode(IdleMode.kBrake);
        */

        /* TODO: Turn on hardware PID control 
        var encoder = turningMotor.getEncoder();
        encoder.setInverted(false);
        //This example assumes a 4" wheel on a 15:1 reduction
        encoder.setPositionConversionFactor(factor);
        CANPIDController pidController = turningMotor.getPIDController();
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setFF(ff);
        pidController.setIZone(IZone);  //IZone is the amount of error before the I term is considered.  It is designed so you don't get a ton of I-term build up at the beginning of the PID control.  You set it when you want I to kick in near the end of control.  It should be positive, 0 disables it.
        pidController.setOutputRange(min, max);
        */

        absEncoder = new CANCoder(constants.TurnMotorId);
        absEncoder.setPositionToAbsolute();
        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        if(Robot.isSimulation()) {
            m_turnMotorSim = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(constants.TurnMotorKv, constants.TurnMotorKa),
                constants.TurnMotor,
                constants.TurnMotorGearRatio
            );

            m_driveMotorSim = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(constants.DriveMotorKv, constants.DriveMotorKa),
                constants.DriveMotor,
                constants.DriveMotorGearRatio
            );
        }
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getSelectedSensorVelocity(), new Rotation2d(absEncoder.getAbsolutePosition() * Math.PI / 180));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(absEncoder.getAbsolutePosition() * Math.PI / 180));
        // Calculate the drive output from the drive PID controller.
        final double driveOutput = drivePIDController.calculate(driveMotor.getSelectedSensorVelocity(), state.speedMetersPerSecond);
        final double driveFeedforward = driveFeedForward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = turningPIDController.calculate(absEncoder.getAbsolutePosition() * Math.PI / 180, state.angle.getRadians());
        final double turnFeedforward = turnFeedForward.calculate(turningPIDController.getSetpoint().velocity);


        turnVoltCommand = turnOutput + turnFeedforward;
        driveVoltCommand = driveOutput + driveFeedforward;

        driveMotor.setVoltage(driveVoltCommand);
        turningMotor.setVoltage(turnVoltCommand);

        /* TODO: use hardware control of motor control instead of SW
        //set the motor to 10 revolutions.  We should divide the encoder to degrees for better control 
        pidController.setReference(10.0, ControlType.kPosition);
        */
    }

    public String getMotorOutputString() {
        return "Drive output: " + driveMotor.get() + "\nTurn output: " + turningMotor.get();
    }

    public void setTurnMagnetOffset(double angle) {
        absEncoder.configGetMagnetOffset();
    }

    public double getTurnMotorValue() {
        return absEncoder.getAbsolutePosition();
    }

    public void simulationPeriodic(double rate) {
        //we need to calculate the motor velocities and encoder positions since they aren't real here
        m_turnMotorSim.setInputVoltage(turnVoltCommand);
        m_driveMotorSim.setInputVoltage(driveVoltCommand);
    
        m_turnMotorSim.update(rate);
        m_driveMotorSim.update(rate);
    
        // Calculate distance traveled using RPM * dt
        var dist = turningEncoder.getPosition();
        dist += m_turnMotorSim.getAngularVelocityRadPerSec() * rate;
        turningEncoder.setPosition(dist);
    
        dist = driveMotor.getSensorCollection().getIntegratedSensorAbsolutePosition();
        dist += m_driveMotorSim.getAngularVelocityRadPerSec() * rate;
        driveMotor.getSensorCollection().setIntegratedSensorPosition(dist, 0);
        absEncoder.setPosition(dist);
    }

    public void putSmartDashboard() {
        SmartDashboard.putNumber(constants.Name + "/driveEncoderRaw", driveMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber(constants.Name + "/absEncoderRaw", absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(constants.Name + "/turnEncoderRaw", turningEncoder.getPosition());
    }
}
