// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
    public static final double kMaxSpeed = 3.0; // 3 meter(s) per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

    private final SwerveModule frontLeft = new SwerveModule(7, 8, 3);
    private final SwerveModule frontRight = new SwerveModule(5, 9, 0);
    private final SwerveModule backLeft = new SwerveModule(4, 11, 1);
    private final SwerveModule backRight = new SwerveModule(6, 10, 2);

    // private final AnalogGyro gyro = new AnalogGyro(0);
    private final PigeonIMU pigeon = new PigeonIMU(13);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation,
            backLeftLocation, backRightLocation);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics,
            new Rotation2d((pigeon.getCompassHeading() % 360) * Math.PI / 180));

    public Drivetrain() {
        // gyro.reset();
        pigeon.clearStickyFaults();
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                        new Rotation2d((pigeon.getCompassHeading() % 360) * Math.PI / 180))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
        
        //System.out.println("Pigeon orientation: " + pigeon.getCompassHeading() % 360);
        //System.out.println("Front left motor output:\n" + frontLeft.getMotorOutputString());
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        odometry.update(new Rotation2d((pigeon.getCompassHeading() % 360) * Math.PI / 180), frontLeft.getState(),
                frontRight.getState(), backLeft.getState(), backRight.getState());
    }

    public void printTurnMotors() {
        System.out.println("Turning motors:");
        System.out.println("Front-right: " + frontRight.getTurnMotorValue());
        
    }
}
