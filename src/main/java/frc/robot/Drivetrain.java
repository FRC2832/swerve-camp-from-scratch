// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
    public static final int FL = 0;
    public static final int FR = 1;
    public static final int RL = 2;
    public static final int RR = 3;

    public static final double kMaxSpeed = 3.0; // 3 meter(s) per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    // locations of the 4 wheel contact points
    private final Translation2d[] locations = {
                                       /* X , Y */
            /* FL */ new Translation2d(-0.381,  0.381), 
            /* FR */ new Translation2d( 0.381,  0.381),
            /* RL */ new Translation2d(-0.381, -0.381), 
            /* RR */ new Translation2d( 0.381, -0.381) };

    private final SwerveModule[] modules = { 
        /* FL */ new SwerveModule(7, 8, 3), 
        /* FR */ new SwerveModule(5, 9, 0),
        /* RL */ new SwerveModule(4, 11, 1), 
        /* RR */ new SwerveModule(6, 10, 2) 
    };

    private final String[] moduleNames = {
        "SwerveDrive_FL",
        "SwerveDrive_FR",
        "SwerveDrive_RL",
        "SwerveDrive_RR"
    };

    private Pose2d[] modulePoses = new Pose2d[4];

    // private final AnalogGyro gyro = new AnalogGyro(0);
    private final PigeonIMU pigeon = new PigeonIMU(13);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        locations[FL], locations[FR], locations[RL], locations[RR]);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getHeading());

    private Field2d field = new Field2d();

    public Drivetrain() {
        // gyro.reset();
        pigeon.clearStickyFaults();
        SmartDashboard.putData("Field", field);
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
        // ask the kinematics to determine our swerve command
        ChassisSpeeds speeds;
        if (fieldRelative == true) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds);

        // sometime the Kinematics spits out too fast of speeds, so this will fix this
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);

        // command each swerve module
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(swerveModuleStates[i]);
        }

        // report our commands to the dashboard
        SmartDashboard.putNumber("SwerveDrive/xSpeed", xSpeed);
        SmartDashboard.putNumber("SwerveDrive/ySpeed", ySpeed);
        SmartDashboard.putNumber("SwerveDrive/rot", rot);
        SmartDashboard.putBoolean("SwerveDrive/fieldRelative", fieldRelative);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        // update our estimation where we are on the field
        odometry.update(getHeading(), modules[FL].getState(),
                modules[FR].getState(), modules[RL].getState(), modules[RR].getState());
        Pose2d pose = getPose();

        // Update the poses for the swerveModules. Note that the order of rotating the
        // position and then adding the translation matters
        for (int i = 0; i < modules.length; i++) {
            var modulePositionFromChassis = locations[i].rotateBy(getHeading())
                    .plus(pose.getTranslation());

            // Module's heading is it's angle relative to the chassis heading
            modulePoses[i] = new Pose2d(modulePositionFromChassis,
                    modules[i].getState().angle.plus(pose.getRotation()));
        }

        // plot it on the simulated field
        field.setRobotPose(pose);
        field.getObject("Swerve Modules").setPoses(modulePoses);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Rotation2d getHeading() {
        //TODO: can we get rid of this allocation
        return new Rotation2d(Math.toRadians(getAngle()));
    }

    /**
     * Gets the robot's angle in degrees
     * @return Robot's angle since powerup
     */
    public double getAngle() {
        return pigeon.getCompassHeading();
    }

    @Override
    public void periodic() {
        //put data on dashboard
        SmartDashboard.putNumber("SwerveDrive/gyroAngle", getAngle());
        SmartDashboard.putNumber("SwerveDrive/gyroHeading", getHeading().getDegrees());
        for(int i=0; i< modules.length; i++) {
            modules[i].putSmartDashboard(moduleNames[i]);
        }
    }

    @Override
    public void simulationPeriodic() {
        double rate = Robot.kDefaultPeriod;
        SwerveModuleState[] states = new SwerveModuleState[4];

        //run the simulation to get the module's velocity/angle
        for(int i=0; i< modules.length; i++) {
            modules[i].simulationPeriodic(rate);
            states[i] = modules[i].getState();
        }
        
        //calculate the robot's speed and angle (we only care about angle here)
        double omega = kinematics.toChassisSpeeds(states).omegaRadiansPerSecond;
        //set the IMU to the calculated robot rotation
        pigeon.addYaw(Math.toDegrees(omega) * rate);
    }
}
