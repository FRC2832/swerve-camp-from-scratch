// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private final XboxController controller = new XboxController(0);
    private final Drivetrain swerve = new Drivetrain();

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3.0);

    private boolean lastEnabled = false;

    @Override
    public void robotInit() {
        CommandScheduler.getInstance().registerSubsystem(swerve);
        this.setNetworkTablesFlushEnabled(true);
        LiveWindow.setEnabled(false);
    }

    @Override
    public void autonomousPeriodic() {
        driveWithJoystick(false);
    }

    @Override
    public void teleopPeriodic() {
        driveWithJoystick(true);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        //have the field position constantly update
        swerve.updateOdometry();

        //automatically turn on/off recording
        if(lastEnabled != isEnabled()) {
            //we know the enabled status changed
            if(lastEnabled == false) {
                //robot started, start recording
                Shuffleboard.startRecording();
            } else {
                //robot stopped, stop recording
                Shuffleboard.stopRecording();
            }
        }
        //save the result for next loop
        lastEnabled = isEnabled();

        //temp variable just to see how fast the update rate is in ShuffleBoard
        SmartDashboard.putNumber("Loop", loop++);
    }
    private int loop = 0;

    private void driveWithJoystick(boolean fieldRelative) {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final double xSpeed = -xSpeedLimiter.calculate(controller.getY(GenericHID.Hand.kLeft))
                * frc.robot.Drivetrain.kMaxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final double ySpeed = -ySpeedLimiter.calculate(controller.getX(GenericHID.Hand.kLeft))
                * frc.robot.Drivetrain.kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final double rot = -rotLimiter.calculate(controller.getX(GenericHID.Hand.kRight))
                * frc.robot.Drivetrain.kMaxAngularSpeed;

        // ask the drivetrain to run
        swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
    }
}
