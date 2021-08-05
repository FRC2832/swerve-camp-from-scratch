// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
    private final XboxController controller = new XboxController(0);
    private final Drivetrain swerve = new Drivetrain();

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3.0);

    @Override
    public void autonomousPeriodic() {
        driveWithJoystick(false);
        swerve.updateOdometry();
    }

    @Override
    public void teleopPeriodic() {
        driveWithJoystick(true);
    }

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
        //System.out.println("X: " + xSpeed + "\tY: " + ySpeed + "\tRot: " + rot);

        swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
        //swerve.printTurnMotors();
    }
}
