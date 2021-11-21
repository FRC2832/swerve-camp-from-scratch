package frc.robot.Commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain;

public class DriveStick extends CommandBase {
    private Drivetrain drive;
    private XboxController controller;

    public DriveStick(Drivetrain drive, XboxController controller) {
        this.drive = drive;
        this.controller = controller;
        addRequirements(drive);
    }

    public void initialize() {

    }

    public void execute() {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        double xSpeed = -controller.getY(GenericHID.Hand.kLeft)
                * frc.robot.Drivetrain.kMaxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        double ySpeed = -controller.getX(GenericHID.Hand.kLeft)
                * frc.robot.Drivetrain.kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        double rot = -controller.getX(GenericHID.Hand.kRight)
                * frc.robot.Drivetrain.kMaxAngularSpeed;

        // ask the drivetrain to run
        drive.drive(xSpeed, ySpeed, rot, true);
    }
}
