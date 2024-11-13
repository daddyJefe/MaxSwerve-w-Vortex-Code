package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoAlignToTargetCommand extends Command {
    private final DriveSubsystem m_drive;
    private final LimelightSubsystem m_limelight;
    private final PIDController m_pidController;

    public AutoAlignToTargetCommand(DriveSubsystem drive, LimelightSubsystem limelight) {
        this.m_drive = drive;
        this.m_limelight = limelight;

        // Create a PID controller to rotate the robot
        this.m_pidController = new PIDController(0.05, 0, 0);  // Tune the PID values as needed
        m_pidController.setTolerance(1); // Tolerance for when the robot is "aligned" to the target
        addRequirements(m_drive); // Require the drive subsystem for this command
    }

    @Override
    public void initialize() {
        // Reset PID controller when the command starts
        m_pidController.reset();
    }

    @Override
    public void execute() {

       
         double ta = m_limelight.getTA();
         System.out.println("TA IS " + ta);
        // Get the horizontal offset from the Limelight (degrees to the target)
        double tx = m_limelight.getTX();
SmartDashboard.putNumber("blah", tx);
        // Use the PID controller to calculate the output for turning the robot
        double turnSpeed = m_pidController.calculate(tx, 0);  // Target angle is 0 (aligned)
        System.out.println("attempting to perform action: rotate " + turnSpeed);
        // Apply the calculated turn speed to the robot

       


        m_drive.drive(0, 0, turnSpeed/4, false, false);  // No forward motion, just turning
    }

    @Override
    public boolean isFinished() {
        // Finish when the robot is within the tolerance of the target angle
        return m_pidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        m_drive.drive(0, 0, 0, false, false);
    }
}
