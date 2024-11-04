package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.DriveForInchesY;
import org.firstinspires.ftc.teamcode.commands.Pivot;
import org.firstinspires.ftc.teamcode.systems.DriveTrain;

public class AutoCommandStack extends SequentialCommandGroup {

    // this is where we make the singular sequential command to be run by the autonomous.
    // here we add the entire autonomouse command sequence
    public AutoCommandStack(DriveTrain driveTrain){
        addCommands(
               // new DriveForInchesY(24, driveTrain, 1000)
                //new Pivot(90, driveTrain)
               // new DriveForInchesY(-20,driveTrain,300)
                );
    }
}
