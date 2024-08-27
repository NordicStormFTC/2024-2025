package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.DriveTrain;

public class TeleOp2024 extends LinearOpMode {

    private DriveTrain drivetrain = new DriveTrain(hardwareMap, this);

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        drivetrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }

}
