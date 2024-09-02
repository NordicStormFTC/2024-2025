package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.systems.DriveTrain;

@TeleOp

public class TeleOp2024 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);

        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.right_trigger > 0.1) {
                driveTrain.powerMod = 0.4;
            } else if(gamepad1.right_trigger < 0.6) {
                driveTrain.powerMod = 0.9;
            }
            driveTrain.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
//            drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        }

    }

}
