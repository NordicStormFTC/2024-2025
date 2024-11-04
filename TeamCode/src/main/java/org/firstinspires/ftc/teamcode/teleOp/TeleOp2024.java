package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.systems.DriveTrain;
import org.firstinspires.ftc.teamcode.systems.DriveTrain2;

@TeleOp

public class TeleOp2024 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
      DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
        final double odoOffset = driveTrain.backDriveEncoder.getCurrentPosition();

                waitForStart();
        while(opModeIsActive()){

//            driveTrain.intake.setDirection(Servo.Direction.FORWARD);
//            driveTrain.spit.setDirection(Servo.Direction.FORWARD);
//
//            driveTrain.intake.setPosition(-1);
//            driveTrain.spit.setPosition(-1);

            //driveTrain.spit.setPosition(1);
            double target = 100;
            double rollingPos = ( driveTrain.backDriveEncoder.getCurrentPosition()) - odoOffset;
            double error = target - rollingPos;
            double velocity = driveTrain.imu.getVelocity().zVeloc;
            telemetry.addData("error", error);
            telemetry.addData("rollingPos",rollingPos);
            telemetry.addData("Vvelocity",velocity);
            telemetry.update();
            //driveTrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 1);
//            if(gamepad1.right_trigger > .6){
//                driveTrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, .4);
//
//            } else if(gamepad1.right_trigger < .6)
//            driveTrain.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 1);
        }
    }


}
