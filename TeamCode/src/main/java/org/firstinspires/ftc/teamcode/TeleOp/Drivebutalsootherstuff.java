package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Drivebutalsootherstuff extends LinearOpMode {


    @Override
    public void runOpMode() {
        DcMotor frontLeftDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor backLeftDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor backRightDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        boolean buttonBPressed = false;

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            if (gamepad1.options) {
                imu.resetYaw();
            }
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            double max = Math.max(Math.abs(x) + Math.abs(y), 1);
            double leftDrivePower = (y + x) / max;
            double rightDrivePower = (y - x) / max;

            frontLeftDrive.setPower(leftDrivePower);
            backLeftDrive.setPower(leftDrivePower);
            frontRightDrive.setPower(rightDrivePower);
            frontRightDrive.setPower(rightDrivePower);

            double turn = 0;
            if (gamepad1.b && !buttonBPressed) {
                buttonBPressed = true;
                turn = heading + 180;
            }
            double p;
            double i = 0;
            double d;
            double heading1 = 0;
            double power;
            while (buttonBPressed) {
                if (heading < turn) {
                    p = turn - heading;
                    i += heading;
                    d = heading1 - heading;
                    heading1 = heading;
                    power = 1 * p + 1 * i + 1 * d;
                    frontLeftDrive.setPower(-power);
                    backLeftDrive.setPower(-power);
                    frontRightDrive.setPower(power);
                    frontRightDrive.setPower(power);
                } else buttonBPressed = false;
            }

            telemetry.addData("left stick y", y);
            telemetry.addData("heading", heading);
            telemetry.addData("turn", turn);
            telemetry.addData("loop", buttonBPressed);
            telemetry.update();
        }
    }
}