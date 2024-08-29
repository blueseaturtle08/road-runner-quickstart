package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class Manuveringthing extends LinearOpMode {
    private DcMotor leftBackDrive;
    private DcMotor leftFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor rightFrontDrive;
    private IMU imu;
    private double heading;

    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        waitForStart();

        drivething(24);
        turnthing(90);
        drivething(24);
        turnthing(-90);
        drivething(24);
        turnthing(-90);
        drivething(24);
        turnthing(90);
        drivething(24);
        turnthing(90);
        drivething(24);

    }

    public void drivething(int distance) {

        distance = (int) ((distance / Math.PI * 3.54) * 28);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setTargetPosition(distance);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setTargetPosition(distance);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setTargetPosition(distance);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setTargetPosition(distance);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void turnthing(double degrees) {
        double turn = heading + degrees;
        double power;
        if (heading < turn) {
            power = 0.1 * (turn - heading);
            leftFrontDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
        }

    }
}
