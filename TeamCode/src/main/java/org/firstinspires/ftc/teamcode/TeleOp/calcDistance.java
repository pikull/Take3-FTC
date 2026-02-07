package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;



import com.qualcomm.hardware.limelightvision.LLResult;

public class calcDistance extends LinearOpMode {
    double distance;
    int pipline;
    public calcDistance(int piplineNum){
        pipline=piplineNum;
    }
    @Override
    public void runOpMode() throws InterruptedException {


        // LIMELIGHT
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(pipline);
        limelight.start();
        LLResult result ;



        double startMovingArmBackDistFromTarget = 600;


        //INIATE MOTOR POSITIONS

        // CURRENT STATES
        boolean iPower1 = false;


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s", status.getName());
            telemetry.addData("LL", "Temp:%1fC, CPU:%.1f%%,FPS: %d", status.getTemp(), status.getCpu(), (int) status.getFps());
            result = limelight.getLatestResult();
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose_MT2();
                distance = getDistance(result.getTa());
                double tx = result.getTx();
                telemetry.addData("result", tx);
                telemetry.addData("distance",distance);
                telemetry.addData("ta", result.getTa());
                telemetry.addData("botpose:", botpose);
                telemetry.update();
            }

        }

    }
    public static double getDistance(double ta){
        double distance = 67.51967/Math.sqrt(ta);

        //double distance = 110.7079 - (48.58263/0.4940878)*(1 - Math.pow(Math.E,-0.4940878*ta));
        return distance;
    }
}