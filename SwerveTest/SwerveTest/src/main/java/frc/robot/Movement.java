package frc.robot;

public class Movement
{
    public Movement(double frontRight, double frontLeft, double backRight, double backLeft)
    {
        FrontLeft = frontLeft;
        FrontRight = frontRight;
        BackLeft = backLeft;
        BackRight = backRight;
    }
    public double FrontRight;   
    public double FrontLeft;
    public double BackRight;
    public double BackLeft;
}