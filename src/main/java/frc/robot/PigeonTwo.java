package frc.robot;

//import com.ctre.phoenix6.ErrorCode;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2Configuration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
/**
 * Class obtained from Hatter's Robotics, FRC Team #708
 *      Updated by #6808 for Phoenix 6 API Library 2024 Season
 */
public class PigeonTwo implements Sendable{
    
    //Pigeon instance (ensure it is a singleton)
    private static PigeonTwo instance = null;

    //Pigeon Inverted Factor, 1.0 not inverted, -1.0 for inverted
    private static final double m_headingInverted = 1.0;
    /**
     * Method to get current instance of Pigeon2 (or establish one if one does not exist)
     * @return Pigeon2 Instance
     */
    public static PigeonTwo getInstance(){
        if(instance == null){
            instance = new PigeonTwo();
        }
        return instance;
    }

    //Pigeon2 object
    private Pigeon2 m_pigeon2;

    //Pigeon2 configuration settings
    private Pigeon2Configuration config = new Pigeon2Configuration();

    //GravityVector object
    private double[] a_gravityVector = new double[3];
    
    private PigeonTwo(){
        try{
            m_pigeon2 = new Pigeon2(Constants.CAN_IDs.PIGEON.ID);
            //Sets the status frame period for two different periods.

            //Default configs = MAKE CONSTANTS IN CONSTANTS FILE & DETERMINE VALUES
            config.Pigeon2Features.DisableNoMotionCalibration = false;
            config.Pigeon2Features.DisableTemperatureCompensation = false;
            config.Pigeon2Features.EnableCompass = true;
            config.MountPose.MountPosePitch = 0;
            config.MountPose.MountPoseRoll  = 0;
            config.MountPose.MountPoseYaw   = 0;

            //Sets pigeon default mountings to values determined above
            m_pigeon2.getConfigurator().apply(config); 

            //Gets gravity vectory and assigns it to a_gravityVector
            //m_pigeon2.getGravityVector(a_gravityVector);
            a_gravityVector[0] = m_pigeon2.getGravityVectorX().getValueAsDouble();
            a_gravityVector[1] = m_pigeon2.getGravityVectorY().getValueAsDouble();
            a_gravityVector[2] = m_pigeon2.getGravityVectorZ().getValueAsDouble();
            
        }catch(Exception e){
            System.out.println("PIGEON INSTANTATION FAILED; THE BIGGEST OOPS HAS OCCURED");
            e.printStackTrace();
        }
        SmartDashboard.putData("Heading", m_pigeon2);

    }

    public Rotation2d getPitch(){
        double pitch = m_pigeon2.getPitch().refresh().getValueAsDouble();
        return Rotation2d.fromDegrees(pitch);
    }

    public Rotation2d getAngle(){
//        double yaw = m_pigeon2.getYaw().refresh().getValueAsDouble();
//        return Rotation2d.fromDegrees(yaw);
        return Rotation2d.fromDegrees(m_pigeon2.getRotation2d().getDegrees()*m_headingInverted);
    }

    public Rotation2d getRoll(){
        double roll = m_pigeon2.getRoll().refresh().getValueAsDouble(); //COMPARE TO getYPR();
        return Rotation2d.fromDegrees(roll);
    }

    public double getRateX(){
        return m_pigeon2.getAngularVelocityXDevice().refresh().getValueAsDouble();
    }

    public double getRateY(){
       return m_pigeon2.getAngularVelocityYDevice().refresh().getValueAsDouble();
    }

    public double getRateZ(){
        //return m_pigeon2.getAngularVelocityZDevice().getValueAsDouble();
        return m_pigeon2.getAngularVelocityZDevice().refresh().getValueAsDouble();
    }

    public double getRawAngle(){
        return (m_pigeon2.getYaw().refresh().getValueAsDouble()*m_headingInverted);
    }

    public double get360Angle(){
        //return m_pigeon2.getYaw().refresh().getValueAsDouble() % 360.0;
        return (m_pigeon2.getRotation2d().getDegrees() % 360.0) * m_headingInverted;
    }

    public StatusCode setAngle(double degrees){
        return m_pigeon2.setYaw(degrees*m_headingInverted);
    }

    public StatusCode reset(){
        return setAngle(0);
    }

    public void getGravityVector(){
        a_gravityVector[0] = m_pigeon2.getGravityVectorX().refresh().getValueAsDouble();
        a_gravityVector[1] = m_pigeon2.getGravityVectorY().refresh().getValueAsDouble();
        a_gravityVector[2] = m_pigeon2.getGravityVectorZ().refresh().getValueAsDouble();
        //return a_gravityVector;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", this::get360Angle, null);
    }
}
