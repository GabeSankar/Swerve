#include "SwerveKnO.h"

using namespace frc;

//KnO stands for Kinematics and Odometry
SwerveKnO::SwerveKnO(units::length::meter_t startingx, units::length::meter_t startingy, units::angle::radian_t  startingangle,units::angle::radian_t gyroAngle):
//Kinematics Object
kinematics(frontLeftLocation, frontRightLocation,backLeftLocation, backRightLocation),
//Odometry Object
odometry(kinematics, Rotation2d(gyroAngle), Pose2d(startingx, startingy,startingangle))
{
//Chassis objects

//Relative Wheel positions to center(Correct Values Later)
frontLeftLocation = Translation2d(34.25_m,34.25_m);
frontRightLocation = Translation2d(34.25_m, -34.25_m);
backLeftLocation = Translation2d(-34.25_m, 34.25_m);
backRightLocation = Translation2d(-34.25_m, -34.25_m);

}
//Columns are in order as frontleft, frontright, backleft, backright and the row is in order of speed and angle
void SwerveKnO::FieldRelativeKinematics(units::velocity::meters_per_second_t xspeed, units::velocity::meters_per_second_t yspeed,
                                        units::radians_per_second_t angularVelocity,units::angle::radian_t CurrentAngle)
{
    //For Linear Motion
    
    /*gives Chassis object a value of the desired conditions so they could be later evaluated via inverse 
    kinematics to attain the wanted states of the modules and return the angle and speed of each wheel in a matrix*/
    speeds = ChassisSpeeds::FromFieldRelativeSpeeds(xspeed,yspeed,angularVelocity, Rotation2d(CurrentAngle));
    //Invese kinematics to assign values to array of module states
    auto states = kinematics.ToSwerveModuleStates(speeds);
    auto [fl, fr, bl, br] = states;
    //referance t the address array
    kinematics.DesaturateWheelSpeeds(&states,(MAXSPEED*1_mps));

    frontLeft = fl;
    frontRight = fr;
    backLeft = bl;
    backRight = br;
    

}
void SwerveKnO::notFieldRelativeKinematics(units::meters_per_second_t  xspeed, units::meters_per_second_t  yspeed, 
                                           units::radians_per_second_t angularVelocity)
{
    speeds = ChassisSpeeds();
    speeds.omega = angularVelocity;
    speeds.vx = xspeed;
    speeds.vy = yspeed;
    //Invese kinematics to assign values to array of module states
    auto states = kinematics.ToSwerveModuleStates(speeds);
    auto [fl, fr, bl, br] = states;
    //referance t the address array
     kinematics.DesaturateWheelSpeeds(&states,(MAXSPEED*1_mps));

    frontLeft = fl;
    frontRight = fr;
    backLeft = bl;
    backRight = br;
   
}
void SwerveKnO::SwerveOdometryGetPose(units::angle::radian_t gyroAngle)
{
    Rotation2d Angle = Rotation2d(gyroAngle);
    robotPose = odometry.Update(Angle, frontLeft,frontRight,backLeft,backRight);
    PoseVector[0] = robotPose.X().value();
    PoseVector[1] = robotPose.Y().value();
    PoseVector[2] = robotPose.Rotation().Radians().value();
}