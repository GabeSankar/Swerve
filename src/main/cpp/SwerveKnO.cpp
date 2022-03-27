#include "SwerveKnO.h"

using namespace frc;

//KnO stands for Kinematics and Odometry
SwerveKnO::SwerveKnO(units::length::meter_t startingx, units::length::meter_t startingy, units::angle::degree_t  startingangle,units::angle::degree_t gyroAngle):
//Kinematics Object
kinematics(frontLeftLocation, frontRightLocation,backLeftLocation, backRightLocation),
//Odometry Object
odometry(kinematics, Rotation2d(gyroAngle), Pose2d(startingx, startingy,startingangle))
{
//Chassis objects

//Relative Wheel positions to center(Correct Values Later)
frontLeftLocation = Translation2d(1_m,1_m);
frontRightLocation = Translation2d(1_m, -1_m);
backLeftLocation = Translation2d(-1_m, 1_m);
backRightLocation = Translation2d(-1_m, -1_m);

}
//Columns are in order as frontleft, frontright, backleft, backright and the row is in order of speed and angle
void SwerveKnO::FieldRelativeKinematics(units::velocity::meters_per_second_t xspeed, units::velocity::meters_per_second_t yspeed,
                                        units::radians_per_second_t angularVelocity,units::angle::degree_t CurrentAngle)
{
    /*gives Chassis object a value of the desired conditions so they could be later evaluated via inverse 
    kinematics to attain the wanted states of the modules and return the angle and speed of each wheel in a matrix*/
    speeds = ChassisSpeeds::FromFieldRelativeSpeeds(xspeed,yspeed,angularVelocity, Rotation2d(CurrentAngle));
    //Invese kinematics to assign values to array of module states
     auto [fl, fr, bl, br] = kinematics.ToSwerveModuleStates(speeds);

    frontLeft = fl;
    frontRight = fr;
    backLeft = bl;
    backRight = br;
    
    /*Updates global matrix NOTE: fix later and make more elegant*/
    //Front Left
    motorDataMatrix[0][0] = frontLeft.speed.value();
    motorDataMatrix[0][1] = frontLeft.angle.Degrees().value();
    //Front Right
    motorDataMatrix[1][0] = frontRight.speed.value();
    motorDataMatrix[1][1] = frontRight.angle.Degrees().value();
    //Back Left
    motorDataMatrix[2][0] = backLeft.speed.value();
    motorDataMatrix[2][1] = backLeft.angle.Degrees().value();
    //Back Right
    motorDataMatrix[3][0] = backRight.speed.value();
    motorDataMatrix[3][1] = backRight.angle.Degrees().value();
}
void SwerveKnO::notFieldRelativeKinematics(units::meters_per_second_t  xspeed, units::meters_per_second_t  yspeed, 
                                           units::radians_per_second_t angularVelocity)
{
    speeds = ChassisSpeeds();
    speeds.omega = angularVelocity;
    speeds.vx = xspeed;
    speeds.vy = yspeed;
    //Invese kinematics to assign values to array of module states
    auto [fl, fr, bl, br] = kinematics.ToSwerveModuleStates(speeds);

    frontLeft = fl;
    frontRight = fr;
    backLeft = bl;
    backRight = br;
    /*Updates global matrix NOTE: fix later and make more elegant*/
    //Front Left
    motorDataMatrix[0][0] = frontLeft.speed.value();
    motorDataMatrix[0][1] = frontLeft.angle.Degrees().value();
    //Front Right
    motorDataMatrix[1][0] = frontRight.speed.value();
    motorDataMatrix[1][1] = frontRight.angle.Degrees().value();
    //Back Left
    motorDataMatrix[2][0] = backLeft.speed.value();
    motorDataMatrix[2][1] = backLeft.angle.Degrees().value();
    //Back Right
    motorDataMatrix[3][0] = backRight.speed.value();
    motorDataMatrix[3][1] = backRight.angle.Degrees().value();
}
void SwerveKnO::SwerveOdometryGetPose(units::angle::degree_t gyroAngle)
{
    Rotation2d Angle = Rotation2d(gyroAngle);
    robotPose = odometry.Update(Angle, frontLeft,frontRight,backLeft,backRight);
    PoseVector[0] = robotPose.X().value();
    PoseVector[1] = robotPose.Y().value();
    PoseVector[2] = robotPose.Rotation().Degrees().value();
}