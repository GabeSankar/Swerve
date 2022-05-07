#pragma once
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <units/angle.h>
#include <frc/geometry/Translation2d.h>
#include <units/base.h>
#include <units/length.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include "Constants.h"

using namespace frc;

//KnO stands for Kinematics and Odometry
class SwerveKnO{
    public:
    SwerveKnO(units::length::meter_t startingx,units::length::meter_t startingy, units::angle::radian_t startingangle,units::angle::radian_t gyroAngle);
    void FieldRelativeKinematics(units::velocity::meters_per_second_t xspeed, units::velocity::meters_per_second_t yspeed,
                                 units::radians_per_second_t angularVelocity,units::angle::radian_t CurrentAngle);
    void notFieldRelativeKinematics(units::velocity::meters_per_second_t xspeed, units::velocity::meters_per_second_t yspeed, 
                                    units::radians_per_second_t angularVelocity);
    void SwerveOdometryGetPose(units::angle::radian_t gyroAngle);
    //States
    SwerveModuleState frontLeft;
    SwerveModuleState frontRight;
    SwerveModuleState backLeft;
    SwerveModuleState backRight;
    //Rows consist of motor data columns consist of motor unins ROWS: Meters per second, degrees
    double motorDataMatrix[4][2];
    //Vector for pose note that it is x, y, and rotation in degrees
    double PoseVector[3];
    

    private:
    SwerveDriveKinematics<4> kinematics;
    SwerveDriveOdometry<4> odometry;
    Translation2d frontLeftLocation,backLeftLocation;
    Translation2d frontRightLocation,backRightLocation;
    ChassisSpeeds speeds;
    Pose2d robotPose;
    Rotation2d Angle;

};