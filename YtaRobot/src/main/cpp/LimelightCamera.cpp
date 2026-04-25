////////////////////////////////////////////////////////////////////////////////
/// @file   LimelightCamera.cpp
/// @author David Stalter
///
/// @details
/// A class designed to support limelight camera functionality on the robot.
///
/// Copyright (c) 2026 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
#include <thread>                                   // for thread management

// C INCLUDES
#include "wpinet/PortForwarder.h"                   // for port forwarding

// C++ INCLUDES
#include "LimelightCamera.hpp"                      // for class declaration
#include "frc/smartdashboard/SmartDashboard.h"      // for smart dashboard support


////////////////////////////////////////////////////////////////
/// @method LimelightCamera::LimelightCamera
///
/// Constructor.  This class is instantiated for possible future
/// use with multiple limelight cameras.  For now, most things
/// are hard coded assuming a single camera.  If multiple
/// cameras are ever used, refactoring will be needed (such as
/// passing name, PID values, etc.)
///
////////////////////////////////////////////////////////////////
LimelightCamera::LimelightCamera(std::string cameraName) :
    m_pCameraNetworkTable(nt::NetworkTableInstance::GetDefault().GetTable(cameraName)),
    m_VisionStrafePid(0.025, 0.02, 0.002),
    m_VisionRotatePid(0.025, 0.02, 0.002),
    m_TargetAprilTagId()
{
    // Enable Ethernet port forwarding (but not USB)
    EnableLimelightPortForwarding(true, false, false);

    // Setting constants for the strafe vision PID controller.  Set point
    // is 0.0_deg (centered on target), tolerance is 1.5_deg, and enable
    // continuous input across the Limelight field of view.
    m_VisionStrafePid.SetSetpoint(0.0);
    m_VisionStrafePid.SetTolerance(1.5);
    m_VisionStrafePid.EnableContinuousInput(-27.0, 27.0);

    // Setting constants for the rotate vision PID controller.  Set point
    // is computed later based on the target tag.  Tolerance is one degree.
    // Enable a full 360 input range.
    m_VisionRotatePid.SetSetpoint(40.0);
    m_VisionRotatePid.SetTolerance(1.0);
    m_VisionRotatePid.EnableContinuousInput(0.0, 360.0);
}



////////////////////////////////////////////////////////////////
/// @method LimelightCamera::EnableLimelightPortForwarding
///
/// This method enalbes Limelight port forwarding for the
/// different options of connecting a Limelight.  The expected
/// use case is to enable Ethernet forwarding.  The Limelight
/// 3A/3G can connect over USB, which is when the USB forwarding
/// options would be used.
///
////////////////////////////////////////////////////////////////
void LimelightCamera::EnableLimelightPortForwarding(bool bEnableEthernetForwarding, bool bEnableUsb0Forwarding, bool bEnableUsb1Forwarding)
{
    constexpr const int LIMELIGHT_START_PORT = 5800;
    constexpr const int LIMELIGHT_END_PORT = 5809;
    wpi::PortForwarder & rWpiPortForwarder = wpi::PortForwarder::GetInstance();

    // It's a little inefficient to loop and then check
    // all the enables, but it keeps things a bit neater.
    for (int port = LIMELIGHT_START_PORT; port <= LIMELIGHT_END_PORT; port++)
    {
        // Enable web interface and video stream through the roboRIO
        if (bEnableEthernetForwarding)
        {
            rWpiPortForwarder.Add(port, "limelight.local", port);
        }

        // Enable web interface and video stream when connected via USB
        // For usbIndex 0: ports 5800-5809 forward to 172.29.0.1
        // For usbIndex 1: ports 5810-5819 forward to 172.29.1.1
        // To access the interface of the camera with usbIndex0, go to
        // roboRIO-(teamnum)-FRC.local:5801. Port 5811 for usbIndex1.
        if (bEnableUsb0Forwarding)
        {
            rWpiPortForwarder.Add(port, "172.29.0.1", port);
        }
        if (bEnableUsb1Forwarding)
        {
            constexpr const int USB1_PORT_OFFSET = 10;
            rWpiPortForwarder.Add((port + USB1_PORT_OFFSET), "172.29.1.1", (port + USB1_PORT_OFFSET));
        }
    }
}



////////////////////////////////////////////////////////////////
/// @method LimelightCamera::SetPriorityId
///
/// This method sets the priority ID for which April tag the
/// limelight camera should use.
///
////////////////////////////////////////////////////////////////
void LimelightCamera::SetPriorityId(TaggedFieldElement fieldElement, DriverStation::Alliance alliance)
{
    uint32_t priorityId = 0U;

    // Change this logic and the IDs to game specific values
    switch (fieldElement)
    {
        case LimelightCamera::TaggedFieldElement::ELEMENT_HUB_CENTER:
        {
            priorityId = (alliance == DriverStation::Alliance::kRed) ? 10U : 25U;
            break;
        }
        case LimelightCamera::TaggedFieldElement::ELEMENT_NONE:
        default:
        {
            break;
        }
    }

    m_pCameraNetworkTable->PutNumber("priorityid", priorityId);
}



////////////////////////////////////////////////////////////////
/// @method LimelightCamera::AlignToTargetSwerve
///
/// This method tries to automatically align the robot to a
/// target based on feedback from the camera using swerve drive.
///
////////////////////////////////////////////////////////////////
void LimelightCamera::AlignToTargetSwerve(SwerveDriveLambdaType swerveDriveLambda, units::angle::degree_t currentYawDegrees)
{
    // Get the horizontal offset from the target
    double targetX = m_pCameraNetworkTable->GetNumber("tx", 0.0);

    // Use the PID controller to compute the strafe and rotation values
    double strafe = m_VisionStrafePid.Calculate(targetX);
    double rotation = m_VisionRotatePid.Calculate(currentYawDegrees.value());

    // Get the primary tracked ID
    int primaryTrackedId = m_pCameraNetworkTable->GetNumber("tid", 0.0);

    SmartDashboard::PutNumber("Limelight targetX", targetX);
    SmartDashboard::PutNumber("Limelight raw strafe", strafe);
    SmartDashboard::PutNumber("Limelight primary ID", primaryTrackedId);

    // Clamping strafe output
    strafe = std::clamp(strafe, -0.95, 0.95);
    rotation = std::clamp(rotation, -0.25, 0.25);

    // WPILib recommended feed forward
    // @todo: Is this necessary?
    if (std::abs(strafe) > 0.01)
    {
        strafe += std::copysign(0.02, strafe);
    }

    if (std::abs(rotation) > 0.01)
    {
        rotation += std::copysign(0.02, rotation);
    }

    // Drive
    swerveDriveLambda({0.0_m, units::meter_t{strafe}}, 0.0);
}



////////////////////////////////////////////////////////////////
/// @method LimelightCamera::UpdateSmartDashboard
///
/// Displays some information to the smart dashboard.
///
////////////////////////////////////////////////////////////////
void LimelightCamera::UpdateSmartDashboard()
{
    SmartDashboard::PutNumber(m_CameraName + "heartbeat", m_pCameraNetworkTable->GetNumber("hb", 0.0));
}
