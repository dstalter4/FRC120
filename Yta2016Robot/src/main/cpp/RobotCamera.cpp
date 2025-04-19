////////////////////////////////////////////////////////////////////////////////
/// @file RobotCamera.cpp
///
/// A class designed to support camera functionality on the robot.
///
/// CMSD FRC 2016
/// Author: David Stalter
/// @Edit History
/// - dts   13-FEB-2016 Created.
///
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "RobotCamera.hpp"          // For class declaration
#include "CameraServer.h"           // For (actually) patched CameraServer instance



////////////////////////////////////////////////////////////////
// @method RobotCamera::RobotCamera
///
/// Constructor.
///
////////////////////////////////////////////////////////////////
RobotCamera::RobotCamera(Camera camera)
: m_Camera(camera)
, m_pAxisCamera()
, m_pUsbCamera()
, m_ImaqSession()
, m_BackImaqSession()
, m_pFrame(imaqCreateImage(IMAQ_IMAGE_RGB, 0))
, m_pBinaryFrame(imaqCreateImage(IMAQ_IMAGE_U8, 0))
, m_TargetReport()
, m_IteratorParicleReport()
, m_ParticleReports()
, m_ReflectRedRange()
, m_ReflectGreenRange()
, m_ReflectBlueRange()
, m_HeartBeat(0)
, m_NumMaskedParticles(0)
, m_NumFilteredParticles(0)
, m_CameraDistance(0.0F)
, m_GroundDistance(0.0F)
, m_BoundingArea(0.0F)
, m_ShapeAreaPercent(0.0F)
, m_TrapezoidPercent(0.0F)
, m_bUsbCameraPresent(false)
, m_bBackUsbCameraPresent(false)
, m_bTargetInRange(false)
{
    // Reset and prepare the Axis camera
    //m_pAxisCamera.reset(new AxisCamera(AXIS_CAMERA_IP_STRING));
    
    //m_pUsbCamera = new USBCamera(USB_CAMERA_STRING, true);
    //m_pUsbCamera->OpenCamera();
    //m_pUsbCamera->SetFPS(CAMERA_FPS);
    //m_pUsbCamera->SetSize(CAMERA_WIDTH, CAMERA_HEIGHT);
    //m_pUsbCamera->UpdateSettings();
    //m_pUsbCamera->StartCapture();
    //m_bUsbCameraPresent = true;
    
    // Make sure there is actually a USB camera present at the specified port
    if (IMAQdxOpenCamera(USB_CAMERA_STRING, IMAQdxCameraControlModeController, &m_ImaqSession) == IMAQdxErrorSuccess)
    {
        IMAQdxConfigureGrab(m_ImaqSession);
        IMAQdxStartAcquisition(m_ImaqSession);
        m_bUsbCameraPresent = true;
    }
    
    if (IMAQdxOpenCamera(BACK_USB_CAMERA_STRING, IMAQdxCameraControlModeController, &m_BackImaqSession) == IMAQdxErrorSuccess)
    {
        IMAQdxConfigureGrab(m_BackImaqSession);
        IMAQdxStartAcquisition(m_BackImaqSession);
        m_bBackUsbCameraPresent = true;
    }
    
    // Initialize the ranges, they aren't constant in case the Smart Dashboard sets them
    m_ReflectRedRange = {RED_REFLECT_MIN, RED_REFLECT_MAX};
    m_ReflectGreenRange = {GREEN_REFLECT_MIN, GREEN_REFLECT_MAX};
    m_ReflectBlueRange = {BLUE_REFLECT_MIN, BLUE_REFLECT_MAX};
}



////////////////////////////////////////////////////////////////
// @method RobotCamera::UpdateSmartDashboard
///
/// This method sends new data to the C++ Smart Dashboard.
///
////////////////////////////////////////////////////////////////
void RobotCamera::UpdateSmartDashboard()
{
    SmartDashboard::PutNumber("Reflect red min",    m_ReflectRedRange.minValue);
    SmartDashboard::PutNumber("Reflect red max",    m_ReflectRedRange.maxValue);
    SmartDashboard::PutNumber("Reflect green min",  m_ReflectGreenRange.minValue);
    SmartDashboard::PutNumber("Reflect green max",  m_ReflectGreenRange.maxValue);
    SmartDashboard::PutNumber("Reflect blue min",   m_ReflectBlueRange.minValue);
    SmartDashboard::PutNumber("Reflect blue max",   m_ReflectBlueRange.maxValue);
    SmartDashboard::PutNumber("Masked particles",   m_NumMaskedParticles);
    SmartDashboard::PutNumber("Filtered particles", m_NumFilteredParticles);
    SmartDashboard::PutNumber("HeartBeat",          m_HeartBeat++);
    SmartDashboard::PutNumber("Left bound",         m_TargetReport.m_BoundingRectLeft);
    SmartDashboard::PutNumber("Right bound",        m_TargetReport.m_BoundingRectRight);
    SmartDashboard::PutNumber("Top bound",          m_TargetReport.m_BoundingRectTop);
    SmartDashboard::PutNumber("Bottom bound",       m_TargetReport.m_BoundingRectBottom);
    SmartDashboard::PutNumber("ConvexHull",         m_TargetReport.m_ConvexHullArea);
    SmartDashboard::PutNumber("Bounding area",      m_BoundingArea);
    SmartDashboard::PutNumber("Shape area",         m_TargetReport.m_Area);
    SmartDashboard::PutNumber("Shape area %",       m_ShapeAreaPercent);
    SmartDashboard::PutNumber("Trapezoid score",    m_TrapezoidPercent);
    SmartDashboard::PutNumber("Camera distance",    m_CameraDistance);
    SmartDashboard::PutNumber("Ground distance",    m_GroundDistance);

    m_ReflectRedRange.minValue      = SmartDashboard::GetNumber("Reflect red min",      m_ReflectRedRange.minValue);
    m_ReflectRedRange.maxValue      = SmartDashboard::GetNumber("Reflect red max",      m_ReflectRedRange.maxValue);
    m_ReflectGreenRange.minValue    = SmartDashboard::GetNumber("Reflect green min",    m_ReflectGreenRange.minValue);
    m_ReflectGreenRange.maxValue    = SmartDashboard::GetNumber("Reflect green max",    m_ReflectGreenRange.maxValue);
    m_ReflectBlueRange.minValue     = SmartDashboard::GetNumber("Reflect blue min",     m_ReflectBlueRange.minValue);
    m_ReflectBlueRange.maxValue     = SmartDashboard::GetNumber("Reflect blue max",     m_ReflectBlueRange.maxValue);
}



////////////////////////////////////////////////////////////////
// @method RobotCamera::ProcessTarget
///
/// The public interface method for getting information about
/// the target.
///
////////////////////////////////////////////////////////////////
bool RobotCamera::ProcessTarget(bool bDoFullProcessing)
{
    GetAndDisplayImage();
    
    // And if m_bUsbCameraPresent?
    if (bDoFullProcessing)
    {
        FilterImage();
        
        GenerateParticleReport();
        
        CalculateTargetParticleValues();
        
        // Don't call this in production code - it hogs resources
        //UpdateSmartDashboard();
    }

    return m_bTargetInRange;

    // Shape drawing coordinates: Top, Left, Height, Width
    //int L = std::trunc(m_TargetReport.BoundingRectLeft);
    //int R = std::trunc(m_TargetReport.BoundingRectRight);
    //int T = std::trunc(m_TargetReport.BoundingRectTop);
    //int B = std::trunc(m_TargetReport.BoundingRectBottom);
    //int W = std::trunc(R - L);
    //int H = std::trunc(B - T);
    //m_pAxisCamera->GetImage(m_pImageFrame);
    //imaqDrawShapeOnImage(frame, frame, { 10, 10, 100, 100 }, DrawMode::IMAQ_DRAW_VALUE, ShapeMode::IMAQ_SHAPE_OVAL, 0.0f);
    //imaqDrawShapeOnImage(m_pImageFrame, m_pImageFrame, { T, L, H, W }, DrawMode::IMAQ_DRAW_VALUE, ShapeMode::IMAQ_SHAPE_OVAL, 1.0f);
    //CameraServer::GetInstance()->SetImage(pDrawableImage);
    //AthenaCameraServer::GetInstance()->SetImage(pDrawableImage);
}



////////////////////////////////////////////////////////////////
// @method RobotCamera::GetAndDisplayImage
///
/// Gets an image from the currently selected camera and sends
/// it to the driver station.
///
////////////////////////////////////////////////////////////////
void RobotCamera::GetAndDisplayImage()
{
    // Grab an image from the camera
    if (m_Camera == AXIS)
    {
        static_cast<void>(m_pAxisCamera->GetImage(m_pFrame));
    }
    else if (m_Camera == USB)
    {
        static_cast<void>(IMAQdxGrab(m_ImaqSession, m_pFrame, true, NULL));
        //m_pUsbCamera->GetImage(m_pFrame);
    }
    else if (m_Camera == BACK_USB)
    {
        static_cast<void>(IMAQdxGrab(m_ImaqSession, m_pFrame, true, NULL));
    }
    else
    {
    }
    
    // Send the image to the driver station
    //CameraServer::GetInstance()->SetImage(m_pFrame);
    CameraServer::GetInstance()->SetImage(m_pFrame);
}

    
    
////////////////////////////////////////////////////////////////
// @method RobotCamera::FilterImage
///
/// Starts to filter an image.  It applies a color filter to the
/// image, then counts and filters the particles.
///
////////////////////////////////////////////////////////////////
void RobotCamera::FilterImage()
{
    // Threshold the image looking for the illuminated green reflective tape
    static_cast<void>(imaqColorThreshold(m_pBinaryFrame, m_pFrame, IMAQ_REPLACE_VALUE, IMAQ_HSV, &m_ReflectRedRange, &m_ReflectGreenRange, &m_ReflectBlueRange));

    // Send particle count to dashboard
    static_cast<void>(imaqCountParticles(m_pBinaryFrame, 1, &m_NumMaskedParticles));
    
    //CameraServer::GetInstance()->SetImage(m_pFrame);
    //AthenaCameraServer::GetInstance()->SetImage(m_pFrame);

    // Filter out small particles
    static_cast<void>(imaqParticleFilter4(m_pBinaryFrame, m_pBinaryFrame, &FILTER_CRITERIA, 1, &FILTER_OPTIONS, NULL, NULL));

    // Send particle count after filtering to dashboard
    static_cast<void>(imaqCountParticles(m_pBinaryFrame, 1, &m_NumFilteredParticles));
}



////////////////////////////////////////////////////////////////
// @method RobotCamera::GenerateParticleReport
///
/// This method iterates over the found particles and performs
/// certain calculations on it.  It saves off the report of the
/// target with the largest area.
///
////////////////////////////////////////////////////////////////
void RobotCamera::GenerateParticleReport()
{
    if (m_NumFilteredParticles > 0)
    {
        double currentMaxArea = 0;

        // Measure particles and sort by particle size
        for (int particleIndex = 0; particleIndex < m_NumFilteredParticles; particleIndex++)
        {
            imaqMeasureParticle(m_pBinaryFrame, particleIndex, 0, IMAQ_MT_AREA_BY_IMAGE_AREA,   &(m_IteratorParicleReport.m_PercentAreaToImageArea));
            imaqMeasureParticle(m_pBinaryFrame, particleIndex, 0, IMAQ_MT_AREA,                 &(m_IteratorParicleReport.m_Area));
            imaqMeasureParticle(m_pBinaryFrame, particleIndex, 0, IMAQ_MT_CONVEX_HULL_AREA,     &(m_IteratorParicleReport.m_ConvexHullArea));
            imaqMeasureParticle(m_pBinaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_TOP,    &(m_IteratorParicleReport.m_BoundingRectTop));
            imaqMeasureParticle(m_pBinaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_LEFT,   &(m_IteratorParicleReport.m_BoundingRectLeft));
            imaqMeasureParticle(m_pBinaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_BOTTOM, &(m_IteratorParicleReport.m_BoundingRectBottom));
            imaqMeasureParticle(m_pBinaryFrame, particleIndex, 0, IMAQ_MT_BOUNDING_RECT_RIGHT,  &(m_IteratorParicleReport.m_BoundingRectRight));
            m_ParticleReports.push_back(m_IteratorParicleReport);

            m_IteratorParicleReport.m_AspectRatio = (m_IteratorParicleReport.m_BoundingRectRight - m_IteratorParicleReport.m_BoundingRectLeft) /
                                                    (m_IteratorParicleReport.m_BoundingRectBottom - m_IteratorParicleReport.m_BoundingRectTop);

            if (m_IteratorParicleReport.m_Area > currentMaxArea)
            {
                currentMaxArea = m_IteratorParicleReport.m_Area;
                m_TargetReport = m_IteratorParicleReport;
            }
        }
    }

    // Concerns about using too much of the heap and frequent dynamic garbage collection?
    m_ParticleReports.erase(m_ParticleReports.begin(), m_ParticleReports.end());
}



////////////////////////////////////////////////////////////////
// @method RobotCamera::GenerateParticleReport
///
/// This method performs certain calculations on the best found
/// particle report.  It will figure out distances to the target
/// and return true if it finds a particle within the expected
/// range.
///
////////////////////////////////////////////////////////////////
void RobotCamera::CalculateTargetParticleValues()
{
    int32_t xRes = 0;
    int32_t yRes = 0;
    imaqGetImageSize(m_pBinaryFrame, &xRes, &yRes);

    // Target distance: 11-16 ft. (132-192 in.)
    // Bottom of goal is 7 ft. (84 in.) from ground

    // d = (TLengthIn * xRes) / (2 * TLengthPix * tan(FOVAng)), negated
    //double distance = -1 * (20 * xRes) / (2 * (targetReport.BoundingRectRight - targetReport.BoundingRectLeft) * tan(18));//tan(21.5778173));//tan(37.4));
    m_CameraDistance = (TARGET_SIZE * xRes) /
                       (2.0F * (m_TargetReport.m_BoundingRectRight - m_TargetReport.m_BoundingRectLeft)
                             * tan(CALIBRATED_CAMERA_ANGLE*RADIANS_TO_DEGREES));

    // ground_distance = sqrt((camera_reported_distance^2) - (84^2))
    // sin(camera_angle) = (height_from_ground) / (camera_reported_distance);
    m_GroundDistance = sqrt((m_CameraDistance * m_CameraDistance) - (TARGET_REFLECTOR_HEIGHT * TARGET_REFLECTOR_HEIGHT));

    m_BoundingArea = (m_TargetReport.m_BoundingRectRight-m_TargetReport.m_BoundingRectLeft)
                   * (m_TargetReport.m_BoundingRectBottom-m_TargetReport.m_BoundingRectTop);
    m_ShapeAreaPercent = (m_TargetReport.m_Area / m_BoundingArea) * DECIMAL_TO_PERCENT;
    m_TrapezoidPercent = (m_TargetReport.m_ConvexHullArea / m_BoundingArea) * DECIMAL_TO_PERCENT;

    // At a distance of 20 feet, the minimum area for the target is about 700 pxl^2
    // Our target range is 11-16 ft. so we will use this as our starting filtering point
    if (((m_GroundDistance + GROUND_DISTANCE_TOLERANCE) >= TARGET_RANGE_MIN) && ((m_GroundDistance - GROUND_DISTANCE_TOLERANCE) <= TARGET_RANGE_MAX))
    {
        m_bTargetInRange = true;
    }
    else
    {
        m_bTargetInRange = false;
    }
}
