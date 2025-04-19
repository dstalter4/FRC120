////////////////////////////////////////////////////////////////////////////////
/// @file RobotCamera.hpp
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
#include "WPILib.h"         // For FRC library

// C++ INCLUDES
// (none)

////////////////////////////////////////////////////////////////
// @class RobotCamera
///
/// Class that provides methods for interacting with the camera.
///
////////////////////////////////////////////////////////////////
class RobotCamera
{
public:
    enum Camera
    {
        AXIS,
        USB,
        BACK_USB
    };

    // Pick a camera to use
    inline void SetCamera(Camera camera);

    // Main processing sequence
    bool ProcessTarget(bool bDoFullProcessing);

    // Constructor
    RobotCamera(Camera camera);

private:
    // Update values on the SmartDashboard
    void UpdateSmartDashboard();
    
    // Gets an image and sends it to the SmartDashboard
    void GetAndDisplayImage();

    // Get an image and start to process it
    void FilterImage();

    // Generate a report about the found particles in an image
    void GenerateParticleReport();

    // Compute some useful information about the identified particle
    void CalculateTargetParticleValues();

    // Destructor, copy constructor, assignment operator
    ~RobotCamera()
    {
        //IMAQdxStopAcquisition(m_ImaqSession);
        m_pUsbCamera->StopCapture();
        m_pUsbCamera->CloseCamera();
    }

    RobotCamera(const RobotCamera &);
    RobotCamera & operator=(const RobotCamera &);

    // A structure to hold measurements of a particle
    struct ParticleReport
    {
        double m_PercentAreaToImageArea;
        double m_Area;
        double m_ConvexHullArea;
        double m_BoundingRectLeft;
        double m_BoundingRectTop;
        double m_BoundingRectRight;
        double m_BoundingRectBottom;
        double m_AspectRatio;
    };
    
    Camera m_Camera;                                    // Keep track of the current camera to process information from
    std::unique_ptr<AxisCamera> m_pAxisCamera;          // Axis camera pointer
    USBCamera * m_pUsbCamera;                           // USB camera pointer
    IMAQdxSession m_ImaqSession;                        // Session for the USB camera
    IMAQdxSession m_BackImaqSession;                    // Session for the back USB camera
    Image * m_pFrame;                                   // Image frame from the camera
    Image * m_pBinaryFrame;                             // Binary processed version of the camera image frame
    ParticleReport m_TargetReport;                      // Information about the best target particle we've found
    ParticleReport m_IteratorParicleReport;             // An object to iterate over objects when identifying a target
    std::vector<ParticleReport> m_ParticleReports;      // The list of particle reports

    Range m_ReflectRedRange;
    Range m_ReflectGreenRange;
    Range m_ReflectBlueRange;

    const ParticleFilterOptions2 FILTER_OPTIONS = {IMAQ_FILTER_REJECT_MATCHES, IMAQ_FILTER_REJECT_BORDERS, IMAQ_FILTER_FILL_HOLES, IMAQ_FILTER_USE_CONNECTIVITY_8};
    const ParticleFilterCriteria2 FILTER_CRITERIA = {IMAQ_MT_AREA_BY_IMAGE_AREA, TARGET_MIN_AREA_PERCENT, TARGET_MAX_AREA_PERCENT, false, false};

    int m_HeartBeat;                // Keep alive with the C++ dashboard
    int m_NumMaskedParticles;       // Number of masked particles found
    int m_NumFilteredParticles;     // Number of filtered particles found

    float m_CameraDistance;         // Distance to the target from the camera
    float m_GroundDistance;         // Actual ground distance to the target
    float m_BoundingArea;           // Area of the target based on its bounding coordinates
    float m_ShapeAreaPercent;       // Percentage of the bounding area the shape occupies
    float m_TrapezoidPercent;       // Likelihood that this is a true rectangle

    bool m_bUsbCameraPresent;       // Remember if the USB camera startup was successful
    bool m_bBackUsbCameraPresent;   // Remember if the back USB camera startup was successful
    bool m_bTargetInRange;          // Remember the last result from full vision processing
    
    const char * AXIS_CAMERA_IP_STRING                  = "10.1.20.9";
    const char * USB_CAMERA_STRING                      = "cam1";
    const char * BACK_USB_CAMERA_STRING                 = "cam2";

    static const int RED_REFLECT_MIN                    = 94;
    static const int RED_REFLECT_MAX                    = 136;
    static const int GREEN_REFLECT_MIN                  = 156;
    static const int GREEN_REFLECT_MAX                  = 255;
    static const int BLUE_REFLECT_MIN                   = 233;
    static const int BLUE_REFLECT_MAX                   = 255;
    static const int IMAQ_REPLACE_VALUE                 = 255;
    static const int IMAQ_FILTER_REJECT_MATCHES         = 0;
    static const int IMAQ_FILTER_REJECT_BORDERS         = 0;
    static const int IMAQ_FILTER_FILL_HOLES             = 1;
    static const int IMAQ_FILTER_USE_CONNECTIVITY_8     = 1;
    static const unsigned int CAMERA_WIDTH              = 320;
    static const unsigned int CAMERA_HEIGHT             = 240;
    static constexpr float CAMERA_FPS                   = 10.0F;
    static constexpr float TARGET_MIN_AREA_PERCENT      = 0.0F;
    static constexpr float TARGET_MAX_AREA_PERCENT      = 100.0F;
    static constexpr float TARGET_SIZE                  = 20.0F;
    static constexpr float TARGET_REFLECTOR_HEIGHT      = 84.0F;
    static constexpr float TARGET_RANGE_MIN             = 132.0F;
    static constexpr float TARGET_RANGE_MAX             = 192.0F;
    static constexpr float GROUND_DISTANCE_TOLERANCE    = 6.0F;
    static constexpr float CALIBRATED_CAMERA_ANGLE      = 21.5778173F;
    static constexpr float RADIANS_TO_DEGREES           = M_PI / 180.0F;
    static constexpr float DECIMAL_TO_PERCENT           = 100.0F;
};




////////////////////////////////////////////////////////////////
// @method RobotCamera::SetCamera
///
/// This method updates which camera is active.
///
////////////////////////////////////////////////////////////////
inline void RobotCamera::SetCamera(Camera camera)
{
    // If the camera is USB, make sure it's present
    if ((camera == USB) && !m_bUsbCameraPresent)
    {
        return;
    }
    else if ((camera == BACK_USB) && !m_bBackUsbCameraPresent)
    {
        return;
    }
    else
    {
    }

    m_Camera = camera;
}
