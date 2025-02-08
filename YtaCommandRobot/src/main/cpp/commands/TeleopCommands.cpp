// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>      // for interacting with the smart dashboard

#include "commands/TeleopCommands.hpp"
#include "subsystems/TeleopSubsystem.hpp"

unsigned x = 0U;
void GlobalRobotHook()
{
    //std::printf("Loop iteration (x): %d\n", x);
    frc::SmartDashboard::PutNumber("Heartbeat", x++);
}

frc2::CommandPtr Yta::Teleop::StaticFactoryCommand(TeleopSubsystem* pTeleopSubsystem)
{
    std::function<void()> funcPtr = GlobalRobotHook;
    return frc2::cmd::Run(funcPtr);
}



TeleopHelperCommand::TeleopHelperCommand(TeleopSubsystem* pTeleopSubsystem) :
    m_pTeleopSubsystem{pTeleopSubsystem}
{
    // Register that this command requires the subsystem.
    AddRequirements(m_pTeleopSubsystem);
}

void TeleopHelperCommand::Execute()
{
    static unsigned e = 0U;
    //std::printf("Loop iteration (e): %d\n", e);
    frc::SmartDashboard::PutNumber("Heartbeat", e++);
}
