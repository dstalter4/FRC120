// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandHelper.h>

class TeleopSubsystem;


namespace Yta::Teleop
{
    // Example static factory for a teleop command
    frc2::CommandPtr StaticFactoryCommand(TeleopSubsystem* pTeleopSubsystem);
}

class TeleopHelperCommand
    : public frc2::CommandHelper<frc2::Command, TeleopHelperCommand>
{
public:
    // Creates a new TeleopHelperCommand.
    //
    // @param subsystem The subsystem used by this command.
    explicit TeleopHelperCommand(TeleopSubsystem* pTeleopSubsystem);
    virtual void Execute();

private:
    TeleopSubsystem* m_pTeleopSubsystem;
};
