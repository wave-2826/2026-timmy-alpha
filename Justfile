set windows-shell := ["powershell.exe", "-NoLogo", "-Command"]
set ignore-comments

ConstantsFile := "src/main/java/frc/robot/Constants.java"

simulate:
    (Get-Content -Path {{ConstantsFile}}) -replace "simMode = Mode.REPLAY;", "simMode = Mode.SIM;" | Set-Content -Path {{ConstantsFile}}
    .\misc\scripts\openSimPrograms.ps1
    .\gradlew simulateJava  "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2025\jdk\"

replay:
    (Get-Content -Path {{ConstantsFile}}) -replace "simMode = Mode.SIM;", "simMode = Mode.REPLAY;" | Set-Content -Path {{ConstantsFile}}
    .\misc\scripts\openSimPrograms.ps1
    .\gradlew simulateJava  "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2025\jdk\"

template-subsystem:
    python ./misc/scripts/template-subsystem.py

tune-turret:
    python ./misc/turretTuning/analyze_data.py
