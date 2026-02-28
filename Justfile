set windows-shell := ["powershell.exe", "-NoLogo", "-Command"]
set ignore-comments

ConstantsFile := "src/main/java/frc/robot/Constants.java"

[windows]
simulate:
    (Get-Content -Path {{ConstantsFile}}) -replace "simMode = Mode.REPLAY;", "simMode = Mode.SIM;" | Set-Content -Path {{ConstantsFile}}
    .\misc\scripts\openSimPrograms.ps1
    .\gradlew hotswapSimulateJava  "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2025\jdk\"

[linux]
simulate:
    sed -i 's/simMode = Mode.REPLAY;/simMode = Mode.SIM;/g' {{ConstantsFile}}
    ./gradlew hotswapSimulateJava

[windows]
replay:
    (Get-Content -Path {{ConstantsFile}}) -replace "simMode = Mode.SIM;", "simMode = Mode.REPLAY;" | Set-Content -Path {{ConstantsFile}}
    .\misc\scripts\openSimPrograms.ps1
    .\gradlew simulateJava  "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2025\jdk\"

[linux]
replay:
    sed -i 's/simMode = Mode.SIM;/simMode = Mode.REPLAY;/g' {{ConstantsFile}}
    ./gradlew simulateJava

template-subsystem:
    python ./misc/scripts/template-subsystem.py

tune-turret:
    python ./misc/turretTuning/analyze_data.py

setup-hotswap:
    python ./misc/scripts/setup-hotswap.py