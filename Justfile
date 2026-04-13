set windows-shell := ["powershell.exe", "-NoLogo", "-Command"]
set ignore-comments
set dotenv-load := true

ConstantsFile := "src/main/java/frc/robot/Constants.java"

[windows]
simulate:
    (Get-Content -Path {{ConstantsFile}}) -replace "simMode = Mode.REPLAY;", "simMode = Mode.SIM;" | Set-Content -Path {{ConstantsFile}}
    .\misc\scripts\openSimPrograms.ps1
    .\gradlew simulateJava "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2026\jdk\"

[linux]
simulate:
    # Delete all but the newest hs_*.log file; java crashes on linux every time we stop it
    ls -t hs_*.log 2>/dev/null | tail -n +2 | xargs -r rm --
    sed -i 's/simMode = Mode.REPLAY;/simMode = Mode.SIM;/g' {{ConstantsFile}}
    ./gradlew simulateJava  \
        -Porg.gradle.java.installations.paths={{env('JAVA_PATH', home_dir() / '.jbr-jcef-17')}} \
        -Porg.gradle.java.installations.auto-detect={{env('JAVA_AUTODETECT', 'false')}} \
        -Porg.gradle.java.installations.auto-download=false

build-watch:
    ./gradlew classes -t

[windows]
replay:
    (Get-Content -Path {{ConstantsFile}}) -replace "simMode = Mode.SIM;", "simMode = Mode.REPLAY;" | Set-Content -Path {{ConstantsFile}}
    .\misc\scripts\openSimPrograms.ps1
    .\gradlew simReplay  "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2026\jdk\"

build:
    ./gradlew build

[linux]
replay:
    sed -i 's/simMode = Mode.SIM;/simMode = Mode.REPLAY;/g' {{ConstantsFile}}
    ./gradlew simReplay

template-subsystem:
    python ./misc/scripts/template-subsystem.py

tune-turret:
    cd ./misc/turretTuning/; python ./analyze_data.py

setup-hotswap:
    python ./misc/scripts/setup-hotswap.py

flip-autos:
    python ./misc/scripts/flipAutos.py

upload-tags TAGS:
    python ./misc/scripts/uploadTagMap.py {{TAGS}}