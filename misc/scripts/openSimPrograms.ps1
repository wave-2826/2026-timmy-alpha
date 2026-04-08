# Exit early if OPEN_SIM_PROGRAMS is set to "false"
if($env:OPEN_SIM_PROGRAMS -eq "false") {
    return
}

function OpenIfNotRunning {
    param (
        [string]$ProgramPath
    )

    $Name = [System.IO.Path]::GetFileNameWithoutExtension($ProgramPath)

    $Running = Get-Process $Name -ErrorAction SilentlyContinue
    if($Running -eq $null) {
        Start-Process $ProgramPath -WindowStyle Normal | Out-Null
    } else {
        # Focus the program
        $procId = $Running.Where({ $_.MainWindowTitle }, 'First').Id
        if(-not $procId) { return }

        if($hwnd -ne 0) {
            $null = (New-Object -ComObject WScript.Shell).AppActivate($procId)
        }
    }
}

# OpenIfNotRunning -ProgramPath "C:\Users\Public\wpilib\2026\advantagescope\AdvantageScope (WPILib).exe"
# OpenIfNotRunning -ProgramPath "C:\Users\Public\wpilib\2026\elastic\elastic_dashboard.exe"
