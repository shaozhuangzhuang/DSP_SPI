# ============================================================================
# F28377D Dual-Core DSP Build Script
# ============================================================================
# Function: Auto build Prj_Ori05 dual-core projects (CPU1 and CPU2)
# Author: Auto-generated
# Date: 2025-10-29
# ============================================================================

param(
    [string]$Action = "build",  # build, clean, rebuild
    [switch]$Parallel = $false, # enable parallel build
    [int]$Jobs = 4              # number of parallel tasks
)

# ============================================================================
# Configuration
# ============================================================================

# CCS installation path (modify according to your setup)
$CCS_ROOT = "C:\ti\ccs1210\ccs"
$GMAKE_PATH = "$CCS_ROOT\utils\bin\gmake.exe"

# Project root directory
$PROJECT_ROOT = $PSScriptRoot

# Project configuration - Prj_Ori05
$CPU1_PROJECT = "$PROJECT_ROOT\Prj_Ori05_cpu1"
$CPU1_BUILD_DIR = "$CPU1_PROJECT\CPU1_FLASH_STANDALONE"
$CPU1_OUTPUT = "$CPU1_BUILD_DIR\Prj_Ori05_cpu1.out"

$CPU2_PROJECT = "$PROJECT_ROOT\Prj_Ori05_cpu2"
$CPU2_BUILD_DIR = "$CPU2_PROJECT\CPU2_FLASH"
$CPU2_OUTPUT = "$CPU2_BUILD_DIR\Prj_Ori05_cpu2.out"

# ============================================================================
# Helper Functions
# ============================================================================

function Print-Header {
    param([string]$Message)
    Write-Host ""
    Write-Host "============================================================================" -ForegroundColor Cyan
    Write-Host " $Message" -ForegroundColor Cyan
    Write-Host "============================================================================" -ForegroundColor Cyan
}

function Print-Success {
    param([string]$Message)
    Write-Host "[SUCCESS] $Message" -ForegroundColor Green
}

function Print-Error {
    param([string]$Message)
    Write-Host "[ERROR] $Message" -ForegroundColor Red
}

function Print-Info {
    param([string]$Message)
    Write-Host "[INFO] $Message" -ForegroundColor Yellow
}

function Check-Prerequisites {
    Print-Header "Check Build Environment"
    
    # Check if gmake.exe exists
    if (-Not (Test-Path $GMAKE_PATH)) {
        Print-Error "Cannot find gmake.exe: $GMAKE_PATH"
        Print-Info "Please modify CCS_ROOT variable in the script"
        exit 1
    }
    Print-Success "Found gmake: $GMAKE_PATH"
    
    # Check if project directories exist
    if (-Not (Test-Path $CPU1_BUILD_DIR)) {
        Print-Error "Cannot find Prj_Ori05_cpu1 build directory: $CPU1_BUILD_DIR"
        exit 1
    }
    Print-Success "Found Prj_Ori05_cpu1 build directory"
    
    if (-Not (Test-Path $CPU2_BUILD_DIR)) {
        Print-Error "Cannot find Prj_Ori05_cpu2 build directory: $CPU2_BUILD_DIR"
        exit 1
    }
    Print-Success "Found Prj_Ori05_cpu2 build directory"
    
    Write-Host ""
}

function Build-Project {
    param(
        [string]$ProjectName,
        [string]$BuildDir,
        [string]$OutputFile
    )
    
    Print-Header "Building $ProjectName"
    
    # Enter build directory
    Push-Location $BuildDir
    
    try {
        # Build gmake command
        $gmake_args = @()
        
        if ($Parallel) {
            $gmake_args += "-j$Jobs"
            Print-Info "Using $Jobs parallel tasks"
        }
        
        $gmake_args += "all"
        
        # Execute build
        Print-Info "Command: $GMAKE_PATH $($gmake_args -join ' ')"
        $start_time = Get-Date
        
        & $GMAKE_PATH $gmake_args
        
        $exit_code = $LASTEXITCODE
        $elapsed = (Get-Date) - $start_time
        
        if ($exit_code -eq 0) {
            if (Test-Path $OutputFile) {
                $file_size = (Get-Item $OutputFile).Length / 1KB
                Print-Success "$ProjectName build succeeded!"
                Print-Info "Output file: $OutputFile"
                Print-Info "File size: $([Math]::Round($file_size, 2)) KB"
                Print-Info "Time elapsed: $([Math]::Round($elapsed.TotalSeconds, 2)) seconds"
                return $true
            } else {
                Print-Error "$ProjectName build failed: output file not generated"
                return $false
            }
        } else {
            Print-Error "$ProjectName build failed! Exit code: $exit_code"
            return $false
        }
    }
    finally {
        Pop-Location
    }
}

function Clean-Project {
    param(
        [string]$ProjectName,
        [string]$BuildDir
    )
    
    Print-Header "Cleaning $ProjectName"
    
    Push-Location $BuildDir
    
    try {
        Print-Info "Command: $GMAKE_PATH clean"
        & $GMAKE_PATH clean
        
        if ($LASTEXITCODE -eq 0) {
            Print-Success "$ProjectName cleaned successfully!"
            return $true
        } else {
            Print-Error "$ProjectName clean failed!"
            return $false
        }
    }
    finally {
        Pop-Location
    }
}

# ============================================================================
# Main Program
# ============================================================================

Print-Header "F28377D Dual-Core DSP Build Tool"
Write-Host "Project Root: $PROJECT_ROOT"
Write-Host "Action: $Action"
Write-Host ""

# Check environment
Check-Prerequisites

# Record start time
$total_start = Get-Date
$success_count = 0
$total_count = 2

# Execute based on action
switch ($Action.ToLower()) {
    "clean" {
        # Clean only
        if (Clean-Project "Prj_Ori05_cpu1" $CPU1_BUILD_DIR) { $success_count++ }
        if (Clean-Project "Prj_Ori05_cpu2" $CPU2_BUILD_DIR) { $success_count++ }
    }
    
    "rebuild" {
        # Clean + Build
        Print-Info "Full rebuild (clean + build)"
        Write-Host ""
        
        if (Clean-Project "Prj_Ori05_cpu1" $CPU1_BUILD_DIR) {
            if (Build-Project "Prj_Ori05_cpu1" $CPU1_BUILD_DIR $CPU1_OUTPUT) {
                $success_count++
            }
        }
        
        if (Clean-Project "Prj_Ori05_cpu2" $CPU2_BUILD_DIR) {
            if (Build-Project "Prj_Ori05_cpu2" $CPU2_BUILD_DIR $CPU2_OUTPUT) {
                $success_count++
            }
        }
    }
    
    "build" {
        # Incremental build
        if (Build-Project "Prj_Ori05_cpu1" $CPU1_BUILD_DIR $CPU1_OUTPUT) {
            $success_count++
        }
        
        if (Build-Project "Prj_Ori05_cpu2" $CPU2_BUILD_DIR $CPU2_OUTPUT) {
            $success_count++
        }
    }
    
    default {
        Print-Error "Unknown action: $Action"
        Print-Info "Supported actions: build, clean, rebuild"
        exit 1
    }
}

# ============================================================================
# Summary
# ============================================================================

$total_elapsed = (Get-Date) - $total_start

Print-Header "Build Summary"
Write-Host "Success: $success_count / $total_count projects" -ForegroundColor $(if ($success_count -eq $total_count) { "Green" } else { "Red" })
Write-Host "Total time: $([Math]::Round($total_elapsed.TotalSeconds, 2)) seconds"

if ($success_count -eq $total_count) {
    Write-Host ""
    Print-Success "All projects built successfully!"
    
    # Show output files
    Write-Host ""
    Print-Info "Output files:"
    if (Test-Path $CPU1_OUTPUT) {
        Write-Host "  - Prj_Ori05_cpu1: $CPU1_OUTPUT"
    }
    if (Test-Path $CPU2_OUTPUT) {
        Write-Host "  - Prj_Ori05_cpu2: $CPU2_OUTPUT"
    }
    
    exit 0
} else {
    Write-Host ""
    Print-Error "Some projects failed to build!"
    exit 1
}

# ============================================================================
# Usage Instructions
# ============================================================================
<#
.SYNOPSIS
    F28377D Dual-Core DSP Build Script

.DESCRIPTION
    Auto build Prj_Ori05 dual-core projects (CPU1 and CPU2)
    Projects: Prj_Ori05_cpu1, Prj_Ori05_cpu2
    Support incremental build, clean, rebuild, parallel build

.PARAMETER Action
    Action mode:
    - build   : Incremental build (default)
    - clean   : Clean all generated files
    - rebuild : Clean and full rebuild

.PARAMETER Parallel
    Enable parallel build (faster)

.PARAMETER Jobs
    Number of parallel tasks (default 4)

.EXAMPLE
    .\build_all.ps1

.EXAMPLE
    .\build_all.ps1 -Parallel

.EXAMPLE
    .\build_all.ps1 -Parallel -Jobs 8

.EXAMPLE
    .\build_all.ps1 -Action clean

.EXAMPLE
    .\build_all.ps1 -Action rebuild

.EXAMPLE
    .\build_all.ps1 -Action rebuild -Parallel -Jobs 8

.NOTES
    File: build_all.ps1
    Author: Auto-generated
    Date: 2025-10-29
    
    Notes:
    1. Modify $CCS_ROOT to your CCS installation path before first run
    2. If script execution error, run:
       Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned
    3. Recommend using parallel build for faster compilation
#>
