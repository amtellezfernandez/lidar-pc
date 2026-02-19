$ErrorActionPreference = "Stop"

if (-not (Test-Path "pyproject.toml")) {
    throw "Run this script from the lidar-pc repository root (where pyproject.toml exists)."
}

python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"

Write-Host "Windows setup complete. Activate with: .\.venv\Scripts\Activate.ps1"
