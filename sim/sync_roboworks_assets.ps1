Param(
  [string]$SourceRoot = "C:\Users\admin\ros2_robotverseny_gazebo24\robotverseny_description\models\roboworks",
  [string]$DestRoot = "C:\Users\admin\ros2_hybrid_astar_nav2\sim\roboworks_model\roboworks"
)

$ErrorActionPreference = "Stop"

if (!(Test-Path $SourceRoot)) {
  throw "SourceRoot not found: $SourceRoot"
}

New-Item -ItemType Directory -Force -Path $DestRoot | Out-Null

Copy-Item -Force "$SourceRoot\model.sdf" "$DestRoot\model.sdf"
Copy-Item -Force "$SourceRoot\model.config" "$DestRoot\model.config"

if (Test-Path "$SourceRoot\meshes") {
  New-Item -ItemType Directory -Force -Path "$DestRoot\meshes" | Out-Null
  Copy-Item -Recurse -Force "$SourceRoot\meshes\*" "$DestRoot\meshes\"
}

Write-Host "Synced roboworks model assets to: $DestRoot"

