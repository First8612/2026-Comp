# for each .path file in ./src/main/deploy/pathplanner/paths 
# with the word "Right" in it,
# take all the Y coordinates, and change to:
#   {new y} = $FieldWidthMeters - {orig y}
# and write the file to the "Left" version of the same filename

# for each .auto file in ./src/main/deploy/pathplanner/autos 
# with the word "Right"
# replace right with left
# and write the file to the "Left" version of the same filename

$FieldWidthMeters = 8;

function Convert-RightToLeft([string]$text) {
    $text = $text -creplace "RIGHT", "LEFT"
    $text = $text -creplace "Right", "Left"
    $text = $text -creplace "right", "left"
    return $text
}

$pathFiles = Get-ChildItem -Path "./src/main/deploy/pathplanner/paths" -Filter "*.path" | Where-Object { $_.Name -like "*Right*" }

foreach ($file in $pathFiles) {
    $content = Get-Content $file.FullName -Raw | ConvertFrom-Json

    foreach ($waypoint in $content.waypoints) {
        $waypoint.anchor.y = $FieldWidthMeters - $waypoint.anchor.y
        if ($null -ne $waypoint.prevControl) {
            $waypoint.prevControl.y = $FieldWidthMeters - $waypoint.prevControl.y
        }
        if ($null -ne $waypoint.nextControl) {
            $waypoint.nextControl.y = $FieldWidthMeters - $waypoint.nextControl.y
        }
    }

    $newName = Convert-RightToLeft $file.Name
    $newPath = Join-Path $file.DirectoryName $newName

    $content | ConvertTo-Json -Depth 10 | Set-Content $newPath
    Write-Host "Created: $newName"
}

$autoFiles = Get-ChildItem -Path "./src/main/deploy/pathplanner/autos" -Filter "*.auto" | Where-Object { $_.Name -like "*Right*" }

foreach ($file in $autoFiles) {
    $content = Get-Content $file.FullName -Raw
    $newContent = Convert-RightToLeft $content
    $newName = Convert-RightToLeft $file.Name
    $newPath = Join-Path $file.DirectoryName $newName
    Set-Content -Path $newPath -Value $newContent
    Write-Host "Created: $newName"
}
