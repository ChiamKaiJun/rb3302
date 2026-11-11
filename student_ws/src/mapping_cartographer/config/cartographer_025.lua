-- Optional 2.5 cm scan-matching grid; retain all other baseline settings.
include "cartographer.lua"

TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.025

-- Also pass resolution:=0.025 to mapping.launch.py for the published /map.
return options
