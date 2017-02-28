# Godel Noether

Implements Godel's meshing and tool path planning plugins using the Path-Planning IR&D work.

## Description

Exports a meshing plugin by the name: `godel_noether::NoetherMesher`.

Exports a tool path planning plugin by the name: `godel_noether::NoetherPathPlanner`. 

## Dependencies

[Godel](https://github.com/ros-industrial-consortium/godel/)
[Noether](https://raesgit.datasys.swri.edu/raes-client/10.R8706) 

## To Use
Under `godel_robots` find your robot support package and then look in the `config/` folder for a `plugins.yaml` file.

Change it to read:

```
meshing_plugin_name: "godel_noether::NoetherMesher"
blend_tool_planning_plugin_name: "godel_noether::NoetherPathPlanner"
...
```
