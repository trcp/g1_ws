local task = {
  task = {
    task_name = "inspection",
    display_name = "Robot Inspection",
    description = "",
    author = "erasers",
  },
  programs = {
    g1_bringup = {
      display_name = "G1 Bring up",
      description = "",
      commands = {
        default = {
          template = "cd /home/unitree/g1_ws && /usr/bin/docker compose run --name erasers_g1 --rm erasers_g1 bash -ic \"ros2 launch g1_bringup bringup.launch.py\"",
          kill = "/usr/bin/docker stop erasers_g1",
          variables = {},
        },
      },
    },
    navigation = {
      display_name = "Navigation",
      description = "all navigation and localization launch",
      commands = {
        default = {
          template = "cd /home/unitree/g1_ws && /usr/bin/docker compose run --name nav --rm erasers_g1 bash -ic \"ros2 launch machida_navigation machida_navigation.launch.py map_dir:=/home/unitree/colcon_ws/map map_name:=map2\"",
          kill = "/usr/bin/docker stop nav",
          variables = {},
        },
      },
    },
    robot_inspection = {
      display_name = "RobotInspection",
      description = "Bringup the Robot Inspection",
      commands = {
        default = {
          template = "cd /home/roboworks/g1_ws && /usr/bin/docker compose run --name ri --rm katana bash -ic \"ros2 run robot_tasks robot_inspection\"",
          kill = "/usr/bin/docker stop ri",
          variables = {},
        },
      },
    },
  },
  -- direction: "horizontal"=上下分割, "vertical"=左右分割
  layout = {
    direction = "horizontal",
    panes = {
      {
        direction = "vertical",
        panes = {
          { program = "g1_bringup" },
          { program = "robot_inspection" },
        },
      },
      {
        direction = "vertical",
        panes = {
           { program = "navigation" },
        },
      },
    },
  },
}

-- Standalone wezterm config support
-- When loaded directly via `wezterm --config-file <this file>`, auto-launches
-- this task. When dofile'd from wezterm.lua (_ERASERS_TASK_CONTEXT="library"),
-- returns the task data table as usual.
local ok, wezterm = pcall(require, "wezterm")
if ok and _G._ERASERS_TASK_CONTEXT ~= "library" then
  local lib = dofile(wezterm.home_dir .. "/g1_ws/wezterm/lib.lua")
  local config = wezterm.config_builder()
  config.automatically_reload_config = true
  wezterm.on('gui-startup', function()
    lib.launch_task(task)
  end)
  return config
end

return task
