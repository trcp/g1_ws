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
      command = {
        template = "cd /home/unitree/g1_ws && /usr/bin/docker compose up erasers_g1",
        kill = "cd /home/unitree/g1_ws && /usr/bin/docker compose down erasers_g1",
        variables = {},
      },
    },
    navigation = {
      display_name = "Navigation",
      description = "all navigation and localization launch",
      command = {
        template = "ros2 launch machida_navigation machida_navigation.launch.py",
        kill = "",
        variables = {},
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
