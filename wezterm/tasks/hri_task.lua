local task = {
  task = {
    task_name = "hri_task",
    display_name = "HRI Task",
    description = "HRI task with YOLO tracking",
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
          template = "cd /home/unitree/g1_ws && /usr/bin/docker compose run --name nav --rm erasers_g1 bash -ic \"ros2 launch machida_navigation machida_navigation.launch.py map_dir:=/home/unitree/colcon_ws/map\"",
          kill = "/usr/bin/docker stop nav",
          variables = {},
        },
      },
    },
    hri_main = {
      display_name = "HriMain",
      description = "Run HRI task main script",
      commands = {
        default = {
          template = "cd /home/roboworks/g1_ws && /usr/bin/docker compose run --name rc --rm katana bash -ic \"cd src/robot_tasks/hri_task/hri_task/ && python3 main.py\"",
          kill = "/usr/bin/docker stop rc",
          variables = {},
        },
      },
    },
    ollama = {
      display_name = "Ollama",
      description = "Run local Ollama server for offline LLM extraction",
      commands = {
        default = {
          template = "/home/roboworks/.local/bin/ollama serve",
          kill = "pkill -f '/home/roboworks/.local/bin/ollama serve'",
          variables = {},
        },
      },
    },
    yolo_human = {
      display_name = "YoloHuman",
      description = "Run YOLO human detection",
      commands = {
        default = {
          template = "cd /home/roboworks/g1_ws/src/robot_tasks/hri_task/hri_task/yolo_human && /usr/bin/docker compose up",
          kill = "cd /home/roboworks/g1_ws/src/robot_tasks/hri_task/hri_task/yolo_human && /usr/bin/docker compose down",
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
          { program = "hri_main" },
        },
      },
      {
        direction = "vertical",
        panes = {
           { program = "navigation" },
           { program = "yolo_human" },
           { program = "ollama" },
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
