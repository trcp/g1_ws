local task = {
  task = {
    task_name = "restaurant",
    display_name = "Restaurant",
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
    manipulation = {
      display_name = "Manipulation",
      description = "Launch Moveit!",
      commands = {
        default = {
          template = "cd /home/unitree/g1_ws && /usr/bin/docker compose run --name moveit --rm erasers_g1 bash -ic \"ros2 launch g1_moveit moveit_control_bringup.launch.py\"",
          kill = "docker stop moveit",
          variables = {},
        },
      },
    },
    navigation = {
      display_name = "Navigation",
      description = "all navigation launch",
      commands = {
        default = {
          template = "cd /home/unitree/g1_ws && /usr/bin/docker compose run --name nav --rm erasers_g1 bash -ic \"ros2 launch machida_navigation machida_navigation.launch.py launch_map_server:=false launch_localization:=false use_head_camera:=false goal_snap_to_free:=true path_check_horizon:=0.3\"",
          kill = "docker stop nav",
          variables = {},
        },
      },
    },
    cartographer = {
      display_name = "Cartographer",
      description = "all localization launch",
      commands = {
        default = {
          template = "cd /home/unitree/g1_ws && /usr/bin/docker compose run --name cartographer --rm erasers_g1 bash -ic \"ros2 launch g1_cartographer 2d_cartographer.launch.py\"",
          kill = "docker stop cartographer",
          variables = {},
        },
      },
    },
    restaurant = {
      display_name = "Restaurant",
      description = "Bringup the Restaurant Task",
      commands = {
        default = {
          template = "export DISPLAY=:1 && cd /home/roboworks/g1_ws && /usr/bin/docker compose run --name restaurant --rm katana bash -ic \"ros2 run robot_tasks restaurant_task\"",
          kill = "docker stop restaurant",
          variables = {},
        },
      },
    },
    recongnition = {
      display_name = "Recongnition",
      description = "Bringup the Recongnition",
      commands = {
        default = {
          template = "export DISPLAY=:1 && cd /home/roboworks/g1_ws && /usr/bin/docker compose run --name recongnition --rm katana bash -ic \"ros2 launch robot_tasks restaurant_depends.launch.py\"",
          kill = "docker stop recongnition",
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
          { program = "restaurant" },
          { program = "recongnition" },
        },
      },
      {
        direction = "vertical",
        panes = {
           { program = "navigation" },
           { program = "manipulation" },
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
