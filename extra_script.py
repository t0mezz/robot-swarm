import os
Import("env")

# Allow overriding ROBOT_ID at flash time:
#   ROBOT_ID=3 pio run -e receiver -t upload
# Defaults to 0.
robot_id = int(os.environ.get("ROBOT_ID", 0))

# Write robot_id_cfg.h next to hardware.h so the relative #include resolves.
# SCons tracks .h dependencies, so changing ROBOT_ID triggers recompilation.
project_dir = env.subst("$PROJECT_DIR")
header_path = os.path.join(project_dir, "lib", "SwarmProtocol", "robot_id_cfg.h")
with open(header_path, "w") as f:
    f.write(f"#pragma once\n#define ROBOT_ID {robot_id}\n")
