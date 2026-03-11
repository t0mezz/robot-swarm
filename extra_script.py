import os
Import("env")

robot_id = os.environ.get("ROBOT_ID")
if robot_id is not None:
    env.Append(CPPDEFINES=[("ROBOT_ID", int(robot_id))])
