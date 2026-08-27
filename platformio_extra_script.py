import os
import subprocess
Import("env")

# Fix littlefs tool path
#print(f"env: {env.Dump()}")
tool_path = env.subst('$PROJECT_PACKAGES_DIR')
if os.name == 'nt':
    tool_dir = ""
else:
    tool_dir = "/"

for ps in tool_path.split(os.path.sep)[:-1]:
    if tool_dir == "":
        # Windows
        tool_dir = os.path.join(tool_dir, ps)
        tool_dir += "\\"
    else:
        tool_dir = os.path.join(tool_dir, ps)

tool_dir = os.path.join(tool_dir, "tools")
tool_dir = os.path.join(tool_dir, "tool-mklittlefs")

if os.name == 'nt':
    mklittlefs_bin = "mklittlefs.exe"
else:
    mklittlefs_bin = "mklittlefs"

env.Replace(MKFSTOOL=os.path.join(tool_dir, mklittlefs_bin))

print(f"Fixing littlefs MKFSTOOL: {env.subst('$MKFSTOOL')}")

# Get the git commit hash
try:
    commit = subprocess.check_output(
        ["git", "rev-parse", "--short", "HEAD"],
        text=True,
    ).strip()
except Exception:
    commit = "unknown"
env.Append(
    BUILD_FLAGS=[
        f'-DGIT_COMMIT_HASH=\\"{commit}\\"',
    ]
)

print(f"Git commit hash: {commit}")