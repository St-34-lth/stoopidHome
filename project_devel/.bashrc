
# fnm
FNM_PATH="/root/.local/share/fnm"
if [ -d "$FNM_PATH" ]; then
  export PATH="$FNM_PATH:$PATH"
  eval "`fnm env`"
fi

 export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:~/project_files/root/rosenv/src"