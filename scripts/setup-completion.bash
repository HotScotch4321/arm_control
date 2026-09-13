# Source this from an interactive shell (not .envrc) after direnv activates:
#   source scripts/ros2-completion.bash
#
# direnv can transport environment variables but not shell builtins like
# `complete`, so the argcomplete registration must run in the interactive
# shell itself.

eval "$(register-python-argcomplete ros2)"
eval "$(register-python-argcomplete colcon)"
