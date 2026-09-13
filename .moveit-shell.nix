# Machine-local entry point for the arm_control workspace (used by .envrc).
#
# Builds ONE ros.buildEnv from ./ros-packages.nix. Add machine-local extras to
# `extraRosPackages` below rather than to ros-packages.nix if you don't want
# them in the shared flake.
let
  flake = builtins.getFlake (toString ./.);
  pkgs = flake.packages.${builtins.currentSystem}.pkgs;
  ros = pkgs.ros;
  nixGL = flake.inputs.nixGL;

  extraRosPackages = with ros; [ ];
in
pkgs.mkShell {
  name = "arm-control-moveit-runtime";

  packages = [
    nixGL.packages.${builtins.currentSystem}.nixGLIntel
    pkgs.git
    pkgs.cmake
    pkgs.ninja
    pkgs.pkg-config
    pkgs.colcon
    pkgs.lapack
    pkgs.blas

    (ros.buildEnv {
      paths = (import ./ros-packages.nix ros) ++ extraRosPackages;
    })
  ];

  shellHook = ''
    echo "ROS 2 Jazzy arm_control environment (ROS_DISTRO=$ROS_DISTRO)"

    # Fix GLX rendering for RViz2 on non-NixOS (Intel GPU).
    if command -v nixGLIntel &>/dev/null; then
      export LIBGL_DRIVERS_PATH="$(nixGLIntel printenv LIBGL_DRIVERS_PATH 2>/dev/null)"
      export __EGL_VENDOR_LIBRARY_FILENAMES="$(nixGLIntel printenv __EGL_VENDOR_LIBRARY_FILENAMES 2>/dev/null)"
      export LD_LIBRARY_PATH="$(nixGLIntel printenv LD_LIBRARY_PATH 2>/dev/null)"
    fi

    if [ -f "$PWD/install/setup.bash" ]; then
      source "$PWD/install/setup.bash"
    fi
  '';
}
