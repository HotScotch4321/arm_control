# Machine-local entry point for the arm_control workspace (used by .envrc).
#
# Builds ONE ros.buildEnv from ./ros-packages.nix. Add machine-local extras to
# `extraRosPackages` below rather than to ros-packages.nix if you don't want
# them in the shared flake.
let
  flake = builtins.getFlake (toString ./.);
  pkgs = flake.packages.${builtins.currentSystem}.pkgs;
  ros = pkgs.ros;

  extraRosPackages = with ros; [ ];
in
pkgs.mkShell {
  name = "arm-control-moveit-runtime";

  packages = [
    pkgs.git
    pkgs.cmake
    pkgs.ninja
    pkgs.pkg-config
    pkgs.colcon

    (ros.buildEnv {
      paths = (import ./ros-packages.nix ros) ++ extraRosPackages;
    })
  ];

  shellHook = ''
    echo "ROS 2 Jazzy arm_control environment (ROS_DISTRO=$ROS_DISTRO)"
  '';
}
