{
  description = "ROS 2 Jazzy robotic arm + MoveIt development environment";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";

    # Important: use the nixpkgs revision expected by nix-ros-overlay.
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
  };

  outputs = { self, nixpkgs, nix-ros-overlay }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem
      (system:
        let
          pkgs = import nixpkgs {
            inherit system;

            overlays = [
              nix-ros-overlay.overlays.default
            ];
          };

          ros = pkgs.rosPackages.jazzy;
        in
        {
          # Exposed so .moveit-shell.nix can layer machine-local runtime
          # additions on top of this shell without re-pinning the overlay.
          packages.pkgs = pkgs // { inherit ros; };

          devShells.default = pkgs.mkShell {
            name = "arm-control-ros2";

            packages = [
              # General build/dev tools
              pkgs.git
              pkgs.cmake
              pkgs.ninja
              pkgs.pkg-config
              pkgs.colcon

              # ROS environment -- one buildEnv, shared list. See ros-packages.nix.
              (ros.buildEnv {
                paths = import ./ros-packages.nix ros;
              })
            ];

            shellHook = ''
              echo "ROS 2 Jazzy robotic-arm environment"
              echo "ROS_DISTRO=$ROS_DISTRO"

              # Automatically overlay our locally-built workspace.
              if [ -f "$PWD/install/local_setup.bash" ]; then
                source "$PWD/install/local_setup.bash"
                echo "Loaded workspace overlay: $PWD/install"
              fi
            '';
          };
        });
}