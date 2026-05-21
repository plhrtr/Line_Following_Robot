{
  description = "Flake for STM32 C development";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
  };

  outputs =
    { self, nixpkgs }@inputs:

    let
      supportedSystems = [
        "x86_64-linux"
        "aarch64-linux"
        "aarch64-darwin"
      ];
      forEachSupportedSystem =
        f:
        inputs.nixpkgs.lib.genAttrs supportedSystems (
          system:
          f {
            inherit system;
            pkgs = import inputs.nixpkgs {
              inherit system;
              config = {
                allowUnfree = true;
              };
            };
          }
        );
    in
    {
      devShells = forEachSupportedSystem (
        { pkgs, system }:
        {
          default =
            pkgs.mkShell.override
              {
              }
              {
                packages = with pkgs; [
                  glibc_multi.dev
                  cmake
                  stm32cubemx
                  gcc
                  clang-tools
                  gcc-arm-embedded
                  ninja
                  openocd
                  stlink
                  stlink-gui
                  serial-studio
                ];

                shellHook = "";
              };
        }
      );

      formatter = forEachSupportedSystem ({ pkgs, ... }: pkgs.nixfmt);
    };
}
