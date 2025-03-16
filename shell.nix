{ pkgs ? import <nixpkgs> {} }:
  pkgs.mkShell {
    nativeBuildInputs = with pkgs.buildPackages; [ 
    android-studio
    android-tools
    jetbrains-toolbox
    ];
}
