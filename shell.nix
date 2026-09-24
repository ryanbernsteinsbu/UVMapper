#!/usr/bin/env nix-shell
{ pkgs ? import <nixpkgs> { } }:
(
  let base = pkgs.appimageTools.defaultFhsEnvArgs; in
  pkgs.buildFHSEnv (base // {
    name = "FHS";
    targetPkgs = pkgs: (with pkgs; [
      gcc glibc zlib
      glfw3 mesa libGL libx11 libxrandr libxi eigen glm
    ]);

    runScript = "zsh";
    extraOutputsToInstall = [ "dev" ];
  })
).env
