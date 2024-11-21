{ pkgs, lib, config, inputs, ... }:
let
  pros-cli = pkgs.python3Packages.buildPythonPackage rec {
    pname = "pros-cli";
    version = "3.5.4";
    doCheck = false;
    propagatedBuildInputs = with pkgs.python3Packages; [
      setuptools
      wheel
      jsonpickle
      pyserial
      tabulate
      cobs
      click
      rich-click
      cachetools
      requests-futures
      semantic-version
      colorama
      pyzmq
      sentry-sdk
      pypng
      # libclang.python
    ];
    src = (
      pkgs.fetchFromGitHub {
        owner = "purduesigbots";
        repo = pname;
        rev = version;
        sha256 = "sha256-za7XBPn8inWyGinTUW1Kqs3711jgpGzmGj4ierynPkA=";
      }
    );

    # My attempts at trying to get the pros/autocomplete library included in dist
    # All failed, so we do the hacky solution of copying it ourselves.
    # If the python version changes, the postInstall must be changed too.

    # postPatch = ''
    #   # Ensure autocomplete scripts are included.
    #   echo "include pros/autocomplete" >> MANIFEST.in
    #   echo "recursive-include pros/autocomplete/ *" >> MANIFEST.in
    #
    #   sed -i "/name='pros-cli',/a \tinclude_package_data=True," setup.py
    #   cat setup.py
    # '';
    #
    # postBuild = ''
    #   # cp -r $src/pros/autocomplete build/lib/pros/autocomplete
    #   ls -lah build/lib/pros
    # '';

    # NOTE: Change `python3.11` to match the current python version.
    # See note above postPatch for details.
    postInstall = ''
      # Verify that pros/autocomplete files are in the output directory
      if ! [ -d "$out/lib/python3.11/site-packages/pros/autocomplete" ]; then
        mkdir -p $out/lib/python3.11/site-packages/pros/autocomplete
        cp -r pros/autocomplete/* $out/lib/python3.11/site-packages/pros/autocomplete/
      fi
    '';
  };

in {
  # https://devenv.sh/basics/
  # env.GREET = "devenv";

  # https://devenv.sh/packages/
  packages = with pkgs; [
    gcc-arm-embedded
    pros-cli
    bear # build compile commands
  ];

  # https://devenv.sh/languages/
  languages.cplusplus.enable = true;
  languages.c.enable = true;

  # quick aliases
  scripts.mut.exec = "bear -- pros --no-sentry --no-analytics mut --after run";
  scripts.mu.exec = "bear -- pros --no-sentry --no-analytics mu";
  scripts.t.exec = "pros --no-sentry --no-analytics t";
  scripts.m.exec = "bear -- pros --no-sentry --no-analytics make";
}
