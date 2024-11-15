{
  description = "PROS for Vex robots";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs?ref=nixpkgs-unstable";
  };

  outputs = { nixpkgs, ... }:
    let
      system = "x86_64-linux";
      pkgs = nixpkgs.legacyPackages.${system};

      pros-cli = pkgs.python3Packages.buildPythonPackage rec {
        pname = "pros-cli";
        version = "3.5.1";
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
          #scan-build
          sentry-sdk
          #observable
          pypng
          #pyinstaller
        ];
        src = (
          pkgs.fetchFromGitHub {
            owner = "purduesigbots";
            repo = pname;
            rev = version;
            sha256 = "sha256-kHGiYDIfB87lUvA/gCjOg890+GeqiGr7wBYWlAdUISE=";
          }
        )

        ;
      };

    in
    {
      devShells.${system}.default = pkgs.mkShell {
        packages = with pkgs; [
          gcc-arm-embedded
          pros-cli
        ];
        shellHook = ''
          echo -n Ready to win Worlds, Champ? 
          export MAKEFLAGS="-j $((`nproc` - 1))"
          alias mut="pros --no-sentry --no-analytics mut --after run"
          alias mu="pros --no-sentry --no-analytics mu"
          alias t="pros --no-sentry --no-analytics t"
        '';
      };
      formatter.${system} = pkgs.nixpkgs-fmt;
    };
}

