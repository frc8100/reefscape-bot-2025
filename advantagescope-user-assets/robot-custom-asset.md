# AdvantageScope User Assets - Robot Custom Asset

## Overview

This document describes how to use custom assets for this robot in AdvantageScope.

> [!INFO]
> See [AdvantageScope User Assets](https://docs.advantagescope.org/more-features/custom-assets/) for more information about user assets.

## Installation

1. Download and extract the CAD files from [GitHub releases](https://github.com/frc8100/reefscape-bot-2025/releases/latest)
    - There should be 4 `.glb` files in the release zip.

<!-- You can now either extract it to this project directory (under `/advantagescope-user-assets/Robot_WCPCC`) and make a symlink to the AdvantageScope userAssets directory, or copy the files to the AdvantageScope userAssets directory (along with the `config.json` file).

For the symlink method:

2. Extract the zip file to the `advantagescope-user-assets/Robot_WCPCC` directory.
    - The `config.json` file should be in the same directory as the `.glb` files.
    - The directory should look like this:
    ```
    advantagescope-user-assets/
    ├── Robot_WCPCC/
    │   ├── config.json
    │   ├── model.glb
    │   ├── model_0.glb
    │   ├── model_1.glb
    │   └── model_2.glb
    ```

3. Create a symlink to the AdvantageScope userAssets directory.
    - On Windows, use the following command in the command prompt *(NOT POWERSHELL)* as admin (when in the project directory):

    ```
    mklink /D "%APPDATA%\AdvantageScope\userAssets\Robot_WCPCC" ".\advantagescope-user-assets\Robot_WCPCC"
    ```

(or)

3. Create a symlink from the `config.json` file to the AdvantageScope userAssets directory.
    - On Windows, use the following command in the command prompt *(NOT POWERSHELL)* as admin (when in the project directory):

    ```
    mklink /H "%APPDATA%\AdvantageScope\userAssets\Robot_WCPCC\config.json" ".\advantagescope-user-assets\Robot_WCPCC\config.json"
    ```
-->

2. Copy the `.glb` files and `config.json` file (in `advantagescope-user-assets/Robot_WCPCC`) to the AdvantageScope userAssets directory.
    - The directory should look like this:
    ```
    %APPDATA%\AdvantageScope\userAssets\Robot_WCPCC\
    ├── config.json
    ├── model.glb
    ├── model_0.glb
    ├── model_1.glb
    └── model_2.glb
    ```

## Usage

> [!INFO]
> For a quick start, download [AdvantageScope-layout.json](./AdvantageScope-layout.json) and import it into AdvantageScope. (File > Import Layout...)

1. Open the AdvantageScope application.
2. Go to the `3D Field` tab.
3. Click on the `Robot` Poses arrow
4. Select the `WCP Competitive Concept 2025` robot.
