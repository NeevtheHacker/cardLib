# Installation

You need the PROS CLI (PROS 4, kernel 4.2.2 or newer). Check with:

```bash
pros --version
```

There are two ways to get cardLib. Pick one.

## Option A: start from the starter project (recommended for a new robot)

1. Download `cardLib-starter@0.1.0.zip` from the latest GitHub Release.
2. Unzip it and open the folder in VS Code.
3. Build it:

    ```bash
    pros make
    ```

The starter already contains cardLib, a `robot/` folder for your ports and devices, and example autons. Go to [Your first project](first-project.md).

## Option B: add cardLib to an existing PROS project

1. Download `cardLib@0.1.0.zip` from the latest GitHub Release into your project folder.
2. From inside your project folder:

    ```bash
    pros c fetch cardLib@0.1.0.zip
    pros c apply cardLib
    ```

3. Include the library in `include/main.h`, below `#include "api.h"`:

    ```cpp
    #include "cardLib/api.hpp"
    ```

4. Build:

    ```bash
    pros make
    ```

!!! warning "Using LemLib at the same time"
    cardLib and LemLib both want to own the drive motors. Use one or the other for the drivetrain.

## Updating

Fetch the new zip and apply it again:

```bash
pros c fetch cardLib@0.2.0.zip
pros c apply cardLib --force-apply
```

Read the [changelog](../project/changelog.md) first: a new major version can change the API.

## Building cardLib yourself

If you cloned the repository instead of downloading a release:

```bash
cd cardLib            # the inner folder, the PROS project
pros make template    # makes cardLib@<version>.zip
```

Laptop tests (no robot needed), from the same folder:

```bash
cmake -S tests -B build/tests
cmake --build build/tests
ctest --test-dir build/tests
```
