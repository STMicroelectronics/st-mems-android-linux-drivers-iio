# Introduction
This repository contains STMicroelectronics MEMS IIO drivers for the following LTS Linux/Android kernels:
- 4.9 (EOL)
- 4.14
- 4.19
- 5.4
- 5.10
- 5.15
- 6.1
- 6.6
- 6.12
- 6.18 (WIP)

It is intended as a reference source tree for STMicroelectronics Industrial I/O (IIO) sensor drivers and related integration assets. The repository includes drivers for multiple sensor classes such as accelerometers, IMUs, magnetometers, pressure sensors, temperature sensors, humidity sensors, and other STM sensing devices.

The project can be used in two main ways: it can be integrated directly into a Linux/Android kernel source tree, or it can be used to build the STM driver subtree as external modules against a prepared kernel build directory. In addition to the driver sources, the repository also provides configuration fragments, kernel-version-specific integration patches, and supporting documentation.

Maintenance for EOL kernels has been discontinued, and their use is strongly discouraged for new developments.

## Table of contents
- [Project layout](#project-layout)
- [Source code integration](#source-code-integration)
- [Apply patches](#apply-patches)
- [Configuration](#configuration)
- [Device tree configuration](#device-tree-configuration)
- [Build modules](#build-modules)
- [License](#license)

## Project layout
The repository is organized to support both source integration into a kernel tree and standalone external-module builds.

### Repository tree overview
```text
stm-ldd-iio/
|-- Documentation/
|   `-- devicetree/
|       `-- bindings/
|-- drivers/
|   `-- iio/
|       `-- stm/
|           |-- accel/
|           |-- common/
|           |-- humidity/
|           |-- imu/
|           |-- magnetometer/
|           |-- pressure/
|           |-- temperature/
|           `-- tmos/
|-- stm_iio_configs/
|-- stm_iio_patches/
|   |-- 4.14.y/
|   |-- 5.4.y/
|   |-- 6.1.y/
|   `-- ...
|-- build.sh
|-- test_checkpatch.sh
|-- README.md
`-- LICENSE.txt
```

### Top-level content
- [drivers/](drivers/): main source tree for STM IIO drivers.
- [Documentation/](Documentation/): additional documentation, including device-tree bindings.
- [stm_iio_configs/](stm_iio_configs/): defconfig fragments for supported sensors and reference platforms.
- [stm_iio_patches/](stm_iio_patches/): kernel-version-specific patches required to integrate STM drivers into a target kernel tree.
- [build.sh](build.sh): helper script for build automation.
- [test_checkpatch.sh](test_checkpatch.sh): helper script to run style and patch-format checks.
- [README.md](README.md), [CONTRIBUTING.md](CONTRIBUTING.md), [LICENSE.txt](LICENSE.txt): repository documentation and project metadata.

### Driver source tree
The main driver sources are under [drivers/iio/stm/](drivers/iio/stm/) and are grouped by sensor family:

- [accel/](drivers/iio/stm/accel/): accelerometers.
- [humidity/](drivers/iio/stm/humidity/): humidity sensors.
- [imu/](drivers/iio/stm/imu/): IMUs and combo inertial devices.
- [magnetometer/](drivers/iio/stm/magnetometer/): magnetometers.
- [pressure/](drivers/iio/stm/pressure/): pressure sensors.
- [temperature/](drivers/iio/stm/temperature/): temperature sensors.
- [tmos/](drivers/iio/stm/tmos/): thermal/object-presence related sensors.
- [common/](drivers/iio/stm/common/): shared compatibility headers and common definitions used by multiple drivers.

Each category directory typically contains its own `Kconfig`, `Kbuild`, and `Makefile`, plus one or more sensor-specific subdirectories (for example [st_lis2dw12/](drivers/iio/stm/accel/st_lis2dw12/), [st_lsm6dsox/](drivers/iio/stm/imu/st_lsm6dsox/), [st_lps22hh/](drivers/iio/stm/pressure/st_lps22hh/), and similar).

### Kernel integration assets
Two repository areas are especially relevant when importing the project into a kernel tree:

- [stm_iio_patches/<kernel-version>/](stm_iio_patches/): patches that update the target kernel build system and, where required, extend the IIO framework with STM-specific additions.
- [stm_iio_configs/](stm_iio_configs/): ready-to-merge configuration fragments that enable the corresponding STM drivers in the target kernel configuration.

### In-tree vs out-of-tree usage
- **In-tree**: follow [Source code integration](#source-code-integration), then apply the patch sets in [stm_iio_patches/](stm_iio_patches/).
- **Out-of-tree**: clone the repository in a local directory and build the drivers with the [drivers/iio/stm/](drivers/iio/stm/) Makefiles.
- In out-of-tree use, the project-local [LICENSE.txt](LICENSE.txt) applies to the repository contents.

This separation allows the same repository to be used both as a development source tree and as an integration package for multiple supported kernel baselines.

## Source code integration
From your kernel source directory, add a git remote (for example `stmems_iio_github`) for this repository:
```bash
git remote add stmems_iio_github \
               https://github.com/STMicroelectronics/st-mems-android-linux-drivers-iio.git
```

Fetch the newly added remote:
```bash
git fetch stmems_iio_github
```

At this point, there are two ways to integrate the driver code into the target kernel branch:
- merge (**recommended strategy**)
- rebase

> [!IMPORTANT]
> **In-tree integration risk (merge/rebase with kernel source):**
> This repository also contains top-level project files (for example [README.md](README.md) and [LICENSE.txt](LICENSE.txt)).
> When integrating with the kernel tree using merge/rebase, a conflict on [README.md](README.md) may occur because some kernel trees already provide their own top-level `README.md`.
> The operation may also introduce a new [LICENSE.txt](LICENSE.txt) file that refers to this ST project repository and not to the Linux kernel licensing files.
> Please review and resolve these files explicitly during conflict resolution.

### Merge
Merge `stmems_iio_github/master` into your target kernel source branch (for example `linux-5.4.y`):

```bash
git merge --allow-unrelated-histories \
          linux-5.4.y \
          stmems_iio_github/master
```

### Rebase
Rebase `stmems_iio_github/master` on top of your target kernel source branch (for example `linux-5.4.y`):

```bash
git rebase --no-fork-point \
           linux-5.4.y \
           stmems_iio_github/master
```

Note: older git versions (for example `2.7.4`) may require slightly different options:

```bash
git merge --no-fork-point \
          linux-5.4.y \
          stmems_iio_github/master
```

```bash
git rebase -Xno-renames \
           --no-fork-point \
           linux-5.4.y \
           stmems_iio_github/master
```

## Apply patches
Once the driver code has been added to the target kernel branch, a small set of additional patches must be applied in order to:
- add STM drivers to the Kconfig and Makefile build system;
- extend the IIO framework with STM-specific events, channels, and devices where required.

Apply the patches available in the newly added repository (for example under [stm_iio_patches/5.4.y/](stm_iio_patches/5.4.y/)):

```bash
git am stm_iio_patches/5.4.y/*-stm-*.patch
```

## Configuration
The [stm_iio_configs/](stm_iio_configs/) directory provides default configuration fragments for the supported drivers.

### Modify target defconfig
A sensor defconfig can be appended to the board defconfig (for example, if the current board configuration file is `arch/arm/configs/stm32_defconfig`):

```bash
cat stm_iio_configs/lsm6dsm_defconfig >> arch/arm/configs/stm32_defconfig
```

Alternatively, the driver configuration can be merged at build time without modifying the board defconfig file.

### Merge configuration
The driver configuration can be merged into an already prepared target kernel configuration using the `merge_config.sh` script provided by the kernel:

```bash
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnu-
scripts/kconfig/merge_config.sh -n .config stm_iio_configs/lsm6dsm_defconfig
```

## Device tree configuration
After enabling the driver in the kernel configuration, the corresponding device must also be described in the platform device tree so that the kernel can instantiate it correctly.

In general, enabling an STM sensor through device tree requires:

- a node on the correct bus (`i2c` or `spi`);
- a valid `compatible` string matching the target driver;
- the correct `reg` value for the device address or chip select;
- interrupt wiring (`interrupt-parent` and `interrupts`) when data-ready or trigger support is required;
- optional power supplies such as `vdd-supply` and `vddio-supply`, if they are used on the board;
- optional properties such as `st,int-pin`, `mount-matrix`, `st,module_id`, and other feature-specific flags supported by the driver.

The exact set of supported and required properties depends on the specific sensor driver. Always refer to the corresponding binding documentation under [Documentation/devicetree/bindings/](Documentation/devicetree/bindings/).

### Example: LSM6DSVX
For the `st_lsm6dsvx` IMU family, the reference binding is available in [Documentation/devicetree/bindings/iio/stm/imu/st_lsm6dsvx.txt](Documentation/devicetree/bindings/iio/stm/imu/st_lsm6dsvx.txt).

Example for an I2C-connected device (`SA0` tied to ground):

```dts
lsm6dsvx-imu@6a {
    compatible = "st,lsm6dsv16x";
    reg = <0x6a>;
    interrupt-parent = <&gpio0>;
    interrupts = <0 IRQ_TYPE_LEVEL_HIGH>;
    vddio-supply = <&sensors_vddio>;
    vdd-supply = <&sensors_vdd>;
    st,int-pin = <1>;
    mount-matrix = "1", "0", "0",
                   "0", "1", "0",
                   "0", "0", "1";
    st,module_id = <1>;
};
```

Example for an SPI-connected device:

```dts
lsm6dsvx-imu@0 {
    compatible = "st,lsm6dsv16x";
    reg = <0x0>;
    spi-max-frequency = <10000000>;
    interrupt-parent = <&gpio0>;
    interrupts = <0 IRQ_TYPE_LEVEL_HIGH>;
    vddio-supply = <&sensors_vddio>;
    vdd-supply = <&sensors_vdd>;
    st,int-pin = <1>;
    mount-matrix = "1", "0", "0",
                   "0", "1", "0",
                   "0", "0", "1";
    st,module_id = <2>;
};
```

For additional optional properties supported by this device family, including sensor hub, QVAR, open-drain interrupt configuration, wake-up support, and compatible string variants (`st,lsm6dsv`, `st,lsm6dsv16x`, `st,lsm6dsv32x`), refer to the binding documentation file.

## Build modules
This repository includes Makefiles for both in-tree and out-of-tree module builds.

### In-kernel-tree build
Set `CROSS_COMPILE` and `ARCH` according to the target board. Example for an RPI 4 target board (32-bit):
```bash
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnu-
make KCFLAGS=-Werror -j $(nproc)
```

### Out-of-kernel-tree build
The drivers in this project can also be built as external modules.

#### Prerequisites
Before starting an out-of-tree build, make sure that:

- the target kernel source/build directory is already configured for the same target architecture/toolchain;
- the directory referenced by `KDIR` already contains the kernel build artifacts needed for external modules (for example `.config`, generated headers, and the usual files produced by `make modules_prepare` or by a full kernel build);
- the selected kernel version is compatible with the STM driver sources you are trying to build;
- if your target kernel requires STM integration patches, those patches are still needed for functional integration even if the drivers are compiled out-of-tree.

#### Environment variables
Example for an RPI 4 target board (32-bit):

```bash
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnu-
export KDIR=<the directory where Linux kernel is built>

# If INSTALL_MOD_PATH is set, the module installation path becomes:
# /INSTALL_MOD_PATH/lib/modules/$(KERNELRELEASE)/kernel/
# instead of /lib/modules/$(KERNELRELEASE)/extra/
export INSTALL_MOD_PATH=<modules installation path>

# Set INSTALL_MOD_DIR to use an alternative name instead of "extra":
# /lib/modules/$(KERNELRELEASE)/INSTALL_MOD_DIR/ instead of standard
# /lib/modules/$(KERNELRELEASE)/extra/
export INSTALL_MOD_DIR=<modules alternate dir>
```

#### Conditional compilation flags
Several STM drivers support optional build-time features controlled through `CONFIG_*` options.
In out-of-tree mode, these options are typically set in the driver-local `Makefile` through `KBUILD_OPTIONS += ...`, and then consumed in the local `Kbuild` via patterns such as:

- `ccflags-$(CONFIG_<FEATURE>) += -DCONFIG_<FEATURE>` (adds compile-time defines);
- `<driver>-$(CONFIG_<FEATURE>) += <feature_file>.o` (adds feature-specific objects).

Recommended workflow:

1. Open the driver-local Makefile (for example [drivers/iio/stm/imu/st_lsm6dsvx/Makefile](drivers/iio/stm/imu/st_lsm6dsvx/Makefile)).
2. Set the required `KBUILD_OPTIONS` values (`y`, `m`, or `n` depending on the option type).
3. Rebuild the module from that driver directory or from [drivers/iio/stm/](drivers/iio/stm/).

Example (`lsm6dsvx`) in [drivers/iio/stm/imu/st_lsm6dsvx/Makefile](drivers/iio/stm/imu/st_lsm6dsvx/Makefile):

```make
KBUILD_OPTIONS += CONFIG_IIO_ST_LSM6DSVX_ASYNC_HW_TIMESTAMP=y
KBUILD_OPTIONS += CONFIG_IIO_ST_LSM6DSVX_MLC_PRELOAD=y
KBUILD_OPTIONS += CONFIG_IIO_ST_LSM6DSVX_QVAR_IN_FIFO=y
```

These options are mapped in [drivers/iio/stm/imu/st_lsm6dsvx/Kbuild](drivers/iio/stm/imu/st_lsm6dsvx/Kbuild) to conditional flags and objects.

Notes:

- Keep option names aligned between `Makefile` and `Kbuild`.
- If an option is not supported by the target kernel configuration, force-enabling it may produce build errors.
- For reproducible builds, commit the intended `KBUILD_OPTIONS` values in the relevant driver-local Makefile.

#### Supported targets
The [drivers/iio/stm/Makefile](drivers/iio/stm/Makefile) can be invoked with the usual module-oriented targets, for example:

- `modules`: build all STM external modules;
- `clean`: remove generated objects for the STM external-module build;
- `modules_install`: install the generated `.ko` files into the selected module destination.

#### Build example
Build all STM modules with warnings treated as errors:

```bash
make KCFLAGS=-Werror -C drivers/iio/stm/ modules -j$(nproc)
```

Build all STM modules and place module object files in a separate directory:

```bash
export BUILD_DIR=/tmp/stm-iio-build
mkdir -p "$BUILD_DIR"
make KCFLAGS=-Werror -C drivers/iio/stm/ MO=$BUILD_DIR modules -j$(nproc)
```

By default, generated module objects (`*.ko`) are produced inside the corresponding driver subdirectories under [drivers/iio/stm/](drivers/iio/stm/). When `MO` is set, build artifacts are redirected to the specified build directory.

#### Install example
Install the generated modules:

```bash
make -C drivers/iio/stm/ modules_install
```

#### End-to-end example
Example sequence for a prepared kernel build directory:

```bash
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnu-
export KDIR=/path/to/kernel/build
export INSTALL_MOD_PATH=/tmp/stm-modules
export INSTALL_MOD_DIR=stm-iio

make KCFLAGS=-Werror -C drivers/iio/stm/ modules -j$(nproc)
make -C drivers/iio/stm/ modules_install
```

After installation, the modules will typically be available under a path similar to:

```text
$INSTALL_MOD_PATH/lib/modules/$(KERNELRELEASE)/$INSTALL_MOD_DIR/
```

#### Notes and limitations

- The out-of-tree build flow is intended to compile the STM driver subtree as external modules; it does not replace kernel-side integration work when additional framework or Kconfig/Makefile changes are required.
- If the target kernel lacks required STM-specific IIO changes, a successful module compilation alone may still be insufficient for runtime integration.
- Use the same compiler, architecture and configuration context used for the target kernel build to avoid ABI or symbol-version mismatches.

### License
This software is distributed under the GNU General Public License v2.0

The project is released under GPLv2 to remain fully compatible with the Linux kernel licensing model and with in-tree kernel integration workflows. This choice also aligns the driver sources with the broader kernel driver ecosystem, where derivative works and redistributed binaries are expected to preserve source availability under the same copyleft terms.

Note: the repository [LICENSE.txt](LICENSE.txt) file is project-specific metadata for this ST driver package and is not a replacement for Linux kernel licensing documentation.
