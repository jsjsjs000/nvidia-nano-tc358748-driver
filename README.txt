# -------------------- Device Tree - Kernel setup --------------------
code ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/Makefile
# ----------------------------------------
dtbo-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-all-p3449-0000-tc358748.dtbo
# ----------------------------------------

code ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/tegra210-p3448-0000-p3449-0000-b00.dts
# ----------------------------------------
#include "porg-platforms/tegra210-porg-camera-rbpcv4-dual-tc358748.dtsi"
# ----------------------------------------

# -------------------- Device Tree backup --------------------
backup=~/nvidia-nano-tc358748-driver
mkdir -p $backup/dts/
cp ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/tegra210-p3448-all-p3449-0000-tc358748.dts $backup/dts/
cp ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/porg-platforms/tegra210-porg-camera-rbpcv4-dual-tc358748.dtsi $backup/dts/
cp ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/porg-platforms/tegra210-camera-rbpcv4-dual-tc358748.dtsi $backup/dts/

# -------------------- Device Tree restore --------------------
backup=~/nvidia-nano-tc358748-driver
cp $backup/dts/tegra210-p3448-all-p3449-0000-tc358748.dts     ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/
cp $backup/dts/tegra210-porg-camera-rbpcv4-dual-tc358748.dtsi ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/porg-platforms/
cp $backup/dts/tegra210-camera-rbpcv4-dual-tc358748.dtsi      ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/porg-platforms/


# -------------------- Driver - Kernel setup --------------------
code ~/l4t-gcc/Linux_for_Tegra/source/public/kernel/kernel-4.9/drivers/media/i2c/Makefile
# ----------------------------------------
obj-$(CONFIG_VIDEO_TC358748)	+= tc358748.o
# ----------------------------------------

code ~/l4t-gcc/Linux_for_Tegra/source/public/kernel/kernel-4.9/drivers/media/i2c/Kconfig
# ----------------------------------------
config VIDEO_TC358748
	tristate "Toshiba TC358748 parallel to CSI-2 converter"
	depends on VIDEO_V4L2 && I2C && VIDEO_V4L2_SUBDEV_API
	---help---
	  Support for the Toshiba TC358748 parallel to MIPI CSI-2 bridge.

	  To compile this driver as a module, choose M here: the
	  module will be called tc358748.
# ----------------------------------------

# -------------------- Driver backup --------------------
backup=~/nvidia-nano-tc358748-driver
mkdir -p $backup/driver/
cp ~/l4t-gcc/Linux_for_Tegra/source/public/kernel/kernel-4.9/drivers/media/i2c/tc358748.c $backup/driver/
cp ~/l4t-gcc/Linux_for_Tegra/source/public/kernel/nvidia/include/media/tc358748.h $backup/driver/

# -------------------- Driver restore --------------------
backup=~/nvidia-nano-tc358748-driver
cp $backup/driver/tc358748.c           ~/l4t-gcc/Linux_for_Tegra/source/public/kernel/kernel-4.9/drivers/media/i2c/
cp $backup/driver/tc358748.h           ~/l4t-gcc/Linux_for_Tegra/source/public/kernel/nvidia/include/media/

# -------------------- Backup patch --------------------
backup=~/nvidia-nano-tc358748-driver
mkdir -p $backup/patch/
cp ~/l4t-gcc/Linux_for_Tegra/source/public/kernel/kernel-4.9/0001-regmap-add-formats.patch $backup/patch/

# -------------------- Apply patch --------------------
cd kernel/kernel-4.9/
cp $backup/patch/0001-regmap-add-formats.patch .
git apply 0001-regmap-add-formats.patch
cd -
