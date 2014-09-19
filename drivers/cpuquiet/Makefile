GCOV_PROFILE := y

subdir-ccflags-y += -Werror
obj-$(CONFIG_CPUQUIET_FRAMEWORK) += cpuquiet.o driver.o sysfs.o cpuquiet_attribute.o governor.o governors/

ifeq ($(CONFIG_ARM),y)
obj-$(CONFIG_TEGRA_CPUQUIET) += tegra/
endif
