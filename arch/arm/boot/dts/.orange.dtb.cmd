cmd_arch/arm/boot/dts/orange.dtb := ../arm-eabi-4.8/bin/arm-eabi-gcc -E -Wp,-MD,arch/arm/boot/dts/.orange.dtb.d.pre.tmp -nostdinc -I/home/oleg_texet/krnl/android_kernel_archos_persimmon_3_10/arch/arm/boot/dts -I/home/oleg_texet/krnl/android_kernel_archos_persimmon_3_10/arch/arm/boot/dts/include -undef -D__DTS__ -x assembler-with-cpp -o arch/arm/boot/dts/.orange.dtb.dts.tmp arch/arm/boot/dts/orange.dts ; /home/oleg_texet/krnl/android_kernel_archos_persimmon_3_10/scripts/dtc/dtc -O dtb -o arch/arm/boot/dts/orange.dtb -b 0 -i arch/arm/boot/dts/ -i /home/oleg_texet/krnl/android_kernel_archos_persimmon_3_10/drivers/misc/mediatek/mach/mt6735/orange/dct/dct/ -d arch/arm/boot/dts/.orange.dtb.d.dtc.tmp arch/arm/boot/dts/.orange.dtb.dts.tmp ; cat arch/arm/boot/dts/.orange.dtb.d.pre.tmp arch/arm/boot/dts/.orange.dtb.d.dtc.tmp > arch/arm/boot/dts/.orange.dtb.d

source_arch/arm/boot/dts/orange.dtb := arch/arm/boot/dts/orange.dts

deps_arch/arm/boot/dts/orange.dtb := \
  arch/arm/boot/dts/mt6735m.dtsi \
    $(wildcard include/config/base.h) \
    $(wildcard include/config/addr.h) \
  /home/oleg_texet/krnl/android_kernel_archos_persimmon_3_10/drivers/misc/mediatek/mach/mt6735/orange/dct/dct/cust_eint.dtsi \

arch/arm/boot/dts/orange.dtb: $(deps_arch/arm/boot/dts/orange.dtb)

$(deps_arch/arm/boot/dts/orange.dtb):
