################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
main.obj: ../main.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/tms470_4.9.5/bin/cl470" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -g --include_path="C:/CMSIS/CMSIS/Include" --include_path="C:/ti/ccsv5/tools/compiler/tms470_4.9.5/include" --include_path="C:/CMSIS/Device/ARM/ARMCM4/Include" --include_path="C:/StellarisWare/utils" --include_path="C:/StellarisWare" --gcc --define=PART_LM4F120H5QR --define=ccs="ccs" --define=ARM_MATH_CM4 --define=__FPU_PRESENT --define=TARGET_IS_BLIZZARD_RA2 --diag_warning=225 --display_error_number --gen_func_subsections=off --ual --preproc_with_compile --preproc_dependency="main.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

mmc-ek-lm4f232h5qd.obj: ../mmc-ek-lm4f232h5qd.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/tms470_4.9.5/bin/cl470" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -g --include_path="C:/CMSIS/CMSIS/Include" --include_path="C:/ti/ccsv5/tools/compiler/tms470_4.9.5/include" --include_path="C:/CMSIS/Device/ARM/ARMCM4/Include" --include_path="C:/StellarisWare/utils" --include_path="C:/StellarisWare" --gcc --define=PART_LM4F120H5QR --define=ccs="ccs" --define=ARM_MATH_CM4 --define=__FPU_PRESENT --define=TARGET_IS_BLIZZARD_RA2 --diag_warning=225 --display_error_number --gen_func_subsections=off --ual --preproc_with_compile --preproc_dependency="mmc-ek-lm4f232h5qd.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

startup_ccs.obj: ../startup_ccs.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/tms470_4.9.5/bin/cl470" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -g --include_path="C:/CMSIS/CMSIS/Include" --include_path="C:/ti/ccsv5/tools/compiler/tms470_4.9.5/include" --include_path="C:/CMSIS/Device/ARM/ARMCM4/Include" --include_path="C:/StellarisWare/utils" --include_path="C:/StellarisWare" --gcc --define=PART_LM4F120H5QR --define=ccs="ccs" --define=ARM_MATH_CM4 --define=__FPU_PRESENT --define=TARGET_IS_BLIZZARD_RA2 --diag_warning=225 --display_error_number --gen_func_subsections=off --ual --preproc_with_compile --preproc_dependency="startup_ccs.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

uartstdio.obj: ../uartstdio.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/tms470_4.9.5/bin/cl470" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -g --include_path="C:/CMSIS/CMSIS/Include" --include_path="C:/ti/ccsv5/tools/compiler/tms470_4.9.5/include" --include_path="C:/CMSIS/Device/ARM/ARMCM4/Include" --include_path="C:/StellarisWare/utils" --include_path="C:/StellarisWare" --gcc --define=PART_LM4F120H5QR --define=ccs="ccs" --define=ARM_MATH_CM4 --define=__FPU_PRESENT --define=TARGET_IS_BLIZZARD_RA2 --diag_warning=225 --display_error_number --gen_func_subsections=off --ual --preproc_with_compile --preproc_dependency="uartstdio.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

usb_msc_structs.obj: ../usb_msc_structs.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/tms470_4.9.5/bin/cl470" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -g --include_path="C:/CMSIS/CMSIS/Include" --include_path="C:/ti/ccsv5/tools/compiler/tms470_4.9.5/include" --include_path="C:/CMSIS/Device/ARM/ARMCM4/Include" --include_path="C:/StellarisWare/utils" --include_path="C:/StellarisWare" --gcc --define=PART_LM4F120H5QR --define=ccs="ccs" --define=ARM_MATH_CM4 --define=__FPU_PRESENT --define=TARGET_IS_BLIZZARD_RA2 --diag_warning=225 --display_error_number --gen_func_subsections=off --ual --preproc_with_compile --preproc_dependency="usb_msc_structs.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

usbdsdcard.obj: ../usbdsdcard.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/tms470_4.9.5/bin/cl470" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -g --include_path="C:/CMSIS/CMSIS/Include" --include_path="C:/ti/ccsv5/tools/compiler/tms470_4.9.5/include" --include_path="C:/CMSIS/Device/ARM/ARMCM4/Include" --include_path="C:/StellarisWare/utils" --include_path="C:/StellarisWare" --gcc --define=PART_LM4F120H5QR --define=ccs="ccs" --define=ARM_MATH_CM4 --define=__FPU_PRESENT --define=TARGET_IS_BLIZZARD_RA2 --diag_warning=225 --display_error_number --gen_func_subsections=off --ual --preproc_with_compile --preproc_dependency="usbdsdcard.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

ustdlib.obj: C:/StellarisWare/utils/ustdlib.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv5/tools/compiler/tms470_4.9.5/bin/cl470" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -g --include_path="C:/CMSIS/CMSIS/Include" --include_path="C:/ti/ccsv5/tools/compiler/tms470_4.9.5/include" --include_path="C:/CMSIS/Device/ARM/ARMCM4/Include" --include_path="C:/StellarisWare/utils" --include_path="C:/StellarisWare" --gcc --define=PART_LM4F120H5QR --define=ccs="ccs" --define=ARM_MATH_CM4 --define=__FPU_PRESENT --define=TARGET_IS_BLIZZARD_RA2 --diag_warning=225 --display_error_number --gen_func_subsections=off --ual --preproc_with_compile --preproc_dependency="ustdlib.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


