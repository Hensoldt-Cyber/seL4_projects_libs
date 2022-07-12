#
# Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
#
# SPDX-License-Identifier: BSD-2-Clause
#

cmake_minimum_required(VERSION 3.8.2)

project(libsel4vm C)

set(configure_string "")

add_compile_options(-std=gnu99)

config_option(
    LibSel4VMDeferMemoryMap
    LIB_SEL4VM_DEFER_MEMORY_MAP
    "Defer mappings of memory reservations until faulted on"
    DEFAULT
    OFF
)
config_option(LibSel4VMVMXTimerDebug LIB_VM_VMX_TIMER_DEBUG "Use VMX Pre-Emption timer for debugging
    Will cause a regular vmexit to happen based on VMX pre-emption
    timer. At each exit the guest state will be printed out. This
    can be used to aid debugging when running a guest causes nothing
    to appear to happen" DEFAULT OFF DEPENDS "KernelArchX86")

config_string(
    LibSel4VMVMXTimerTimeout
    LIB_VM_VMX_TIMER_TIMEOUT
    "Cycles between VMX timer pre-emptions
    Cycles between timer exits. Time spent loading the VMCS context
    count, so setting this too low may result in the guest making
    no progress"
    DEFAULT
    4000
    DEPENDS
    "LibSel4VMVMXTimerDebug"
)

mark_as_advanced(LibSel4VMDeferMemoryMap LibSel4VMVMXTimerDebug LibSel4VMVMXTimerTimeout)

add_config_library(sel4vm "${configure_string}")

file(
    GLOB
        sources
        src/*.c
        src/sel4_arch/${KernelSel4Arch}/*.c
)

if(KernelArchX86)
    list(
        APPEND
            sources
            src/arch/x86/boot.c
            src/arch/x86/debug.c
            src/arch/x86/ept.c
            src/arch/x86/guest_irq_controller.c
            src/arch/x86/guest_vcpu_fault_arch.c
            src/arch/x86/guest_x86_context.c
            src/arch/x86/halt.c
            src/arch/x86/interrupt.c
            src/arch/x86/ioports.c
            src/arch/x86/vm.c
            src/arch/x86/vmcall.c
            src/arch/x86/vmcs.c
            src/arch/x86/vmx_timer.c
            src/arch/x86/processor/cpuid.c
            src/arch/x86/processor/cr.c
            src/arch/x86/processor/decode.c
            src/arch/x86/processor/lapic.c
            src/arch/x86/processor/msr.c
            src/arch/x86/i8259/i8259.c
    )
elseif(KernelArchARM)
    list(
        APPEND
            sources
            src/arch/arm/boot.c
            src/arch/arm/fault.c
            src/arch/arm/guest_arm_context.c
            src/arch/arm/guest_irq_controller.c
            src/arch/arm/guest_memory_arch.c
            src/arch/arm/guest_vcpu_fault_arch.c
            src/arch/arm/mem_abort.c
            src/arch/arm/syscalls.c
            src/arch/arm/vm.c
            src/arch/arm/vgic/vgic.c
    )
elseif(KernelArchRiscV)
    # no support yet
else()
    message(FATAL_ERROR "unknown KernelArch '${KernelArch}'")
endif()


add_library(sel4vm STATIC EXCLUDE_FROM_ALL ${sources})
target_include_directories(
    sel4vm
    PUBLIC include arch_include/${KernelArch} sel4_arch_include/${KernelSel4Arch}
)
target_include_directories(
    sel4vm
    PRIVATE src src/arch/${KernelArch} src/sel4_arch/${KernelSel4Arch}
)
target_link_libraries(
    sel4vm
    muslc
    sel4
    sel4simple
    utils
    sel4utils
    sel4vka
    sel4vspace
    platsupport
    sel4_autoconf
    sel4vm_Config
)
