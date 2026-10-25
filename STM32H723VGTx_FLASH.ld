/*
******************************************************************************
**

**  File        : LinkerScript.ld
**
**  Author		: STM32CubeMX
**
**  Abstract    : Linker script for STM32H723VGTx series
**                1024Kbytes FLASH and 368Kbytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
*****************************************************************************
** @attention
**
** <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
**
** Redistribution and use in source and binary forms, with or without modification,
** are permitted provided that the following conditions are met:
**   1. Redistributions of source code must retain the above copyright notice,
**      this list of conditions and the following disclaimer.
**   2. Redistributions in binary form must reproduce the above copyright notice,
**      this list of conditions and the following disclaimer in the documentation
**      and/or other materials provided with the distribution.
**   3. Neither the name of STMicroelectronics nor the names of its contributors
**      may be used to endorse or promote products derived from this software
**      without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
** SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
** CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
** OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM) + LENGTH(RAM);    /* end of RAM */
/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x5000;      /* required amount of heap  */
_Min_Stack_Size = 0x5000; /* required amount of stack */

/* Specify the memory areas */
MEMORY
{
DTCMRAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 128K
RAM (xrw)      : ORIGIN = 0x24000000, LENGTH = 128K
RAM_D2 (xrw)      : ORIGIN = 0x30000000, LENGTH = 32K
RAM_D3 (xrw)      : ORIGIN = 0x38000000, LENGTH = 16K
ITCMRAM (xrw)      : ORIGIN = 0x00000000, LENGTH = 64K
/* 末尾 2×128KB(扇区 6/7)留给"通用参数存储"(bsp/param), 所以应用区收到 768K */
FLASH (rx)      : ORIGIN = 0x8000000, LENGTH = 768K
PARAM_A (r)     : ORIGIN = 0x080C0000, LENGTH = 128K
PARAM_B (r)     : ORIGIN = 0x080E0000, LENGTH = 128K
}

/* Define output sections */
SECTIONS
{
  __mc02_flash_start__ = ORIGIN(FLASH);
  __mc02_flash_end__ = ORIGIN(FLASH) + LENGTH(FLASH);
  __mc02_param_a_start__ = ORIGIN(PARAM_A);
  __mc02_param_a_end__ = ORIGIN(PARAM_A) + LENGTH(PARAM_A);
  __mc02_param_b_start__ = ORIGIN(PARAM_B);
  __mc02_param_b_end__ = ORIGIN(PARAM_B) + LENGTH(PARAM_B);
  ASSERT(ORIGIN(PARAM_A) == ORIGIN(FLASH) + LENGTH(FLASH),
         "参数A分区必须紧接应用Flash区域")
  ASSERT(ORIGIN(PARAM_B) == ORIGIN(PARAM_A) + LENGTH(PARAM_A),
         "参数B分区必须紧接参数A分区")
  ASSERT(LENGTH(PARAM_A) == 0x20000 && LENGTH(PARAM_B) == 0x20000,
         "H723参数分区必须各占一个128KiB扇区")
  ASSERT(__mc02_param_b_end__ == 0x08100000,
         "参数分区必须位于H723 1MiB Flash末端")
  __mc02_dtcm_start__ = ORIGIN(DTCMRAM);
  __mc02_dtcm_end__ = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);
  __mc02_main_ram_start__ = ORIGIN(RAM);
  __mc02_main_ram_end__ = ORIGIN(RAM) + LENGTH(RAM);
  __mc02_ram_d2_start__ = ORIGIN(RAM_D2);
  __mc02_ram_d2_end__ = ORIGIN(RAM_D2) + LENGTH(RAM_D2);
  __mc02_ram_d3_start__ = ORIGIN(RAM_D3);
  __mc02_ram_d3_end__ = ORIGIN(RAM_D3) + LENGTH(RAM_D3);

  /* 崩溃记录固定在主RAM起点；启动清零只覆盖 .bss，不触碰这512字节。 */
  .crash_record ORIGIN(RAM) (NOLOAD) :
  {
    . = ALIGN(32);
    __mc02_crash_record_start__ = .;
    KEEP(*(.crash_record.log))
    KEEP(*(.crash_test_arm))
    KEEP(*(.crash_record.recovery))
    KEEP(*(.crash_record))
    . = __mc02_crash_record_start__ + 0x200;
    __mc02_crash_record_end__ = .;
  } >RAM
  ASSERT(SIZEOF(.crash_record) == 0x200, "崩溃记录保留区必须为512字节")
  ASSERT(ADDR(.crash_record) == ORIGIN(RAM), "崩溃记录保留区必须从主RAM起点开始")
  ASSERT(__mc02_crash_record_end__ == ORIGIN(RAM) + 0x200,
         "崩溃记录保留区结束地址不符合固定布局")

  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* 链接器生成的 Build ID 保存在 Flash，启动时复制到 RAM 身份记录。 */
  .note.gnu.build-id :
  {
    . = ALIGN(4);
    __mc02_build_id_note_start__ = .;
    KEEP(*(.note.gnu.build-id))
    . = ALIGN(4);
    __mc02_build_id_note_end__ = .;
  } >FLASH

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data goes into FLASH */
  .rodata :
  {
    . = ALIGN(4);
    KEEP(*(.rodata.mc02_observation_identity))
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data : 
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM AT> FLASH

  /* 日志队列由BSPLogInit清零，静态任务栈和控制块由BSPLogPortInit清零。 */
  .log_workspace (NOLOAD) :
  {
    . = ALIGN(32);
    __log_workspace_start__ = .;
    KEEP(*(.log_workspace))
    . = ALIGN(32);
    __log_workspace_end__ = .;
  } >DTCMRAM
  ASSERT(SIZEOF(.log_workspace) <= 0x4000, "logger workspace exceeds 16 KiB DTCM budget")

  /* CAN固定队列位于RAM_D2；BSP在首次使用时显式清零该NOLOAD区域。 */
  .can_workspace (NOLOAD) :
  {
    . = ALIGN(32);
    __can_workspace_start__ = .;
    KEEP(*(.can_workspace))
    . = ALIGN(32);
    __can_workspace_end__ = .;
  } >RAM_D2
  ASSERT(SIZEOF(.can_workspace) <= 0x8000, "CAN workspace exceeds 32 KiB RAM_D2 budget")

  
  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss secion */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM

  /* 崩溃现场段: 启动代码的 FillZerobss 只清 _sbss.._ebss, 所以这一段的内容
     在系统复位(看门狗/软件复位/NRST)后依然保留 —— 崩溃记录就放在这里,
     下次启动时能被读出来。注意: 掉电后内容丢失。 */
  .noinit (NOLOAD) :
  {
    . = ALIGN(4);
    *(.noinit)
    *(.noinit*)
    . = ALIGN(4);
  } >RAM

  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}


