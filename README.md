# CH32V003-UART-DMA-RX-RISCV-ASSEMBLY-EXAMPLE
get lightwieght RISCV assembler from https://github.com/theandrew168/bronzebeard#setup , details attached
connect CH32V005 to terminal via dongle . outputs a string on startup. On receiving input of 8 bytes from erminal DMA transfer interrupt fires and received bytes are transmitted back to terminal inside the ISR.
PD5 TX , PD6 RX
