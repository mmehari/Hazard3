# SoC integration file

file example_soc.v

# CPU + debug components

list $HDL/hazard3.f
list $HDL/debug/dtm/hazard3_jtag_dtm.f
list $HDL/debug/dm/hazard3_dm.f

# RISC-V timer

list peri/hazard3_riscv_timer.f
 
# USB communications device class

file peri/usb_cdc/usb_cdc/phy_tx.v
file peri/usb_cdc/usb_cdc/phy_rx.v
file peri/usb_cdc/usb_cdc/sie.v
file peri/usb_cdc/usb_cdc/ctrl_endp.v
file peri/usb_cdc/usb_cdc/bulk_endp.v
file peri/usb_cdc/usb_cdc/in_fifo.v
file peri/usb_cdc/usb_cdc/out_fifo.v
file peri/usb_cdc/usb_cdc/usb_cdc.v
file peri/usb_cdc_apb.v
file peri/gpio_apb.v
 
# AHB3-Lite hosted scope

file peri/ahbscope.v

# Generic SoC components from libfpga

file ../libfpga/common/reset_sync.v

list ../libfpga/peris/uart/uart.f
list ../libfpga/peris/spi_03h_xip/spi_03h_xip.f
list ../libfpga/mem/ahb_cache.f
list ../libfpga/mem/ahb_sync_sram.f

list ../libfpga/busfabric/ahbl_crossbar.f
file ../libfpga/busfabric/ahbl_to_apb.v
file ../libfpga/busfabric/apb_splitter.v

