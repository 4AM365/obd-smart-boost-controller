# Purpose 
My attempt at the easiest, best, cheapest boost controller ever. No wiring, no tuning, no blown 4L60s.

Uses CAN data and math to apply boost only when the 4L60E can take it. Self-learns and improves over time.



# How to use
Install the dongle on OBD2 port under dash. Pass the boost controller wire through the firewall and mount it in the engine bay. 

Use it as a four-port.

.000000000000000000000000000000000000000000000000000000000000000000000000000000000000

# HARDWARE
MCU: STM32F103C8T6

CAN Transceiver: MCP2551

Power supply: 5V LDO

OBD DLC male: 179631-1

PCB connector for MAC: 1724470202 Molex

Wire-side connector for MAC: 0436450200