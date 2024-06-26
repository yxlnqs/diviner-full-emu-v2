# diviner-full-emu-v2

also note : everything with // is sth i added so n1 needs to buy any shitware from diviner.


for all the people asking how sh1t diviner is : 

 in his bar controller ( src/pcileech_tlps128_bar_controller.sv ) he / the dev of the sh1t he resold used an 8 bit register for the bar which is complelty shit bcs the 2.5gbe uses 577 eeproms and not 81 
 so he builds 1 of 16 bar ranges ( ip/pcileech_bar_zero4k.coe he builds the first 16 x 16 range when it has 16 🔥 ) 

also check `pcie_7x/pcie_7x_0_core_top.v`
