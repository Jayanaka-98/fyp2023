cmd_/home/linux/Documents/malithjkd/sdk_826_linux_3.3.13/driver/Module.symvers := sed 's/\.ko$$/\.o/' /home/linux/Documents/malithjkd/sdk_826_linux_3.3.13/driver/modules.order | scripts/mod/modpost -m -a  -o /home/linux/Documents/malithjkd/sdk_826_linux_3.3.13/driver/Module.symvers -e -i Module.symvers   -T -
