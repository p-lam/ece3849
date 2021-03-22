# invoke SourceDir generated makefile for rtos.pem4f
rtos.pem4f: .libraries,rtos.pem4f
.libraries,rtos.pem4f: package/cfg/rtos_pem4f.xdl
	$(MAKE) -f C:\Users\ADAMYA~1\OneDrive\Desktop\3849_workspace\ece3849d20_lab2_ayang_plam/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\ADAMYA~1\OneDrive\Desktop\3849_workspace\ece3849d20_lab2_ayang_plam/src/makefile.libs clean

