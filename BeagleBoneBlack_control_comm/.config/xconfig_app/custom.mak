## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,ea8fnv linker.cmd package/cfg/app_pea8fnv.oea8fnv

linker.cmd: package/cfg/app_pea8fnv.xdl
	$(SED) 's"^\"\(package/cfg/app_pea8fnvcfg.cmd\)\"$""\"C:/CSS_prj/BeagleBoneBlack_control_comm/.config/xconfig_app/\1\""' package/cfg/app_pea8fnv.xdl > $@
