# naya-ctrl: small text command interface on a second USB serial port.
if(CONFIG_NAYA_CTRL)
  target_sources(app PRIVATE src/naya_ctrl.c)
endif()
