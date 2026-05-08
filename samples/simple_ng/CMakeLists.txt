add_executable(simple_ng simple_ng.c)
target_link_libraries(simple_ng soem)
add_executable(servo_cli servo_cli.c)
target_link_libraries(servo_cli soem)
add_executable(plc_emulator
  plc_emulator.c
  plc_protocol.c
  plc_tcp_client.c
)
target_link_libraries(plc_emulator soem)
install(TARGETS simple_ng servo_cli plc_emulator DESTINATION bin)
