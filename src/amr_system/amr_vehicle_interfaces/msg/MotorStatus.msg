# Physical node mapping: LEFT=1, RIGHT=0. RPM is forward-positive wheel RPM.
uint8 node_id
amr_vehicle_interfaces/DataStatus encoder
float64 wheel_rpm
# Heartbeat carries the CANSimple axis state/error prefix.
amr_vehicle_interfaces/DataStatus heartbeat
uint32 error_code
uint32 axis_state
amr_vehicle_interfaces/DataStatus power
float64 input_voltage_v
float64 bus_current_a
amr_vehicle_interfaces/DataStatus temperature
float64 driver_temperature_c
