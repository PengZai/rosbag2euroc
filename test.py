import yaml

# Define the dictionary
IMU_SENSOR_YAML = {
    "sensor_type": "imu",
    "comment": "VI-Sensor IMU (ADIS16448)",
    "T_BS": {
        "cols": 4,
        "rows": 4,
        "data": [
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        ],
    },
    "rate_hz": 200,
    "gyroscope_noise_density": 1.6968e-04,
    "gyroscope_random_walk": 1.9393e-05,
    "accelerometer_noise_density": 2.0000e-3,
    "accelerometer_random_walk": 3.0000e-3,
}

# Custom YAML Dumper to format lists as inline
class InlineListDumper(yaml.Dumper):
    def increase_indent(self, flow=False, indentless=False):
        return super(InlineListDumper, self).increase_indent(flow, False)

# Save to a .yaml file
file_name = "imu_sensor.yaml"
with open(file_name, "w") as file:
    # Add YAML version header and custom comments
    file.write("%YAML:1.0\n%YAML:1.0\n")
    file.write("#Default imu sensor yaml file\n")
    
    # Dump the dictionary with inline lists
    yaml.dump(
        IMU_SENSOR_YAML,
        file,
        Dumper=InlineListDumper,
        default_flow_style=False,
        sort_keys=False
    )

    # Add additional comments
    file.write(
        "\n# Sensor extrinsics wrt. the body-frame.\n"
        "# inertial sensor noise model parameters (static)\n"
        "gyroscope_noise_density: 1.6968e-04     # [ rad / s / sqrt(Hz) ]   ( gyro \"white noise\" )\n"
        "gyroscope_random_walk: 1.9393e-05       # [ rad / s^2 / sqrt(Hz) ] ( gyro bias diffusion )\n"
        "accelerometer_noise_density: 2.0000e-3  # [ m / s^2 / sqrt(Hz) ]   ( accel \"white noise\" )\n"
        "accelerometer_random_walk: 3.0000e-3    # [ m / s^3 / sqrt(Hz) ].  ( accel bias diffusion )\n"
    )

print(f"Custom formatted YAML file written to {file_name}")
