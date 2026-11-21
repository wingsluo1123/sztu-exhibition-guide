ground_z = -1.0
point_z_values = [-1.0, -0.8, -0.3]

for point_z in point_z_values:
    height = point_z - ground_z
    print(f"原 z={point_z:.1f} 米，离地高度={height:.1f} 米")
