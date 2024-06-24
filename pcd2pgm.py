import open3d as o3d
import numpy as np
import cv2

def filter_point_cloud_by_z(pcd, z_min, z_max):
    """
    Filter points based on z-coordinate.
    :param pcd: open3d.geometry.PointCloud
    :param z_min: float, minimum z-coordinate to include
    :param z_max: float, maximum z-coordinate to include
    :return: filtered point cloud
    """
    points = np.asarray(pcd.points)
    filtered_points = points[(points[:, 2] > z_min) & (points[:, 2] < z_max)]
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
    return filtered_pcd

def pcd_to_2d_occupancy_grid(pcd, resolution):
    points = np.asarray(pcd.points)

    # Extract x, y coordinates and project to 2D
    x_coords = points[:, 0]
    y_coords = points[:, 1]

    # Determine the spatial extent
    x_min, x_max = x_coords.min(), x_coords.max()
    y_min, y_max = y_coords.min(), y_coords.max()

    # Calculate width and height in cells
    width = int((x_max - x_min) / resolution)
    height = int((y_max - y_min) / resolution)

    # Create an empty occupancy grid
    occupancy_grid = np.ones((height, width), dtype=np.uint8) * 255  # Free space is white (255)

    # Scale the coordinates to the grid size
    x_scaled = ((x_coords - x_min) / (x_max - x_min) * (width - 1)).astype(int)
    y_scaled = ((y_coords - y_min) / (y_max - y_min) * (height - 1)).astype(int)

    # Invert y-coordinates to correct the mirroring issue
    y_scaled = height - y_scaled - 1

    # Fill the occupancy grid with obstacles (black)
    occupancy_grid[y_scaled, x_scaled] = 0

    return occupancy_grid

def save_pgm(grid_data, file_name):
    # Save the occupancy grid as a PGM file
    cv2.imwrite(file_name + '.pgm', grid_data)
    cv2.imwrite(file_name + '.png', grid_data)

def visualize_point_cloud(pcd, point_size=1):
    """
    Visualize the point cloud with coordinate axes and specified point size.
    :param pcd: open3d.geometry.PointCloud
    :param point_size: int, size of the points
    """
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    vis.add_geometry(axes)
    render_option = vis.get_render_option()
    render_option.point_size = point_size
    vis.run()
    vis.destroy_window()

def main():
    pcd_file = '/home/jerrywang/sentry-navigation/slam_ws/src/mid360_localization/third_party/FAST_LIO/PCD/openlab.pcd'  # Path to your PCD file
    resolution = 0.05  # Resolution in meters
    pgm_file_name = input("Enter the output filename: ")  # Output file name

    pcd = o3d.io.read_point_cloud(pcd_file)
    
    while True:
        try:
            z_min = float(input("Enter minimum z-coordinate to include (default: -0.6): "))
            z_max = float(input("Enter maximum z-coordinate to include (default: 1.0): "))
        except ValueError:
            print("Invalid input. Please enter numeric values for z_min and z_max.")
            continue

        filtered_pcd = filter_point_cloud_by_z(pcd, z_min, z_max)
        visualize_point_cloud(filtered_pcd)

        confirm = input("Do you want to save the filtered point cloud? (yes/no): ").strip().lower()
        if confirm == 'yes':
            occupancy_grid = pcd_to_2d_occupancy_grid(filtered_pcd, resolution)
            save_pgm(occupancy_grid, pgm_file_name)
            print(f"PGM file saved as {pgm_file_name}")
            break
        elif confirm == 'no':
            print("You can re-enter z_min and z_max values.")
        else:
            print("Invalid input. Please enter 'yes' or 'no'.")

if __name__ == '__main__':
    main()
