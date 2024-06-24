import open3d as o3d
import numpy as np

def apply_rotation(pcd, axis, angle):
    """
    Apply rotation to the point cloud.
    :param pcd: open3d.geometry.PointCloud
    :param axis: str, 'x', 'y', or 'z'
    :param angle: float, angle in degrees
    :return: rotated point cloud
    """
    R = pcd.get_rotation_matrix_from_xyz(np.radians((angle if axis == 'x' else 0, 
                                                     angle if axis == 'y' else 0, 
                                                     angle if axis == 'z' else 0)))
    pcd.rotate(R, center=(0, 0, 0))
    return pcd

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
    pcd_file = '/home/jerrywang/sentry-navigation/slam_ws/src/mid360_localization/FAST-LIO-SAM-QN/fast_lio_sam_qn/results/openlab.pcd'  # Path to your PCD file
    pcd = o3d.io.read_point_cloud(pcd_file)

    print("Point cloud loaded. Use the following commands to rotate the point cloud:")
    print("'x <angle>' to rotate around the x-axis")
    print("'y <angle>' to rotate around the y-axis")
    print("'z <angle>' to rotate around the z-axis")
    print("'save <file_name>' to save the corrected point cloud")
    print("'point_size <size>' to change the point size in the visualization")
    print("'quit' to exit")

    point_size = 1  # Default point size

    while True:
        visualize_point_cloud(pcd, point_size)
        user_input = input("Enter command: ").strip().split()
        if len(user_input) == 2:
            command = user_input[0]
            if command in ['x', 'y', 'z']:
                try:
                    angle = float(user_input[1])
                    pcd = apply_rotation(pcd, command, angle)
                except ValueError:
                    print("Invalid angle. Please enter a numeric value.")
            elif command == 'point_size':
                try:
                    point_size = int(user_input[1])
                except ValueError:
                    print("Invalid point size. Please enter an integer value.")
            else:
                print("Invalid command. Please enter 'x', 'y', 'z', or 'point_size'.")
        elif len(user_input) == 2 and user_input[0] == 'save':
            save_file = user_input[1]
            o3d.io.write_point_cloud(save_file, pcd)
            print(f"Point cloud saved as {save_file}")
        elif len(user_input) == 1 and user_input[0] == 'quit':
            break
        else:
            print("Invalid command. Please follow the instructions.")

if __name__ == '__main__':
    main()
