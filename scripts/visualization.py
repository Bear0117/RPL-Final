import json
import open3d as o3d
import numpy as np

def create_colored_box(center, extent, color):
    """
    Create a colored box mesh given a center and half-extents.
    extent = [ext_x, ext_y, ext_z] are half-sizes of the box.
    """
    # Open3D's create_mesh_box expects full dimensions, not half-extents.
    box = o3d.geometry.TriangleMesh.create_box(width=2*extent[0], 
                                               height=2*extent[1], 
                                               depth=2*extent[2])
    # Translate the box so that its center is at the given center
    # By default, create_box() creates a box centered at (0,0,0) with corners at [0,w], [0,h], [0,d].
    # Actually, create_box() places the box origin at a corner. We need to shift it by half the size.
    translation = np.array(center) - np.array([extent[0], extent[1], extent[2]])
    box.translate(translation)
    
    # Assign color
    box.paint_uniform_color(color)
    return box

def create_camera_frustum(camera_translation, camera_rotation, 
                          fx=500, fy=500, cx=320, cy=240, width=640, height=480, 
                          depth=5.0):
    """
    Create a representation of the camera:
    - A line set representing the frustum rays.
    - A small rectangle representing the image plane.

    camera_translation: [x,y,z] of camera in world coords
    camera_rotation: 3x3 rotation matrix from camera to world frame
                     (If given camera->world, points in camera frame 
                      can be transformed by R and then translated.)

    The camera coordinate system is assumed to be:
    - Forward: Z+
    - Right: X+
    - Down: Y+

    The image plane at depth=1.0 means we consider the image plane located
    at Z=+1 in camera space.
    """
    cam_center = np.array(camera_translation)
    R = np.array(camera_rotation)

    # Compute the corners of the image plane in camera coords
    # In pinhole: x = (u - cx)*Z/fx, y = (v - cy)*Z/fy, with Z=depth
    corners_2d = np.array([
        [0, 0],      # top-left pixel
        [width, 0],  # top-right pixel
        [width, height],   # bottom-right pixel
        [0, height]        # bottom-left pixel
    ])

    # Project pixel coords into camera coords at Z=depth
    corners_3d_cam = []
    for (u,v) in corners_2d:
        x = (u - cx)*depth/fx
        y = (v - cy)*depth/fy
        z = depth
        corners_3d_cam.append([x, y, z])
    corners_3d_cam = np.array(corners_3d_cam)

    # Transform corners from camera frame to world frame
    # world_point = R * cam_point + translation
    # Here, R is camera->world (assuming given rotation_matrix is camera->world)
    corners_3d_world = (R @ corners_3d_cam.T).T + cam_center

    # Create line segments from camera center to each corner of the image plane
    points = [cam_center] + list(corners_3d_world)
    points = np.array(points)
    # Points:
    # 0: camera center
    # 1-4: corners of image plane
    lines = [
        [0,1],
        [0,2],
        [0,3],
        [0,4],
        # and also the edges of the image plane:
        [1,2],
        [2,3],
        [3,4],
        [4,1]
    ]

    # Line set
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines)
    )
    # Color for lines: let's make them green
    colors = [[0,1,0] for _ in lines]
    line_set.colors = o3d.utility.Vector3dVector(colors)

    return line_set

def create_colored_box(center, extent, color, label=None, description=None):
    # 確保 extent 為正數，設置最小值
    extent = [max(e, 0.01) for e in extent]

    # 創建邊界框
    box = o3d.geometry.TriangleMesh.create_box(
        width=2 * extent[0],
        height=2 * extent[1],
        depth=2 * extent[2],
    )

    # 設置邊界框的中心和顏色
    box.translate(center)
    box.paint_uniform_color(color)
    return box

if __name__ == "__main__":
    # Load the ConceptGraph JSON
    with open("./CG_CERB/obj_json_r_mapping_stride10.json", "r") as f:
        concept_graph = json.load(f)

    # Create bounding boxes for each node
    geometries = []
    color_palette = [
        [1,0,0],
        [0,1,0],
        [0,0,1],
        [1,1,0],
        [1,0,1],
        [0,1,1]
    ]

    # objects  = concept_graph.get("objects", [])
    # for i, obj  in enumerate(objects):
    #     object_id = obj["id"]
    #     object_tag = obj["object_tag"]
    #     bbox_center = obj["bbox_center"]
    #     bbox_extent = obj["bbox_extent"]
    #     color = color_palette[object_id % len(color_palette)]
    #     box = create_colored_box(bbox_center, bbox_extent, color, tag=object_tag)
    #     geometries.append(box)

    for key, obj in concept_graph.items():
        bbox_center = obj["bbox_center"]
        bbox_extent = obj["bbox_extent"]
        color = color_palette[obj["id"] % len(color_palette)]

        try:
            box = create_colored_box(bbox_center, bbox_extent, color)
            geometries.append(box)
        except Exception as e:
            print(f"Error creating box for object {obj['id']}: {e}")

    # Load camera poses
    with open("camera_poses.json", "r") as f:
        camera_poses = json.load(f)

    # Create frustums for each camera
    for cam in camera_poses.get("cameras", []):
        camera_position = cam["camera_position"]
        camera_orientation = cam["camera_orientation"]
        translation = np.array([camera_position["x"], camera_position["y"], camera_position["z"]])
        yaw = np.deg2rad(camera_orientation["yaw"])
        pitch = np.deg2rad(camera_orientation["pitch"])
        roll = np.deg2rad(camera_orientation["roll"])
        rotation_matrix = np.array([
            [np.cos(yaw) * np.cos(pitch), np.cos(yaw) * np.sin(pitch) * np.sin(roll) - np.sin(yaw) * np.cos(roll), np.cos(yaw) * np.sin(pitch) * np.cos(roll) + np.sin(yaw) * np.sin(roll)],
            [np.sin(yaw) * np.cos(pitch), np.sin(yaw) * np.sin(pitch) * np.sin(roll) + np.cos(yaw) * np.cos(roll), np.sin(yaw) * np.sin(pitch) * np.cos(roll) - np.cos(yaw) * np.sin(roll)],
            [-np.sin(pitch), np.cos(pitch) * np.sin(roll), np.cos(pitch) * np.cos(roll)]
        ])
        frustum = create_camera_frustum(translation, rotation_matrix)
        geometries.append(frustum)





        # translation = cam["translation"]
        # rotation_matrix = np.array(cam["rotation_matrix"])
        # frustum = create_camera_frustum(translation, rotation_matrix)
        # geometries.append(frustum)

    # Visualize
    o3d.visualization.draw_geometries(geometries)
