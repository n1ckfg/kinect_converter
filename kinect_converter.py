import numpy as np
import open3d as o3d
import cv2
import ffmpeg
import os

def get_intrinsics(name="generic", mode="default"):
    name = name.lower()
    mode = mode.lower()

    # Generic
    # https://en.wikipedia.org/wiki/Camera_resectioning
    # https://docs.opencv.org/4.x/dc/d3a/classcv_1_1viz_1_1Camera.html#a753809aff611cdcc5a8a377126156b05
    cx = 320 # principal point x
    cy = 240 # principal point y
    fx = 525 # focal length x
    fy = 525 # focal length y
    width = 640
    height = 480

    if (name == "primesense"):
        cx = 319.5
        cy = 239.5
        fx = 525.0
        fy = 525.0
    elif (name == "kinect_2"):
        # https://medium.com/yodayoda/from-depth-map-to-point-cloud-7473721d3f
        cx = 254.878
        cy = 205.395
        fx = 365.456
        fy = 365.456
    elif (name == "azure_kinect"):
        # https://forum.open3d.org/t/azure-kinect-intrinsic-structure/121
        # TODO modes
        cx = 2044.2911376953125
        cy = 1565.8837890625
        fx = 1959.37890625
        fy = 1958.880126953125
    elif (name == "apple_truedepth"):
        # http://nghiaho.com/?p=2629
        cx = 322.18
        cy = 240.03
        fx = 434.89
        fy = 434.89
    elif (name == "apple_lidar"):
        # https://developer.apple.com/forums/thread/663995
        cx = 233.9272925 # 960 / 3840 * 935.70917
        cy = 178.40451 # 720 / 2880 * 713.61804
        fx = 399.585 # 960 / 3840 * 1598.34
        fy = 399.585 # 720 / 2880 * 1598.34
        width = 960
        height = 720
        '''
        cx = 130.122 # 534 / 3840 * 935.70917
        cy = 95.149 # 384 / 2880 * 713.61804
        fx = 222.269 # 534 / 3840 * 1598.34
        fy = 213.112 # 384 / 2880 * 1598.34
        width = 534
        height = 384
        '''
    elif (name == "orbbec_astra"):
        # https://github.com/raulmur/ORB_SLAM2/issues/524
        cx = 308.189
        cy = 233.283
        fx = 503.642
        fy = 512.275
    elif (name == "orbbec_astra_mini"):
        # https://3dclub.orbbec3d.com/t/what-are-the-intrinsic-camera-parameters-for-the-orbbec-astra-mini/1616/2
        cx = 320.1
        cy = 247.6
        fx = 535.4
        fy = 539.2
    elif (name == "realsense_D435"):
        if (mode == "640x360"):
            # https://answers.ros.org/question/363236/intel-realsense-d435-intrinsic-parameters/
            cx = 320.818268
            cy = 178.779297
            fx = 322.282410
            fy = 322.282410
            width = 640
            height = 360
        elif (mode == "640x480"):
            # https://support.intelrealsense.com/hc/en-us/community/posts/4403651641491-about-ros-camera-calibration-and-d435-on-chip-calibration-values
            cx = 320.90576171875
            cy = 235.999221801758
            fx = 617.034790039063
            fy = 617.2119140625
            width = 640
            height = 480
        elif (mode == "848x480"):
            # https://github.com/IntelRealSense/realsense-ros/issues/1661
            cx = 423.014007568359
            cy = 239.275390625
            fx = 426.795440673828
            fy = 426.795440673828
            width = 848
            height = 480
        elif (mode == "1280x720"):
            # https://github.com/IntelRealSense/realsense-ros/issues/1661
            cx = 638.51171875
            cy = 358.90625
            fx = 644.219543457031
            fy = 644.219543457031
            width = 1280
            height = 720

    return cx, cy, fx, fy, width, height

def extract_frames(url, input_type, fps=24, rgb_dir="input_rgb", depth_dir="input_depth"):
    input = ffmpeg.input(url)
    video = input.video.filter("fps", fps)
    video_rgb = None
    video_depth = None

    if (input_type == "record3d"):
        video_rgb = ffmpeg.crop(video, 0, 0, 720, 960)
        video_rgb = video_rgb.filter("transpose", 2) # rotate -90 deg
        video_depth = ffmpeg.crop(video, 720, 0, 720, 960)
        video_depth = video_depth.filter("transpose", 2) # rotate -90 deg
    elif (input_type == "holoflix"):
        video_rgb = ffmpeg.crop(video, 640, 120, 640, 480)
        video_depth = ffmpeg.crop(video, 0, 120, 640, 480)
    else:
        video_rgb = video
        video_depth = video
    
    output_rgb = ffmpeg.output(video_rgb, os.path.join(rgb_dir, "rgb_%05d.png"), start_number=0)
    output_rgb.run(overwrite_output=True)

    output_depth = ffmpeg.output(video_depth, os.path.join(depth_dir, "depth_%05d.png"), start_number=0)
    output_depth.run(overwrite_output=True)    

def do_inpainting(imDepth):
    mask = cv2.cvtColor(imDepth, cv2.COLOR_BGR2GRAY)
    ret, mask = cv2.threshold(mask, 16, 255, cv2.THRESH_BINARY_INV)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=1)
    imDepth = cv2.inpaint(imDepth, mask, 3, cv2.INPAINT_TELEA) # source, mask, radius, method (TELEA or NS)
    return imDepth

def point_cloud_from_rgbd(rgb, depth, name="generic", mode="default"):
    #rgb = o3d.io.read_image(rgbUrl)
    #depth = o3d.io.read_image(depthUrl)
    rgb = cv2_to_o3d(rgb)
    depth = cv2_to_o3d(depth)
    cx, cy, fx, fy, width, height = get_intrinsics(name, mode)
    width = int(rgb.get_max_bound()[0])
    height = int(rgb.get_max_bound()[1])
    #intrinsics = o3d.io.read_pinhole_camera_intrinsic("intrinsics.json")
    intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx=fx, fy=fy, cx=cx, cy=cy)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb, depth)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics)

    return pcd

def mesh_from_point_cloud(pcd):
	pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
	mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd)
	return mesh

def xyz_from_point_cloud(pcd, u, v, width):
    loc = u + v * width
    return pcd.points[loc]

def save_point_cloud(pcd, name="output.ply"):
    o3d.io.write_point_cloud(name, pcd)

def save_mesh(mesh, name="output.ply"):
	o3d.io.write_triangle_mesh(name, mesh)

def cv2_to_o3d(img):
    return o3d.geometry.Image(np.asarray(img))

# old method with no open3d dependency
def uvd_to_xyz(u, v, d, depth_scale=1, name="generic", mode="default"):
    cx, cy, fx, fy, width, height = get_intrinsics(name, mode)

    '''
    d *= scale
    x_over_z = (cx - u) / fx
    y_over_z = (cy - v) / fy

    z = d / np.sqrt(1. + x_over_z**2 + y_over_z**2)
    x = x_over_z * z
    y = y_over_z * z

    # open3d version
    # http://www.open3d.org/docs/0.6.0/python_api/open3d.geometry.create_point_cloud_from_rgbd_image.html
    '''
    z = d * depth_scale
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    #print((x, y, z))

    return (x, y, z)