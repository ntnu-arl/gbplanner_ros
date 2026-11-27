"""Convert the retained SMB RGB camera macros to native Fortress sensors."""
import math
import xml.etree.ElementTree as ET


CAMERAS = ('rgb_cam_front', 'rgb_cam_left', 'rgb_cam_right')
OPTICAL_RPY = (-math.pi / 2., 0., -math.pi / 2.)


def _text(parent, name, value):
    element = ET.SubElement(parent, name)
    element.text = str(value)
    return element


def add_native_cameras(robot, source):
    """Append source-configured RGB sensors and ROS optical frames to a URDF."""
    extensions = {camera: [] for camera in CAMERAS}
    for extension in source.findall('gazebo'):
        camera = extension.get('reference')
        sensor = extension.find('sensor')
        if camera in extensions and sensor is not None:
            plugin = sensor.find('plugin')
            if plugin is not None and plugin.get('filename') == 'libgazebo_ros_camera.so':
                extensions[camera].append((sensor, plugin))

    existing = {(element.tag, element.get('name')) for element in robot
                if element.tag in ('link', 'joint')}
    for camera in CAMERAS:
        definitions = extensions[camera]
        if len(definitions) != 1:
            raise ValueError('Expected exactly one original SMB camera: ' + camera)
        original, classic = definitions[0]
        if robot.find("link[@name='" + camera + "']") is None:
            raise ValueError('Missing preserved physical camera body: ' + camera)

        image = original.find('camera/image')
        clip = original.find('camera/clip')
        noise = original.find('camera/noise')
        if (original.get('type') != 'camera' or image is None or clip is None or noise is None or
                classic.findtext('cameraName') != camera or
                classic.findtext('imageTopicName') != 'image_raw' or
                classic.findtext('cameraInfoTopicName') != 'camera_info' or
                classic.findtext('frameName') != camera):
            raise ValueError('Unexpected source SMB camera contract: ' + camera)
        if (image.findtext('format') != 'R8G8B8' or
                noise.findtext('type') != 'gaussian' or
                abs(float(noise.findtext('stddev')) - .007) > 1.e-12):
            raise ValueError('Unexpected source SMB image format/noise: ' + camera)

        width = int(image.findtext('width'))
        height = int(image.findtext('height'))
        rate = float(original.findtext('update_rate'))
        horizontal_fov = float(original.findtext('camera/horizontal_fov'))
        if width != 1440 or height != 1080 or abs(rate - 20.) > 1.e-9:
            raise ValueError('Unexpected source SMB image size/rate: ' + camera)

        optical = camera + '_optical_frame'
        joint_name = camera + '_optical_joint'
        if (('link', optical) in existing or ('joint', joint_name) in existing):
            raise ValueError('Duplicate SMB optical frame: ' + optical)
        existing.update((('link', optical), ('joint', joint_name)))
        ET.SubElement(robot, 'link', name=optical)
        joint = ET.SubElement(robot, 'joint', name=joint_name, type='fixed')
        ET.SubElement(joint, 'parent', link=camera)
        ET.SubElement(joint, 'child', link=optical)
        ET.SubElement(joint, 'origin', xyz='0 0 0', rpy=' '.join(map(str, OPTICAL_RPY)))

        image_topic = '/' + camera + '/image_raw'
        info_topic = '/' + camera + '/camera_info'
        extension = ET.SubElement(robot, 'gazebo', reference=camera)
        native = ET.SubElement(extension, 'sensor', name=camera + '_camera', type='camera')
        _text(native, 'always_on', 'true')
        _text(native, 'update_rate', rate)
        _text(native, 'topic', image_topic)
        native_camera = ET.SubElement(native, 'camera')
        _text(native_camera, 'horizontal_fov', horizontal_fov)
        native_image = ET.SubElement(native_camera, 'image')
        _text(native_image, 'width', width)
        _text(native_image, 'height', height)
        _text(native_image, 'format', 'R8G8B8')
        native_clip = ET.SubElement(native_camera, 'clip')
        _text(native_clip, 'near', float(clip.findtext('near')))
        _text(native_clip, 'far', float(clip.findtext('far')))
        native_noise = ET.SubElement(native_camera, 'noise')
        _text(native_noise, 'type', 'gaussian')
        _text(native_noise, 'mean', float(noise.findtext('mean')))
        _text(native_noise, 'stddev', float(noise.findtext('stddev')))
        _text(native_camera, 'camera_info_topic', info_topic)
        _text(native_camera, 'optical_frame_id', optical)

    if any(len(extensions[camera]) != 1 for camera in CAMERAS):
        raise ValueError('Expected all three original SMB RGB cameras')
