import math
import pytest

from geometry_msgs.msg import PoseWithCovarianceStamped
from re_localization.srv import ReLocalization
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from offline_common import HERE, ROOT, Runtime


@pytest.mark.parametrize('override,expected', [(None, 0), (2, 2)])
def test_ndt_thread_parameter_default_and_explicit_override(override, expected):
    with Runtime('localization_parameter') as rt:
        arguments = [] if override is None else ['-p', f'ndt_num_threads:={override}']
        rt.start('re_localization', 'pcl_localization_node', arguments)
        client = rt.node.create_client(GetParameters, '/pcl_localization/get_parameters')
        result = rt.call(client, GetParameters.Request(names=['ndt_num_threads']))
        assert len(result.values) == 1 and result.values[0].type == 2
        assert result.values[0].integer_value == expected


def test_ndt_localization_service_and_negative_inputs():
    with Runtime('localization') as rt:
        rt.start('re_localization', 'pcl_localization_node', [
            '--params-file', str(ROOT / 'utils/src/re_localization/param/localization_real.yaml'),
            '-p', f'map_path:={HERE / "localization.pcd"}',
            '-p', 'lidar_topic:=/offline/localization_cloud'])
        parameters = rt.node.create_client(GetParameters, '/pcl_localization/get_parameters')
        actual = rt.call(parameters, GetParameters.Request(names=['ndt_num_threads']))
        assert len(actual.values) == 1 and actual.values[0].type == 2 and actual.values[0].integer_value == 8, \
            f'Configured ndt_num_threads=8 was not applied: {actual.values}'
        rt.lifecycle('pcl_localization', 1)
        rt.lifecycle('pcl_localization', 3)
        client = rt.node.create_client(ReLocalization, '/re_localization')
        request = ReLocalization.Request()
        request.initial_pose = PoseWithCovarianceStamped()
        request.initial_pose.header.frame_id = 'wrong_frame'
        request.initial_pose.pose.pose.orientation.w = 1.
        result = rt.call(client, request)
        assert not result.success
        request.initial_pose.header.frame_id = 'map'
        result = rt.call(client, request, 7.)
        assert not result.success, 'No input cloud must fail, not fabricate a pose'
        points = []
        # Numeric test fixture, independent of original 1.4GB real map.
        for i in range(40):
            for j in range(30):
                u, v = i * .18 - 3.6, j * .12 - .6
                points.extend([(u, -3 + .15 * math.sin(i * .3), v, 100.),
                               (4 + .1 * math.cos(j * .4), u, v, 150.),
                               (u, v * 2, -1 + .07 * math.sin(i * .7 + j * .5), 200.)])
        fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
        cloud = point_cloud2.create_cloud(Header(frame_id='base_link'), fields, points)
        pub = rt.node.create_publisher(PointCloud2, '/offline/localization_cloud', 10)

        def publish():
            cloud.header.stamp = rt.node.get_clock().now().to_msg()
            pub.publish(cloud)

        result = rt.call(client, request, 20., publish)
        assert result.success and math.isfinite(result.fitness_score)
        assert result.fitness_score < .05
        p = result.pose.pose.pose.position
        assert math.sqrt(p.x*p.x + p.y*p.y + p.z*p.z) < .35
        print(f'NDT_OMP fixture registration: score={result.fitness_score:.6f}, pose=({p.x:.4f},{p.y:.4f},{p.z:.4f})')
        # Existing implementation explicitly caches its first cloud; verify that path too.
        result = rt.call(client, request, 20.)
        assert result.success and result.fitness_score < .05
