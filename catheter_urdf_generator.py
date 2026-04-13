#!/usr/bin/env python3
"""
Xacro Generator for Flexible Catheter

This script generates Xacro data for a flexible catheter modeled as a serial link
with N links and N-1 universal joints. The catheter consists of:
- Tip link (length L1)
- Bending section (N-2 links, total length L2)
- Base link (length L3)

Parameters:
- N: Number of links
- D: Outer diameter of catheter
- L1: Length of tip link
- L2: Total length of bending section
- L3: Length of base link
- K: Spring constant for rotary springs
- M: Total mass of catheter
"""

import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom
import math
import argparse
import numpy as np
import os


class CatheterXacroGenerator:
    def __init__(self, N, D, L1, L2, L3, K, Kd, Kf, M, package_dir="catheter_package",
                 anatomy_stl=None, anatomy_xyz=(0, 0, 0), anatomy_rpy=(0, 0, 0),
                 anatomy_scale=0.001):
        self.N = N
        self.D = D
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.K = K
        self.damping = Kd
        self.friction = Kf
        self.M = M
        self.package_dir = package_dir
        self.anatomy_stl = anatomy_stl
        self.anatomy_xyz = anatomy_xyz
        self.anatomy_rpy = anatomy_rpy
        self.anatomy_scale = anatomy_scale
        self.meshes_dir = os.path.join(package_dir, "meshes")
        self.urdf_dir = os.path.join(package_dir, "urdf")
        self.sdf_dir = os.path.join(package_dir, "sdf")
        self.launch_dir = os.path.join(package_dir, "launch")
        self.worlds_dir = os.path.join(package_dir, "worlds")
        
        self.validate_parameters()
        self.calculate_derived_values()
        self.create_package_structure()
    
    def validate_parameters(self):
        """Validate input parameters"""
        if self.N < 3:
            raise ValueError("N must be at least 3 (tip, at least one bending link, base)")
        if any(val <= 0 for val in [self.D, self.L1, self.L2, self.L3, self.K, self.M]):
            raise ValueError("All parameters must be positive")
    
    def calculate_derived_values(self):
        """Calculate derived values from input parameters"""
        self.bending_links = self.N - 2
        #self.bending_link_length = self.L2 / self.bending_links if self.bending_links > 0 else 0
        self.bending_link_length = self.L2 / self.bending_links if self.bending_links > 0 else 0
        self.radius = self.D / 2
        
        # Mass distribution
        self.tip_mass = self.M * (self.L3 / (self.L1 + self.L2 + self.L3))
        self.base_mass = self.M * (self.L1 / (self.L1 + self.L2 + self.L3))
        self.bending_mass_per_link = self.M * (self.L2 / (self.L1 + self.L2 + self.L3)) / self.bending_links if self.bending_links > 0 else 0
    
    def create_package_structure(self):
        """Create package directory structure"""
        if not os.path.exists(self.package_dir):
            os.makedirs(self.package_dir)
        if not os.path.exists(self.meshes_dir):
            os.makedirs(self.meshes_dir)
        if not os.path.exists(self.urdf_dir):
            os.makedirs(self.urdf_dir)
        if not os.path.exists(self.sdf_dir):
            os.makedirs(self.sdf_dir)
        if not os.path.exists(self.launch_dir):
            os.makedirs(self.launch_dir)
        if not os.path.exists(self.worlds_dir):
            os.makedirs(self.worlds_dir)
    
    def generate_cylinder_stl(self, radius, length, filename, resolution=20):
        """Generate STL file for a cylinder"""
        theta = np.linspace(0, 2*np.pi, resolution)
        
        # Generate vertices for top and bottom circles
        top_center = [0, 0, length/2]
        bottom_center = [0, 0, -length/2]
        
        top_circle = [[radius * np.cos(t), radius * np.sin(t), length/2] for t in theta]
        bottom_circle = [[radius * np.cos(t), radius * np.sin(t), -length/2] for t in theta]
        
        vertices = [top_center, bottom_center] + top_circle + bottom_circle
        
        # Generate triangular faces
        faces = []
        n = len(theta)
        
        # Top cap triangles (fan from center)
        for i in range(n):
            faces.append([0, 2 + i, 2 + (i + 1) % n])
        
        # Bottom cap triangles (fan from center)0.00
        for i in range(n):
            faces.append([1, 2 + n + (i + 1) % n, 2 + n + i])
        
        # Side triangles
        for i in range(n):
            next_i = (i + 1) % n
            # Two triangles per side face
            faces.append([2 + i, 2 + n + i, 2 + next_i])
            faces.append([2 + next_i, 2 + n + i, 2 + n + next_i])
        
        # Write STL file
        stl_path = os.path.join(self.meshes_dir, filename)
        with open(stl_path, 'w') as f:
            f.write("solid cylinder\n")
            
            for face in faces:
                # Calculate normal vector
                v1 = np.array(vertices[face[1]]) - np.array(vertices[face[0]])
                v2 = np.array(vertices[face[2]]) - np.array(vertices[face[0]])
                normal = np.cross(v1, v2)
                normal = normal / np.linalg.norm(normal) if np.linalg.norm(normal) > 0 else [0, 0, 1]
                
                f.write(f"  facet normal {normal[0]:.6f} {normal[1]:.6f} {normal[2]:.6f}\n")
                f.write("    outer loop\n")
                for vertex_idx in face:
                    vertex = vertices[vertex_idx]
                    f.write(f"      vertex {vertex[0]:.6f} {vertex[1]:.6f} {vertex[2]:.6f}\n")
                f.write("    endloop\n")
                f.write("  endfacet\n")
            
            f.write("endsolid cylinder\n")
        
        return stl_path
    
    def generate_package_xml(self):
        """Generate package.xml for ROS2 package"""
        package_name = os.path.basename(self.package_dir)
        
        package_xml_content = f'''<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypefile="package_format3.xsd"?>
<package format="3">
  <name>{package_name}</name>
  <version>0.0.0</version>
  <description>Flexible catheter URDF package generated by catheter_urdf_generator</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
    <gazebo_ros gazebo_model_path="${{prefix}}/../"/>
    <gazebo_ros gazebo_media_path="${{prefix}}/media"/>
    <gazebo_ros plugin_path="${{prefix}}/plugins"/>
  </export>
</package>'''
        
        package_xml_path = os.path.join(self.package_dir, "package.xml")
        with open(package_xml_path, 'w') as f:
            f.write(package_xml_content)
        
        return package_xml_path
    
    def generate_cmakelists_txt(self):
        """Generate CMakeLists.txt for ROS2 package"""
        package_name = os.path.basename(self.package_dir)
        
        cmake_content = f'''cmake_minimum_required(VERSION 3.8)
project({package_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${{PROJECT_NAME}}/urdf/
)

# Install mesh files
install(DIRECTORY meshes/
  DESTINATION share/${{PROJECT_NAME}}/meshes/
)

# Install SDF files
install(DIRECTORY sdf/
  DESTINATION share/${{PROJECT_NAME}}/sdf/
)

# Install world files
install(DIRECTORY worlds/
  DESTINATION share/${{PROJECT_NAME}}/worlds/
)

# Install launch files (if any)
install(DIRECTORY launch/
  DESTINATION share/${{PROJECT_NAME}}/launch/
  FILES_MATCHING PATTERN "*.py"
  PATTERN "*.launch.py"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()'''
        
        cmake_path = os.path.join(self.package_dir, "CMakeLists.txt")
        with open(cmake_path, 'w') as f:
            f.write(cmake_content)
        
        return cmake_path
    
    def generate_sdf(self, xacro_filename):
        """Generate SDF file using ElementTree for better structure and validation"""
        package_name = os.path.basename(self.package_dir)
        sdf_filename = xacro_filename.replace('.xacro', '.sdf')
        
        # Create SDF root element
        sdf = ET.Element('sdf', version='1.7')
        model = ET.SubElement(sdf, 'model', name=package_name)
        
        # Add base link
        self._add_sdf_link(model, 'base_link',
                           parent_name=None,
                           pose=f'0 0 {self.L1/2} 0 0 0',
                           mass=self.base_mass,
                           length=self.L1,
                           mesh_file=f'file://{os.path.abspath(os.path.join(self.meshes_dir, "base_link.stl"))}')
        
        # Add bending links
        #current_z = self.L1
        parent_name = 'base_to_bending_1_y'
        for i in range(self.bending_links):
            link_name = f'bending_link_{i+1}'
            self._add_sdf_link(model, link_name,
                               parent_name=parent_name,
                               #parent_name=None,
                               pose=f'0 0 {self.bending_link_length/2} 0 0 0',
                               mass=self.bending_mass_per_link,
                               length=self.bending_link_length,
                               mesh_file=f'model://{package_name}/meshes/bending_link.stl')
            parent_name = f'bending_{i+1}_to_{i+2}_y'
            #current_z += self.bending_link_length
        
        # Add tip link
        parent_name = f'bending_{self.bending_links}_to_tip_x'
        self._add_sdf_link(model, 'tip_link',
                           parent_name=parent_name,
                           pose=f'0 0 {self.L3/2} 0 0 0',
                           mass=self.tip_mass,
                           length=self.L3,
                           mesh_file=f'model://{package_name}/meshes/tip_link.stl')
        
        # Add fixed joint to connect base link to world
        self._add_fixed_joint(model, 'world_to_base', 'world', 'base_link')
        
        # Add universal joints
        joint_names = self._add_universal_joints(model)
        
        # Add joint state publisher plugin
        self._add_joint_state_publisher(model, joint_names)
        
        # Save SDF file with pretty formatting
        self._save_sdf_file(sdf, sdf_filename)
        
        return os.path.join(self.sdf_dir, sdf_filename)
    
    def _add_sdf_link(self, parent, name, parent_name, pose, mass, length, mesh_file):
        """Add a link to SDF using ElementTree"""
        link = ET.SubElement(parent, 'link', name=name)
        
        # Pose
        if parent_name==None:
            ET.SubElement(link, 'pose').text = pose
        else:
            ET.SubElement(link, 'pose', relative_to=parent_name).text = pose
        
        # Inertial properties
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = str(mass)
        inertia = ET.SubElement(inertial, 'inertia')
        
        # Calculate inertia values
        ixx_iyy = (1/12) * mass * (3 * self.radius**2 + length**2)
        izz = 0.5 * mass * self.radius**2
        
        ET.SubElement(inertia, 'ixx').text = str(ixx_iyy)
        ET.SubElement(inertia, 'iyy').text = str(ixx_iyy)
        ET.SubElement(inertia, 'izz').text = str(izz)
        ET.SubElement(inertia, 'ixy').text = '0'
        ET.SubElement(inertia, 'ixz').text = '0'
        ET.SubElement(inertia, 'iyz').text = '0'
        
        # Visual
        visual = ET.SubElement(link, 'visual', name=f'{name}_visual')
        geometry = ET.SubElement(visual, 'geometry')
        #mesh = ET.SubElement(geometry, 'mesh')
        #ET.SubElement(mesh, 'uri').text = mesh_file
        cyl = ET.SubElement(geometry, 'cylinder')
        ET.SubElement(cyl, 'radius').text = f'{self.radius}'
        ET.SubElement(cyl, 'length').text = f'{length}'
        material = ET.SubElement(visual, 'material')
        ET.SubElement(material, 'ambient').text = '0.8 0.8 0.8 1.0'
        ET.SubElement(material, 'diffuse').text = '0.8 0.8 0.8 1.0'
        
        # Collision
        collision = ET.SubElement(link, 'collision', name=f'{name}_collision')
        geometry = ET.SubElement(collision, 'geometry')
        #mesh = ET.SubElement(geometry, 'mesh')
        #ET.SubElement(mesh, 'uri').text = mesh_file
        cyl = ET.SubElement(geometry, 'cylinder')
        ET.SubElement(cyl, 'radius').text = f'{self.radius}'
        ET.SubElement(cyl, 'length').text = f'{length}'

    
    def _add_intermediate_link(self, parent, name, parent_name, pose):
        """Add intermediate link for universal joints"""
        link = ET.SubElement(parent, 'link', name=name)
        if parent_name == None:
            ET.SubElement(link, 'pose').text = pose
        else:
            ET.SubElement(link, 'pose', relative_to=parent_name).text = pose

        # Minimal inertial properties
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = '0.001'
        inertia = ET.SubElement(inertial, 'inertia')
        ET.SubElement(inertia, 'ixx').text = '1e-6'
        ET.SubElement(inertia, 'iyy').text = '1e-6'
        ET.SubElement(inertia, 'izz').text = '1e-6'
        ET.SubElement(inertia, 'ixy').text = '0'
        ET.SubElement(inertia, 'ixz').text = '0'
        ET.SubElement(inertia, 'iyz').text = '0'
    
    def _add_revolute_joint(self, parent, name, parent_link, child_link, axis_xyz, pose='0 0 0 0 0 0'):
        """Add revolute joint with spring dynamics"""
        joint = ET.SubElement(parent, 'joint', name=name, type='revolute')
        ET.SubElement(joint, 'parent').text = parent_link
        ET.SubElement(joint, 'child').text = child_link
        ET.SubElement(joint, 'pose', relative_to=parent_link).text = pose
        
        # Axis
        axis = ET.SubElement(joint, 'axis')
        ET.SubElement(axis, 'xyz').text = axis_xyz
        
        # Limits
        limit = ET.SubElement(axis, 'limit')
        ET.SubElement(limit, 'lower').text = str(-math.pi/2)
        ET.SubElement(limit, 'upper').text = str(math.pi/2)
        ET.SubElement(limit, 'effort').text = '100'
        ET.SubElement(limit, 'velocity').text = '10'
        
        # Dynamics
        dynamics = ET.SubElement(axis, 'dynamics')
        ET.SubElement(dynamics, 'damping').text = f'{self.damping}'
        ET.SubElement(dynamics, 'friction').text = f'{self.friction}'
        ET.SubElement(dynamics, 'spring_stiffness').text = str(self.K)
    
    def _add_fixed_joint(self, parent, name, parent_link, child_link):
        """Add fixed joint"""
        joint = ET.SubElement(parent, 'joint', name=name, type='fixed')
        ET.SubElement(joint, 'parent').text = parent_link
        ET.SubElement(joint, 'child').text = child_link
        ET.SubElement(joint, 'pose').text = '0 0 0 0 0 0'
    
    def _add_universal_joints(self, model):
        """Add universal joints and return joint names for plugin"""
        joint_names = []
        
        if self.bending_links > 0:
            # Base to first bending link
            intermediate_name = 'base_to_bending_1_x_rotation'
            joint_x = 'base_to_bending_1_x'
            joint_y = 'base_to_bending_1_y'
            self._add_revolute_joint(model, joint_x, 'base_link', intermediate_name, '1 0 0', f'0 0 {self.L1/2} 0 0 0')
            self._add_intermediate_link(model,
                                        name=intermediate_name,
                                        parent_name=joint_x,
                                        #parent_name=None,
                                        #pose=f'0 0 {self.L1} 0 0 0')
                                        pose=f'0 0 0 0 0 0')
            self._add_revolute_joint(model, joint_y, intermediate_name, 'bending_link_1', '0 1 0')
            joint_names.extend([joint_x, joint_y])
            
            # Bending link universal joints
            for i in range(1, self.bending_links):
                intermediate_name = f'bending_{i}_to_{i+1}_x_rotation'
                joint_x = f'bending_{i}_to_{i+1}_x'
                joint_y = f'bending_{i}_to_{i+1}_y'
                self._add_revolute_joint(model, joint_x, f'bending_link_{i}', intermediate_name, '1 0 0', f'0 0 {self.bending_link_length/2} 0 0 0')
                self._add_intermediate_link(model,
                                            name=intermediate_name,
                                            parent_name=joint_x,
                                            pose=f'0 0 0 0 0 0')
                self._add_revolute_joint(model, joint_y, intermediate_name, f'bending_link_{i+1}', '0 1 0')
                joint_names.extend([joint_x, joint_y])

            # Last bending link to tip
            final_z = self.L3 + self.bending_links * self.bending_link_length
            intermediate_name = f'bending_{self.bending_links}_to_tip_x_rotation'
            joint_x = f'bending_{self.bending_links}_to_tip_x'
            joint_y = f'bending_{self.bending_links}_to_tip_y'
            self._add_revolute_joint(model, joint_x, f'bending_link_{self.bending_links}', intermediate_name, '1 0 0', f'0 0 {self.bending_link_length/2} 0 0 0')
            self._add_intermediate_link(model,
                                        name=intermediate_name,
                                        parent_name=joint_x,
                                        pose=f'0 0 0 0 0 0')
            self._add_revolute_joint(model, joint_y, intermediate_name, 'tip_link', '0 1 0')
            joint_names.extend([joint_x, joint_y])
        else:
            # Direct base to tip universal joint
            intermediate_name = 'base_to_tip_x_rotation'
            joint_x = 'base_to_tip_x'
            joint_y = 'base_to_tip_y'
            self._add_revolute_joint(model, joint_x, 'base_link', intermediate_name, '1 0 0', f'0 0 {self.L1/2} 0 0 0')
            self._add_intermediate_link(model,
                                        name=intermediate_name,
                                        parent_name=joint_x,
                                        pose=f'0 0 0 0 0 0')
            self._add_revolute_joint(model, joint_y, intermediate_name, 'tip_link', '0 1 0')
            joint_names.extend([joint_x, joint_y])
        
        return joint_names
    
    def _add_joint_state_publisher(self, model, joint_names):
        """Add joint state publisher plugin"""
        #gazebo = ET.SubElement(model, 'gazebo')
        plugin = ET.SubElement(model, 'plugin',
                              filename='gz-sim-joint-state-publisher-system',
                              name='gz::sim::systems::JointStatePublisher')
        
        #for joint_name in joint_names:
        #    ET.SubElement(plugin, 'joint_name').text = joint_name
    
    def _save_sdf_file(self, sdf_root, filename):
        """Save SDF file with pretty formatting"""
        # Pretty print XML
        rough_string = ET.tostring(sdf_root, 'unicode')
        reparsed = minidom.parseString(rough_string)
        pretty_xml = reparsed.toprettyxml(indent='  ')
        
        # Remove empty lines
        lines = [line for line in pretty_xml.split('\n') if line.strip()]
        final_xml = '\n'.join(lines)
        
        # Save SDF file
        sdf_path = os.path.join(self.sdf_dir, filename)
        with open(sdf_path, 'w') as f:
            f.write(final_xml)
        
        return sdf_path
    
    def generate_launch_file(self, xacro_filename):
        """Generate ROS2 launch file for the catheter"""
        package_name = os.path.basename(self.package_dir)
        launch_filename = f"{package_name}_launch.py"
        sdf_filename = xacro_filename.replace('.xacro', '.sdf')

        if self.anatomy_stl:
            anatomy_basename = os.path.basename(self.anatomy_stl)
            gazebo_block = f'''
    def _launch_gazebo(context):
        """Resolve installed anatomy mesh path and write a temp world SDF."""
        pkg = get_package_share_directory('{package_name}')
        anatomy_mesh = os.path.join(pkg, 'meshes', '{anatomy_basename}')
        world_tmpl = os.path.join(pkg, 'worlds', 'custom_world.sdf')
        with open(world_tmpl, 'r') as f:
            content = f.read().replace('__ANATOMY_MESH_PATH__', anatomy_mesh)
        import tempfile
        tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False)
        tmp.write(content)
        tmp.close()
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'),
                             'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={{'gz_args': f'-r {{tmp.name}}'}}.items(),
        )]

    gazebo_action = OpaqueFunction(function=_launch_gazebo)'''
            extra_imports = '\nfrom launch.actions import IncludeLaunchDescription, OpaqueFunction'
        else:
            gazebo_block = f'''
    custom_world_file = os.path.join(pkg_share, 'worlds', 'custom_world.sdf')
    gazebo_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={{'gz_args': f'-r {{custom_world_file}}'}}.items(),
    )'''
            extra_imports = '\nfrom launch.actions import IncludeLaunchDescription'

        launch_content = f'''import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription{extra_imports}
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Package Directories
    pkg_share = get_package_share_directory('{package_name}')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Parse robot description from xacro
    robot_description_file = os.path.join(pkg_share, 'urdf', '{xacro_filename}')
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {{'robot_description': robot_description_config.toxml()}}

    # Robot state publisher (publishes TF for catheter and anatomy)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Joint state publisher (optional - uncomment for GUI control)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[{{'name': '{package_name}_model',
                    'file': os.path.join(pkg_share, 'sdf', '{sdf_filename}')}}],
        output='screen',
    )

    # Gz - ROS Bridge for joint states and pose
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/{package_name}_model/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
            '/world/catheter_world/model/{package_name}_model/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        remappings=[
            ('/world/catheter_world/model/{package_name}_model/joint_state', '/joint_states'),
        ],
        output='screen'
    )
{gazebo_block}

    return LaunchDescription(
        [
            # Launch Gazebo (with anatomy in world if configured)
            gazebo_action,
            # Spawn the robot model
            spawn,
            # Bridge Gazebo and ROS2
            bridge,
            # Publish robot state (includes anatomy TF when configured)
            robot_state_publisher,
            # Uncomment the next line to enable joint state publisher GUI
            # joint_state_publisher_gui,
            # Launch RViz
            rviz,
        ]
    )
'''

        launch_path = os.path.join(self.launch_dir, launch_filename)
        with open(launch_path, 'w') as f:
            f.write(launch_content)

        return launch_path
    
    def generate_custom_world(self):
        """Generate custom world SDF file for Gazebo"""
        anatomy_block = ''
        if self.anatomy_stl:
            ax, ay, az = self.anatomy_xyz
            ar, ap, ayaw = self.anatomy_rpy
            sc = self.anatomy_scale
            anatomy_block = f'''
    <!-- Anatomy model: mesh path (__ANATOMY_MESH_PATH__) resolved at launch time -->
    <model name="anatomy">
      <static>true</static>
      <pose>{ax} {ay} {az} {ar} {ap} {ayaw}</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>file://__ANATOMY_MESH_PATH__</uri>
              <scale>{sc} {sc} {sc}</scale>
            </mesh>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 0.5</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>'''

        world_content = f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="catheter_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane for reference -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
{anatomy_block}
  </world>
</sdf>'''
        
        # Save world file
        world_path = os.path.join(self.worlds_dir, "custom_world.sdf")
        with open(world_path, 'w') as f:
            f.write(world_content)
        
        return world_path
    
    def create_link_inertia(self, mass, length, radius):
        """Calculate inertia tensor for cylindrical link"""
        ixx = iyy = (1/12) * mass * (3 * radius**2 + length**2)
        izz = 0.5 * mass * radius**2
        return {'ixx': ixx, 'iyy': iyy, 'izz': izz, 'ixy': 0, 'ixz': 0, 'iyz': 0}
    
    def add_link(self, root, name, mass, length, radius, xyz_origin="0 0 0"):
        """Add a link element to the URDF"""
        link = ET.SubElement(root, 'link', name=name)
        
        # Visual
        visual = ET.SubElement(link, 'visual')
        visual_origin = ET.SubElement(visual, 'origin', xyz=xyz_origin, rpy="0 0 0")
        visual_geometry = ET.SubElement(visual, 'geometry')
        ET.SubElement(visual_geometry, 'cylinder', radius=str(radius), length=str(length))
        visual_material = ET.SubElement(visual, 'material', name="catheter_material")
        ET.SubElement(visual_material, 'color', rgba="0.8 0.8 0.8 1.0")
        
        # Collision
        collision = ET.SubElement(link, 'collision')
        collision_origin = ET.SubElement(collision, 'origin', xyz=xyz_origin, rpy="0 0 0")
        collision_geometry = ET.SubElement(collision, 'geometry')
        ET.SubElement(collision_geometry, 'cylinder', radius=str(radius), length=str(length))
        
        # Inertial
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'origin', xyz=xyz_origin, rpy="0 0 0")
        ET.SubElement(inertial, 'mass', value=str(mass))
        inertia_values = self.create_link_inertia(mass, length, radius)
        ET.SubElement(inertial, 'inertia', **{k: str(v) for k, v in inertia_values.items()})
    
    def add_joint(self, root, name, parent, child, xyz_origin, joint_type="revolute", axis="1 0 0"):
        """Add a joint element to the URDF"""
        joint = ET.SubElement(root, 'joint', name=name, type=joint_type)
        ET.SubElement(joint, 'parent', link=parent)
        ET.SubElement(joint, 'child', link=child)
        ET.SubElement(joint, 'origin', xyz=xyz_origin, rpy="0 0 0")
        ET.SubElement(joint, 'axis', xyz=axis)
        
        # Add limits and dynamics for revolute joints
        if joint_type == "revolute":
            ET.SubElement(joint, 'limit', lower=str(-math.pi/2), upper=str(math.pi/2), 
                         effort="100", velocity="10")
            ET.SubElement(joint, 'dynamics', damping=f'{self.damping}', friction=f'{self.friction}')
    
    def add_spring_plugin(self, root, joint_name, spring_constant):
        """Add Gazebo plugin for spring behavior"""
        gazebo = ET.SubElement(root, 'gazebo')
        plugin = ET.SubElement(gazebo, 'plugin', name=f"{joint_name}_spring", 
                              filename="libgazebo_ros_joint_spring.so")
        ET.SubElement(plugin, 'joint_name').text = joint_name
        ET.SubElement(plugin, 'spring_constant').text = str(spring_constant)
        ET.SubElement(plugin, 'reference_position').text = "0.0"
    
    def generate_xacro(self):
        """Generate the complete Xacro"""
        robot = ET.Element('robot', name="flexible_catheter")
        robot.set('xmlns:xacro', 'http://www.ros.org/wiki/xacro')
        
        # Generate STL files for different link types
        tip_stl = self.generate_cylinder_stl(self.radius, self.L3, "tip_link.stl")
        base_stl = self.generate_cylinder_stl(self.radius, self.L1, "base_link.stl")
        bending_stl = self.generate_cylinder_stl(self.radius, self.bending_link_length, "bending_link.stl")
        
        # Add xacro properties
        self.add_xacro_properties(robot)
        
        # Add macros
        self.add_catheter_link_macro(robot)
        self.add_universal_joint_macro(robot)

        # World virtual link (root frame, anchors both catheter and anatomy)
        ET.SubElement(robot, 'link', name='world')

        # Use macros to create links
        # Link frame origin at joint, geometry extends from origin
        # Base link is now the root/parent
        package_name = os.path.basename(self.package_dir)
        ET.SubElement(robot, 'xacro:catheter_link',
                     name='base_link', mass='${base_mass}', length='${base_length}',
                     radius='${catheter_radius}', xyz_origin=f'0 0 {self.L1/2}',
                     mesh_file=f'package://{package_name}/meshes/base_link.stl')
        
        for i in range(self.bending_links):
            ET.SubElement(robot, 'xacro:catheter_link',
                         name=f'bending_link_{i+1}', mass='${bending_mass_per_link}',
                         length='${bending_link_length}', radius='${catheter_radius}',
                         xyz_origin=f'0 0 {self.bending_link_length/2}',
                         mesh_file=f'package://{package_name}/meshes/bending_link.stl')
        
        ET.SubElement(robot, 'xacro:catheter_link',
                     name='tip_link', mass='${tip_mass}', length='${tip_length}',
                     radius='${catheter_radius}', xyz_origin=f'0 0 {self.L3/2}',
                     mesh_file=f'package://{package_name}/meshes/tip_link.stl')


        # Use macros to create joints with same naming as SDF
        # Position joints at the end of each parent link (now base to tip)
        if self.bending_links > 0:
            # Base to first bending link
            ET.SubElement(robot, 'xacro:universal_joint',
                         name='base_to_bending_1', parent='base_link', child='bending_link_1',
                         xyz_origin=f'0 0 {self.L1}', spring_k='${spring_constant}')
            
            # Intermediate bending link joints
            for i in range(1, self.bending_links):
                ET.SubElement(robot, 'xacro:universal_joint',
                             name=f'bending_{i}_to_{i+1}', parent=f'bending_link_{i}',
                             child=f'bending_link_{i+1}', xyz_origin=f'0 0 {self.bending_link_length}',
                             spring_k='${spring_constant}')
            
            # Last bending link to tip
            ET.SubElement(robot, 'xacro:universal_joint',
                         name=f'bending_{self.bending_links}_to_tip',
                         parent=f'bending_link_{self.bending_links}', child='tip_link',
                         xyz_origin=f'0 0 {self.bending_link_length}', spring_k='${spring_constant}')
        else:
            # Direct base to tip if no bending links
            ET.SubElement(robot, 'xacro:universal_joint',
                         name='base_to_tip', parent='base_link', child='tip_link',
                         xyz_origin=f'0 0 {self.L1}', spring_k='${spring_constant}')

        # Fix base_link to world (world becomes the URDF root, matching the SDF)
        j_world = ET.SubElement(robot, 'joint', name='world_to_base', type='fixed')
        ET.SubElement(j_world, 'parent', link='world')
        ET.SubElement(j_world, 'child', link='base_link')
        ET.SubElement(j_world, 'origin', xyz='0 0 0', rpy='0 0 0')

        # Anatomy link — fixed to world frame, visible in both RViz and Gazebo
        if self.anatomy_stl:
            anatomy_basename = os.path.basename(self.anatomy_stl)
            anatomy_lk = ET.SubElement(robot, 'link', name='anatomy_link')
            vis = ET.SubElement(anatomy_lk, 'visual')
            ET.SubElement(vis, 'origin', xyz='0 0 0', rpy='0 0 0')
            vis_geom = ET.SubElement(vis, 'geometry')
            mesh_el = ET.SubElement(vis_geom, 'mesh')
            mesh_el.set('filename', f'package://{package_name}/meshes/{anatomy_basename}')
            mesh_el.set('scale',
                        f'{self.anatomy_scale} {self.anatomy_scale} {self.anatomy_scale}')
            anat_mat = ET.SubElement(vis, 'material', name='anatomy_material')
            ET.SubElement(anat_mat, 'color', rgba='0.8 0.2 0.2 0.5')

            j_anat = ET.SubElement(robot, 'joint', name='world_to_anatomy', type='fixed')
            ET.SubElement(j_anat, 'parent', link='world')
            ET.SubElement(j_anat, 'child', link='anatomy_link')
            ET.SubElement(j_anat, 'origin',
                          xyz=(f'{self.anatomy_xyz[0]} {self.anatomy_xyz[1]}'
                               f' {self.anatomy_xyz[2]}'),
                          rpy=(f'{self.anatomy_rpy[0]} {self.anatomy_rpy[1]}'
                               f' {self.anatomy_rpy[2]}'))

        return robot
    
    def add_xacro_properties(self, root):
        """Add xacro properties for parameters"""
        ET.SubElement(root, 'xacro:property', name='catheter_links', value=str(self.N))
        ET.SubElement(root, 'xacro:property', name='catheter_diameter', value=str(self.D))
        ET.SubElement(root, 'xacro:property', name='catheter_radius', value=str(self.radius))
        ET.SubElement(root, 'xacro:property', name='tip_length', value=str(self.L3))
        ET.SubElement(root, 'xacro:property', name='bending_length', value=str(self.L2))
        ET.SubElement(root, 'xacro:property', name='base_length', value=str(self.L1))
        ET.SubElement(root, 'xacro:property', name='spring_constant', value=str(self.K))
        ET.SubElement(root, 'xacro:property', name='total_mass', value=str(self.M))
        ET.SubElement(root, 'xacro:property', name='bending_links', value=str(self.bending_links))
        ET.SubElement(root, 'xacro:property', name='bending_link_length', value=str(self.bending_link_length))
        ET.SubElement(root, 'xacro:property', name='tip_mass', value=str(self.tip_mass))
        ET.SubElement(root, 'xacro:property', name='base_mass', value=str(self.base_mass))
        ET.SubElement(root, 'xacro:property', name='bending_mass_per_link', value=str(self.bending_mass_per_link))
        
        # Add material definition
        material = ET.SubElement(root, 'material', name='catheter_material')
        ET.SubElement(material, 'color', rgba='0.8 0.8 0.8 1.0')
    
    def add_catheter_link_macro(self, root):
        """Add xacro macro for catheter links"""
        macro = ET.SubElement(root, 'xacro:macro', name='catheter_link')
        macro.set('params', 'name mass length radius xyz_origin mesh_file')
        
        link = ET.SubElement(macro, 'link', name='${name}')
        
        # Visual
        visual = ET.SubElement(link, 'visual')
        ET.SubElement(visual, 'origin', xyz='${xyz_origin}', rpy='0 0 0')
        visual_geometry = ET.SubElement(visual, 'geometry')
        ET.SubElement(visual_geometry, 'mesh', filename='${mesh_file}')
        ET.SubElement(visual, 'material', name='catheter_material')
        
        # Collision
        collision = ET.SubElement(link, 'collision')
        ET.SubElement(collision, 'origin', xyz='${xyz_origin}', rpy='0 0 0')
        collision_geometry = ET.SubElement(collision, 'geometry')
        ET.SubElement(collision_geometry, 'mesh', filename='${mesh_file}')
        
        # Inertial
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'origin', xyz='${xyz_origin}', rpy='0 0 0')
        ET.SubElement(inertial, 'mass', value='${mass}')
        
        # Calculate inertia using xacro expressions
        ixx_iyy = '${(1/12) * mass * (3 * radius * radius + length * length)}'
        izz = '${0.5 * mass * radius * radius}'
        
        ET.SubElement(inertial, 'inertia', 
                     ixx=ixx_iyy, iyy=ixx_iyy, izz=izz,
                     ixy='0', ixz='0', iyz='0')
    
    def add_universal_joint_macro(self, root):
        """Add xacro macro for universal joints with springs - matching SDF joint names"""
        macro = ET.SubElement(root, 'xacro:macro', name='universal_joint')
        macro.set('params', 'name parent child xyz_origin spring_k')
        
        # Intermediate link for universal joint
        intermediate_link = ET.SubElement(macro, 'link', name='${name}_x_rotation')
        inertial = ET.SubElement(intermediate_link, 'inertial')
        ET.SubElement(inertial, 'mass', value='0.001')
        ET.SubElement(inertial, 'inertia', ixx='1e-6', iyy='1e-6', izz='1e-6',
                     ixy='0', ixz='0', iyz='0')
        
        # X-axis joint (parent -> intermediate)
        joint_x = ET.SubElement(macro, 'joint', name='${name}_x', type='revolute')
        ET.SubElement(joint_x, 'parent', link='${parent}')
        ET.SubElement(joint_x, 'child', link='${name}_x_rotation')
        ET.SubElement(joint_x, 'origin', xyz='${xyz_origin}', rpy='0 0 0')
        ET.SubElement(joint_x, 'axis', xyz='1 0 0')
        ET.SubElement(joint_x, 'limit', lower=str(-math.pi/2), upper=str(math.pi/2), 
                     effort='100', velocity='10')
        ET.SubElement(joint_x, 'dynamics', damping=f'{self.damping}', friction=f'{self.friction}')
        
        # Y-axis joint (intermediate -> child)
        joint_y = ET.SubElement(macro, 'joint', name='${name}_y', type='revolute')
        ET.SubElement(joint_y, 'parent', link='${name}_x_rotation')
        ET.SubElement(joint_y, 'child', link='${child}')
        ET.SubElement(joint_y, 'origin', xyz='0 0 0', rpy='0 0 0')
        ET.SubElement(joint_y, 'axis', xyz='0 1 0')
        ET.SubElement(joint_y, 'limit', lower=str(-math.pi/2), upper=str(math.pi/2), 
                     effort='100', velocity='10')
        ET.SubElement(joint_y, 'dynamics', damping=f'{self.damping}', friction=f'{self.friction}')
        
        # Spring plugins for Gazebo
        gazebo_x = ET.SubElement(macro, 'gazebo')
        plugin_x = ET.SubElement(gazebo_x, 'plugin', name='${name}_x_spring',
                                filename='libgazebo_ros_joint_spring.so')
        joint_name_x = ET.SubElement(plugin_x, 'joint_name')
        joint_name_x.text = '${name}_x'
        spring_const_x = ET.SubElement(plugin_x, 'spring_constant')
        spring_const_x.text = '${spring_k}'
        ref_pos_x = ET.SubElement(plugin_x, 'reference_position')
        ref_pos_x.text = '0.0'
        
        gazebo_y = ET.SubElement(macro, 'gazebo')
        plugin_y = ET.SubElement(gazebo_y, 'plugin', name='${name}_y_spring',
                                filename='libgazebo_ros_joint_spring.so')
        joint_name_y = ET.SubElement(plugin_y, 'joint_name')
        joint_name_y.text = '${name}_y'
        spring_const_y = ET.SubElement(plugin_y, 'spring_constant')
        spring_const_y.text = '${spring_k}'
        ref_pos_y = ET.SubElement(plugin_y, 'reference_position')
        ref_pos_y.text = '0.0'
    
    def save_xacro(self, filename=None):
        """Save the Xacro to file in urdf directory and generate ROS2 package files"""
        robot = self.generate_xacro()
        
        # Use package name as default filename if none provided
        if filename is None:
            package_name = os.path.basename(self.package_dir)
            filename = f"{package_name}.xacro"
        
        # Pretty print XML
        rough_string = ET.tostring(robot, 'unicode')
        reparsed = minidom.parseString(rough_string)
        pretty_xml = reparsed.toprettyxml(indent="  ")
        
        # Remove empty lines
        lines = [line for line in pretty_xml.split('\n') if line.strip()]
        final_xml = '\n'.join(lines)
        
        # Save xacro file in urdf directory
        xacro_path = os.path.join(self.urdf_dir, filename)
        with open(xacro_path, 'w') as f:
            f.write(final_xml)
        
        # Generate ROS2 package files
        package_xml_path = self.generate_package_xml()
        cmake_path = self.generate_cmakelists_txt()

        # Copy anatomy STL into the package meshes directory
        if self.anatomy_stl:
            import shutil
            anatomy_basename = os.path.basename(self.anatomy_stl)
            shutil.copy2(self.anatomy_stl,
                         os.path.join(self.meshes_dir, anatomy_basename))

        # Generate SDF file for Gazebo
        sdf_path = self.generate_sdf(filename)

        # Generate launch file
        launch_path = self.generate_launch_file(filename)

        # Generate custom world file
        world_path = self.generate_custom_world()

        print(f"ROS2 package created: {self.package_dir}/")
        print(f"  Package files:")
        print(f"    - {os.path.relpath(package_xml_path, self.package_dir)}")
        print(f"    - {os.path.relpath(cmake_path, self.package_dir)}")
        print(f"  Xacro saved to: {os.path.relpath(xacro_path, self.package_dir)}")
        print(f"  SDF saved to: {os.path.relpath(sdf_path, self.package_dir)} (base fixed to world)")
        print(f"  World file saved to: {os.path.relpath(world_path, self.package_dir)}")
        print(f"  Launch file saved to: {os.path.relpath(launch_path, self.package_dir)}")
        print(f"  STL files in: {os.path.relpath(self.meshes_dir, self.package_dir)}/")
        print(f"    - tip_link.stl, bending_link.stl, base_link.stl")
        if self.anatomy_stl:
            print(f"    - {os.path.basename(self.anatomy_stl)}  (anatomy)")


def main():
    parser = argparse.ArgumentParser(description="Generate Xacro for flexible catheter")
    parser.add_argument("--N", type=int, default=5, help="Number of links")
    parser.add_argument("--D", type=float, default=0.002, help="Outer diameter (m)")
    parser.add_argument("--L1", type=float, default=0.05, help="Base link length (m)")
    parser.add_argument("--L2", type=float, default=0.1, help="Bending section length (m)")
    parser.add_argument("--L3", type=float, default=0.01, help="Tip link length (m)")
    parser.add_argument("--K", type=float, default=0.1, help="Spring constant (Nm/rad)")
    parser.add_argument("--Kd", type=float, default=0.1, help="Damping constant")
    parser.add_argument("--Kf", type=float, default=0.01, help="Friction")
    parser.add_argument("--M", type=float, default=0.01, help="Total mass (kg)")

    parser.add_argument("--output", type=str, default="catheter_package",
                       help="Package directory name")
    parser.add_argument("--xacro-name", type=str, default=None,
                       help="Xacro filename to save in urdf/ directory (default: package_name.xacro)")

    # Anatomy model arguments (makes anatomy visible in both RViz and Gazebo)
    parser.add_argument("--anatomy-stl", type=str, default=None,
                       help="Path to anatomy STL file")
    parser.add_argument("--anatomy-x",     type=float, default=0.0,   help="Anatomy X position (m)")
    parser.add_argument("--anatomy-y",     type=float, default=0.0,   help="Anatomy Y position (m)")
    parser.add_argument("--anatomy-z",     type=float, default=0.0,   help="Anatomy Z position (m)")
    parser.add_argument("--anatomy-roll",  type=float, default=0.0,   help="Anatomy roll  (rad)")
    parser.add_argument("--anatomy-pitch", type=float, default=0.0,   help="Anatomy pitch (rad)")
    parser.add_argument("--anatomy-yaw",   type=float, default=0.0,   help="Anatomy yaw   (rad)")
    parser.add_argument("--anatomy-scale", type=float, default=0.001,
                       help="Anatomy mesh scale factor (default 0.001 converts mm→m)")

    args = parser.parse_args()

    try:
        generator = CatheterXacroGenerator(
            args.N, args.D, args.L1, args.L2, args.L3,
            args.K, args.Kd, args.Kf, args.M, args.output,
            anatomy_stl=args.anatomy_stl,
            anatomy_xyz=(args.anatomy_x, args.anatomy_y, args.anatomy_z),
            anatomy_rpy=(args.anatomy_roll, args.anatomy_pitch, args.anatomy_yaw),
            anatomy_scale=args.anatomy_scale)
        generator.save_xacro(args.xacro_name)
        
        print(f"Generated catheter with parameters:")
        print(f"  Links: {args.N}")
        print(f"  Diameter: {args.D} m")
        print(f"  Base length: {args.L1} m")
        print(f"  Bending section length: {args.L2} m")
        print(f"  Tip length: {args.L3} m")
        print(f"  Spring constant: {args.K} Nm/rad")
        print(f"  Damping constant: {args.Kd}")
        print(f"  Friction constant: {args.Kf}")
        print(f"  Total mass: {args.M} kg")
        
    except ValueError as e:
        print(f"Error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())
