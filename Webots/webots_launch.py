import sys
import os
import os.path
from pathlib import Path
import logging
import xml.dom.minidom
from urdf2webots.importer import convertUrdfFile
from fractions import Fraction

PACKAGE_NAME = 'os_augustus'
DEFAULT_WORLD = 'default.wbt'

def IsDocker():
    cgroup = Path("/proc/self/cgroup")
    return Path('/.dockerenv').is_file() or cgroup.is_file() and cgroup.read_text().find("docker") > -1


def main(args=None):

    worldfile = None
    is_docker = IsDocker()

    if (is_docker):
        logging.info('Launching with Docker settings...')
        base_path = '/Webots/'
        if os.environ['WORLD'] and os.path.isfile(base_path + 'worlds/' + os.environ['WORLD']):
            worldfile = os.environ['WORLD']
        else:
            logging().error('The given worldfile does not exist, using default.wbt instead')
            worldfile = DEFAULT_WORLD
        if 'ROBOT' in os.environ:
            urdf_filename = os.environ['ROBOT']
            urdf_filepath= base_path + 'resource/' + urdf_filename + '/' + urdf_filename + '.urdf'
            base_urdf_filepath = '/Webots/resource/robot.urdf'
    else:
        logging.info('Launching...')
        base_path = ''
        if(len(sys.argv)>1 and os.path.isfile(os.path.dirname(os.path.realpath(__file__)) + '/worlds/' + sys.argv[1])):
            worldfile = sys.argv[1]
        else:
            logging.info('The given worldfile does not exist, using default.wbt instead')
            worldfile = DEFAULT_WORLD
        if(len(sys.argv) > 2):
            urdf_filename = sys.argv[2]
            urdf_filepath= base_path + 'resource/' + urdf_filename + '/' + urdf_filename + '.urdf'
            base_urdf_filepath = base_path + 'resource/robot.urdf'


    '''
    For general use:
        Make sure that the joints in the urdf are named 'Joint{index of joint}' , for example '<joint name="Joint3" type="revolute">'
        Make sure that the ground/base/world link in the urdf file is named 'base_link'
    For rendering:
        STL-s should be placed into the /Webots/resource/{name of your urdf file}/mesh folder
        Make sure that inside your urdf, the path of your stl files are like the following: <mesh filename="{name of your urdf file}/mesh/link6.stl"/>
    '''

    # Generate and properly alter proto and ros2 control urdf file and place them in the correct folders if urdf parameter is given

    # Check if file exists
    if os.path.isfile(urdf_filepath):
        logging.info('Generating robot ' + urdf_filename)

        os.system('cp ' + urdf_filepath + ' ' + base_urdf_filepath)
        DOM = xml.dom.minidom
        doc = DOM.parse(open(base_urdf_filepath))
        robot_tag = doc.getElementsByTagName('robot')[0]
        robot_tag.setAttribute('name', urdf_filename)
        with open(base_urdf_filepath, 'w') as file:
            file.write(doc.toprettyxml())
        

        # Generate the proto file, set external controller and move it to the proper folder
        convertUrdfFile(input=base_urdf_filepath)
        with open( urdf_filename + '.proto', 'r') as file:
            filedata = file.read()
            filedata = filedata.replace('void', '<extern>')  # the extern controller is in controller_and_interfaces
            filedata = filedata.replace(
                'supervisor      FALSE', 'supervisor      TRUE ')
            filedata = filedata.replace(
                'synchronization TRUE', 'synchronization FALSE')
        with open(urdf_filename + '.proto', 'w') as file:
            file.write(filedata)
        os.system('mv ' + urdf_filename + '.proto ' + base_path + 'protos/' + urdf_filename + '.proto')

        # Move meshes to the proper folder
        os.system('rm -rf ' + base_path + 'resource/mesh')
        os.system('rm -rf ' + base_path + 'resource/robot.urdf')
        mesh_folderpath = base_path + 'resource/' + urdf_filename + '/' + 'mesh'
        proto_resource_folderpath = 'protos/resource/' + urdf_filename
        os.system('mkdir -p ' + proto_resource_folderpath)
        os.system('cp -R ' + mesh_folderpath + ' ' + proto_resource_folderpath )
    else:
        print('Robot ' + urdf_filename + ' does not exist.')
        return

    try:
        logging.info('Launching world: ' + worldfile)
        logging.info('Loading robot: ' + urdf_filename)
        if (is_docker):
            os.system('/etc/init.d/tinyproxy start')
            os.system('xvfb-run --auto-servernum webots --no-rendering --stdout --stderr --minimize --stream --batch /Webots/worlds/' +
                      worldfile + ' & ' + 'python3 /Webots/controller_and_interface/robot_controller.py ' + urdf_filename + '/' + urdf_filename + '.urdf')
        else:
            os.system('webots --stream --batch worlds/' + worldfile + ' & ' +
                      'python3 controller_and_interface/robot_controller.py ' + urdf_filename + '/' + urdf_filename + '.urdf')
    except:
        print(
            'An error has occured while trying to launch webots with the given parameters.')


if __name__ == '__main__':
    main()