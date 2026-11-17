# -*- coding: utf-8 -*-
"""
Created on Sun May 12 19:15:34 2019

@author: syuntoku
"""

import adsk, adsk.core, adsk.fusion
import os.path, re
from xml.etree import ElementTree
from xml.dom import minidom
from shutil import copytree
import fileinput
import sys

def export_stl(design, save_dir, root):
    """Export meshes for the same top-level occurrences used in the URDF.

    Exporting an Occurrence directly is supported by Fusion's STL exporter.
    It avoids the old approach of copying geometry into temporary components
    and renaming the source components (which fails for the root component).
    """
    export_mgr = design.exportManager
    meshes_dir = os.path.join(save_dir, 'meshes')
    os.makedirs(meshes_dir, exist_ok=True)

    for occ in root.occurrences:
        # Match the selection and naming logic used in core/Link.py.
        if occ.bRepBodies.count == 0:
            continue
        if occ.component.name == 'base_link':
            mesh_name = 'base_link'
        else:
            mesh_name = re.sub('[ :()]', '_', occ.name)

        file_name = os.path.join(meshes_dir, mesh_name)
        options = export_mgr.createSTLExportOptions(occ, file_name)
        options.sendToPrintUtility = False
        options.isBinaryFormat = True
        options.isOneFilePerBody = False
        options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow

        if not export_mgr.execute(options):
            raise RuntimeError('Failed to export STL for occurrence: ' + occ.name)


def file_dialog(ui):
    """
    display the dialog to save the file
    """
    # Set styles of folder dialog.
    folderDlg = ui.createFolderDialog()
    folderDlg.title = 'Fusion Folder Dialog'

    # Show folder dialog
    dlgResult = folderDlg.showDialog()
    if dlgResult == adsk.core.DialogResults.DialogOK:
        return folderDlg.folder
    return False


def origin2center_of_mass(inertia, center_of_mass, mass):
    """
    convert the moment of the inertia about the world coordinate into
    that about center of mass coordinate


    Parameters
    ----------
    moment of inertia about the world coordinate:  [xx, yy, zz, xy, yz, xz]
    center_of_mass: [x, y, z]


    Returns
    ----------
    moment of inertia about center of mass : [xx, yy, zz, xy, yz, xz]
    """
    x = center_of_mass[0]
    y = center_of_mass[1]
    z = center_of_mass[2]
    translation_matrix = [y**2+z**2, x**2+z**2, x**2+y**2,
                         -x*y, -y*z, -x*z]
    return [ round(i - mass*t, 6) for i, t in zip(inertia, translation_matrix)]


def prettify(elem):
    """
    Return a pretty-printed XML string for the Element.
    Parameters
    ----------
    elem : xml.etree.ElementTree.Element


    Returns
    ----------
    pretified xml : str
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

def create_package(package_name, save_dir, package_dir):
    try: os.mkdir(save_dir + '/launch')
    except: pass

    try: os.mkdir(save_dir + '/urdf')
    except: pass

    try: os.mkdir(save_dir + '/config')
    except: pass

    try: os.mkdir(save_dir + '/' +package_name)
    except: pass
    with open(os.path.join(save_dir, package_name, '__init__.py'), 'w'):
        pass

    try: os.mkdir(save_dir + '/resource')
    except: pass
    with open(os.path.join(save_dir, 'resource', package_name), 'w'):
        pass

    try: os.mkdir(save_dir + '/test')
    except: pass

    copytree(package_dir, save_dir, dirs_exist_ok=True)
    
def update_setup_py(save_dir, package_name):
    file_name = save_dir + '/setup.py'

    for line in fileinput.input(file_name, inplace=True):
        if "package_name = 'fusion2urdf_ros2'" in line:
            sys.stdout.write("package_name = '" + package_name + "'\n")
        else:
            sys.stdout.write(line)

def update_setup_cfg(save_dir, package_name):
    file_name = save_dir + '/setup.cfg'

    for line in fileinput.input(file_name, inplace=True):
        if "script-dir" in line:
            sys.stdout.write("script-dir=$base/lib/" + package_name + "\n")
        elif "install-scripts" in line:
            sys.stdout.write("install-scripts=$base/lib/" + package_name + "\n")
        else:
            sys.stdout.write(line)

def update_package_xml(save_dir, package_name):
    file_name = save_dir + '/package.xml'

    for line in fileinput.input(file_name, inplace=True):
        if '<name>' in line:
            sys.stdout.write("<name>" + package_name + "</name>\n")
        elif '<description>' in line:
            sys.stdout.write("<description>The " + package_name + " package</description>\n")
        else:
            sys.stdout.write(line)
