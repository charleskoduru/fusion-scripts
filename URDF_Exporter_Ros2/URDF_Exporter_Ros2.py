
# Author: Updated for Fusion 360 and ROS 2
# Description: Export Fusion 360 robot assembly to a ROS 2 description package

import adsk.core
import adsk.fusion
import traceback
import os


def run(context):
    ui = None
    title = "Fusion2URDF ROS2"

    try:
        # Initialize Fusion 360
        app = adsk.core.Application.get()
        ui = app.userInterface

        ui.messageBox(
            "ROS 2 URDF Exporter started.\nLoading exporter modules...",
            title
        )

        # Import modules inside run() so import errors are displayed
        try:
            from .utils import utils
            from .core import Link, Joint, Write

        except Exception:
            ui.messageBox(
                "Failed to import exporter modules:\n\n"
                + traceback.format_exc(),
                title
            )
            return

        # Get active Fusion design
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)

        if not design:
            ui.messageBox(
                "No active Fusion 360 design found.",
                title
            )
            return

        root = design.rootComponent
        components = design.allComponents

        # Robot and ROS 2 package names
        robot_name = root.name.split()[0]
        package_name = robot_name + "_description"

        # Select output folder
        save_dir = utils.file_dialog(ui)

        if not save_dir:
            ui.messageBox("Export cancelled.", title)
            return

        save_dir = os.path.join(save_dir, package_name)

        os.makedirs(save_dir, exist_ok=True)

        # Package template directory
        package_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "package"
        )

        if not os.path.isdir(package_dir):
            raise FileNotFoundError(
                "Package template folder not found:\n"
                + package_dir
            )

        # Generate joint information
        success_msg = "Successfully create URDF file"
        msg = success_msg

        joints_dict, msg = Joint.make_joints_dict(
            root, msg
        )

        if msg != success_msg:
            ui.messageBox(msg, title)
            return

        # Generate link inertial information
        inertial_dict, msg = Link.make_inertial_dict(
            root, msg
        )

        if msg != success_msg:
            ui.messageBox(msg, title)
            return

        if "base_link" not in inertial_dict:
            ui.messageBox(
                "No base_link found.\n\n"
                "Rename your robot base component to base_link.",
                title
            )
            return

        # Link positions
        links_xyz_dict = {}

        # Generate URDF and Xacro files
        Write.write_urdf(
            joints_dict,
            links_xyz_dict,
            inertial_dict,
            package_name,
            robot_name,
            save_dir
        )

        Write.write_materials_xacro(
            joints_dict,
            links_xyz_dict,
            inertial_dict,
            package_name,
            robot_name,
            save_dir
        )

        Write.write_transmissions_xacro(
            joints_dict,
            links_xyz_dict,
            inertial_dict,
            package_name,
            robot_name,
            save_dir
        )

        Write.write_gazebo_xacro(
            joints_dict,
            links_xyz_dict,
            inertial_dict,
            package_name,
            robot_name,
            save_dir
        )

        Write.write_display_launch(
            package_name,
            robot_name,
            save_dir
        )

        Write.write_gazebo_launch(
            package_name,
            robot_name,
            save_dir
        )

        # Generate ROS 2 package files
        utils.create_package(
            package_name,
            save_dir,
            package_dir
        )

        utils.update_setup_py(
            save_dir,
            package_name
        )

        utils.update_setup_cfg(
            save_dir,
            package_name
        )

        utils.update_package_xml(
            save_dir,
            package_name
        )

        # Export component meshes
        ui.messageBox(
            "URDF generated. Preparing STL meshes...",
            title
        )

        utils.copy_occs(root)

        utils.export_stl(
            design,
            save_dir,
            components
        )

        ui.messageBox(
            "ROS 2 URDF export completed!\n\n"
            "Output folder:\n" + save_dir,
            title
        )

    except Exception:
        if ui:
            ui.messageBox(
                "ROS 2 URDF EXPORT FAILED:\n\n"
                + traceback.format_exc(),
                title
            )


def stop(context):
    pass
