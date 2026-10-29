import xml.etree.ElementTree as ET
from xml.dom import minidom


def create_snake_xml(filename="planar_snake_cars_servo.xml", num_segments=11):
    mujoco = ET.Element("mujoco", model="planar_snake_car_servo")

    # Compiler: Angle set to radians
    ET.SubElement(
        mujoco,
        "compiler",
        angle="radian",
        coordinate="local",
        inertiafromgeom="true",
    )
    # High-frequency physics timestep (0.002s = 500 Hz)
    option = ET.SubElement(
        mujoco, "option", integrator="implicitfast", timestep="0.002"
    )
    ET.SubElement(option, "flag", energy="enable", clampctrl="enable")
    ET.SubElement(mujoco, "size", nconmax="1000")

    # Default settings:
    # - solref/solimp: equal to MuJoCo's defaults (kept explicit for clarity)
    # - joint armature/damping/frictionloss here are SERVO values (reflected
    #   gearbox inertia and friction). Wheels and the head's world-frame joints
    #   override them below, because wheel bearings roll nearly freely and the
    #   head joints are only a free-floating base, not real hardware.
    default = ET.SubElement(mujoco, "default")
    ET.SubElement(
        default,
        "geom",
        conaffinity="0",
        condim="3",
        friction="0.6 0.005 0.0001",  # sliding, torsional, rolling (condim 3 uses only sliding)
        margin="0",
        density="28.3",  # gives ~900 g total with 11 segments (compiled mass was 1.041 kg at 32.73)
        solref="0.02 1",
        solimp="0.9 0.95 0.001",
    )
    ET.SubElement(
        default, "joint", armature="0.005", damping="0.05", frictionloss="0.005"
    )  # servo joints; wheel class and head joints override

    default_wheel = ET.SubElement(default, "default", **{"class": "wheel"})
    ET.SubElement(
        default_wheel,
        "joint",
        axis="0 1 0",
        type="hinge",
        armature="1e-5",  # ~ real wheel spin inertia (about 8e-6 kg m^2); was 0.005
        damping="5e-5",      # free-rolling bearing; was 0.1 (=40 N s/m reflected per wheel)
        frictionloss="0",  # was 0.005 (constant 0.1 N drag per wheel)
    )
    ET.SubElement(
        default_wheel,
        "geom",
        type="cylinder",
        fromto="0 0 0  0 0.025 0",
        size="0.05",
        material="geom",
        rgba=".9 .9 .9 1",
        friction="0.6 0.005 0.01992",  # sliding friction 0.6 for now: ratio is ~0.0332
    )

    default_car = ET.SubElement(default, "default", **{"class": "car"})
    ET.SubElement(default_car, "geom", material="geom", rgba="0.8 0.7 .6 1")
    ET.SubElement(
        default_car,
        "joint",
        pos="0 0 0",
        axis="0 0 1",
        type="hinge",
        limited="true",
        range="-1.5708 1.5708",  # +/- 90 degrees in radians
    )

    # Servo Position Actuators:
    # - forcerange="-1.4 1.4": Peak stall torque (1.4 N*m)
    # - kp="0.5", kv="0.05": Position gain + motor speed damping (Back-EMF)
    ET.SubElement(
        default,
        "position",
        ctrllimited="true",
        ctrlrange="-1.5 1.5",
        forcelimited="true",
        forcerange="-1.4 1.4",  # 1.4 N*m stall torque
        kp="30",  # torque saturates past 1.4/30 = 0.047 rad error, so the torque ceiling,
                  # not kp, limits how fast the joints move
        kv="0.8",
        gear="1",
    )

    # Assets
    asset = ET.SubElement(mujoco, "asset")
    ET.SubElement(
        asset,
        "texture",
        builtin="gradient",
        height="100",
        rgb1="1 1 1",
        rgb2="0 0 0",
        type="skybox",
        width="100",
    )
    ET.SubElement(
        asset,
        "texture",
        builtin="flat",
        height="1278",
        mark="cross",
        markrgb="1 1 1",
        name="texgeom",
        random="0.01",
        rgb1="0.8 0.6 0.4",
        rgb2="0.8 0.6 0.4",
        type="cube",
        width="127",
    )
    ET.SubElement(
        asset,
        "texture",
        builtin="checker",
        height="100",
        name="texplane",
        rgb1="0 0 0",
        rgb2="0.8 0.8 0.8",
        type="2d",
        width="100",
    )
    ET.SubElement(
        asset,
        "material",
        name="MatPlane",
        reflectance="0.5",
        shininess="1",
        specular="1",
        texrepeat="80 80",
        texture="texplane",
    )
    ET.SubElement(
        asset, "material", name="geom", texture="texgeom", texuniform="true"
    )
    ET.SubElement(
        asset,
        "material",
        name="MatTarget",
        rgba="1 0 0 1",
        reflectance="0.05",
        shininess="0.1",
        specular="0.1",
    )

    # Visual settings
    visual = ET.SubElement(mujoco, "visual")
    ET.SubElement(visual, "map", znear="0.0625")
    ET.SubElement(visual, "quality", shadowsize="4096")

    # Worldbody
    worldbody = ET.SubElement(mujoco, "worldbody")
    ET.SubElement(
        worldbody,
        "light",
        cutoff="100",
        diffuse="1 1 1",
        dir="-0 0 -1.3",
        directional="true",
        exponent="1",
        pos="0 0 1.3",
        specular=".1 .1 .1",
    )
    ET.SubElement(
        worldbody,
        "geom",
        name="floor",
        size="80 80 10",
        material="MatPlane",
        conaffinity="1",
        condim="3",
        pos="0 0 -0.1",
        friction="0.6 0.005 0.0001",  # contact uses the max of floor and wheel, so match them
        rgba="0.8 0.9 0.8 1",
        type="plane",
    )

    # Target body
    """
    # Uncomment here and line 181 for sphere at origin

    target = ET.SubElement(worldbody, "body", name="target_ball", pos="0 0 0.1")
    ET.SubElement(
        target, "joint", axis="1 0 0", name="target_slider_x", type="slide"
    )
    ET.SubElement(
        target, "joint", axis="0 1 0", name="target_slider_y", type="slide"
    )


    ET.SubElement(
        target,
        "geom",
        name="target_ball_geom",
        type="sphere",
        pos="0 0 0",
        size="0.2",
        material="MatTarget",
    )

    ET.SubElement(
        target,
        "camera",
        name="track_ball",
        mode="trackcom",
        pos="0 -4 6",
        euler="0.5236 0 0",
    )
    ET.SubElement(
        target,
        "camera",
        name="track_ball2",
        mode="targetbodycom",
        pos="0 0 1",
        zaxis="0 0 90",
    )

    """
    
    # Snake Head (car1)
    car1 = ET.SubElement(worldbody, "body", name="car1", pos="0 0 0.0")
    for axis, name in [
        ("1 0 0", "slider_x"),
        ("0 1 0", "slider_y"),
        ("0 0 1", "slider_z"),
    ]:
        ET.SubElement(
            car1, "joint", axis=axis, name=name, type="slide",
            armature="0", damping="0", frictionloss="0",
        )
    for axis, name in [
        ("1 0 0", "hinge_x"),
        ("0 1 0", "hinge_y"),
        ("0 0 1", "hinge_z"),
    ]:
        ET.SubElement(
            car1, "joint", pos="0 0 0", axis=axis, name=name, type="hinge",
            armature="0", damping="0", frictionloss="0",
        )

    # Head wheels & geometries
    wheel_l_0 = ET.SubElement(
        car1, "body", pos="-0.175 0.05 -0.05", **{"childclass": "wheel"}
    )
    ET.SubElement(wheel_l_0, "joint", name="joint_wheel_l_00")
    ET.SubElement(wheel_l_0, "geom")

    ET.SubElement(
        car1,
        "geom",
        name="head",
        pos="-0.175 0 0",
        size="0.125 0.05 0.05",
        type="box",
        material="geom",
        rgba="0.8 0.7 .6 1",
    )
    ET.SubElement(
        car1,
        "geom",
        fromto="-0.30 0 0  -0.05 0 0",
        size="0.05",
        type="capsule",
        rgba="1.0 0.0 0.0 1",
        density="0",     # visual marker stripe only -- was silently doubling segment mass
        contype="0",
        conaffinity="0",
    )

    wheel_r_0 = ET.SubElement(
        car1, "body", pos="-0.175 -0.075 -0.05", **{"childclass": "wheel"}
    )
    ET.SubElement(wheel_r_0, "joint", name="joint_wheel_r_00")
    ET.SubElement(wheel_r_0, "geom")

    ET.SubElement(
        car1,
        "site",
        name="site_head",
        pos="-0.175 0 0.0",
        type="box",
        size=".01 .01 .01",
    )

    # Cameras
    cameras = [
        ("head", "fixed", "0 0 0.025", "90", "1.5708 -1.5708 0", None),
        ("track_snake1", "trackcom", "-1.4 -1.25 2.25", None, "0.5236 0 0", None),
        ("track_snake2", "trackcom", "-1.5 0 3", None, "0 0 0", None),
        ("track_snake3", "trackcom", "0 -4 6", None, "0.5236 0 0", None),
        ("track_snake31", "trackcom", "1.25 -3.75 4.5", None, "0.5236 0 0", None),
        ("track_snake32", "trackcom", "-4.0 0.0 0.75", None, "0 -1.5708 0", None),
        (
            "track_snake33",
            "trackcom",
            "-4.5 0.0 1.75",
            None,
            None,
            "0.0 -1 0.0 0.6 0 1 ",
        ),
        ("track_snake4", "trackcom", "0 0 5", None, "0 0 0", None),
    ]
    for name, mode, pos, fovy, euler, xyaxes in cameras:
        attrs = {"name": name, "mode": mode, "pos": pos}
        if fovy:
            attrs["fovy"] = fovy
        if euler:
            attrs["euler"] = euler
        if xyaxes:
            attrs["xyaxes"] = xyaxes
        ET.SubElement(car1, "camera", **attrs)

    # Programmatically build nested body segments (car2 through carN)
    parent_body = car1
    for i in range(2, num_segments + 1):
        joint_idx_str = f"{i-1:02d}"

        car_body = ET.SubElement(
            parent_body,
            "body",
            name=f"car{i}",
            pos="-0.35 0 0",
            **{"childclass": "car"},
        )
        ET.SubElement(car_body, "joint", name=f"joint{joint_idx_str}")

        wheel_l = ET.SubElement(
            car_body,
            "body",
            pos="-0.175 0.05 -0.05",
            **{"childclass": "wheel"},
        )
        ET.SubElement(wheel_l, "joint", name=f"joint_wheel_l_{joint_idx_str}")
        ET.SubElement(wheel_l, "geom")

        ET.SubElement(
            car_body,
            "geom",
            name=f"car{i}_geom",
            pos="-0.175 0 0",
            size="0.125 0.05 0.05",
            type="box",
            material="geom",
            rgba="0.8 0.7 .6 1",
        )
        ET.SubElement(
            car_body,
            "geom",
            fromto="-0.30 0 0  -0.05 0 0",
            size="0.05",
            type="capsule",
            rgba="1.0 0.0 0.0 1",
            density="0",     # visual marker stripe only -- was silently doubling segment mass
            contype="0",
            conaffinity="0",
        )

        wheel_r = ET.SubElement(
            car_body,
            "body",
            pos="-0.175 -0.075 -0.05",
            **{"childclass": "wheel"},
        )
        ET.SubElement(wheel_r, "joint", name=f"joint_wheel_r_{joint_idx_str}")
        ET.SubElement(wheel_r, "geom")

        parent_body = car_body

    # Actuators
    actuator = ET.SubElement(mujoco, "actuator")
    for i in range(1, num_segments):
        j_str = f"joint{i:02d}"
        ET.SubElement(
            actuator,
            "position",
            name=f"actuator_{j_str}",
            joint=j_str,
            forcerange="-1.4 1.4",
        )

    # Sensors
    sensor = ET.SubElement(mujoco, "sensor")
    for i in range(1, num_segments):
        j_str = f"joint{i:02d}"
        ET.SubElement(
            sensor,
            "actuatorfrc",
            name=f"sensor_actuatorfrc_{j_str}",
            actuator=f"actuator_{j_str}",
        )

    ET.SubElement(
        sensor, "velocimeter", name="sensor_velocimeter", site="site_head"
    )
    ET.SubElement(
        sensor, "accelerometer", name="sensor_accelerometer", site="site_head"
    )

    # Format XML with indentation and write to file
    raw_str = ET.tostring(mujoco, encoding="utf-8")
    reparsed = minidom.parseString(raw_str)
    pretty_xml = reparsed.toprettyxml(indent="    ")

    with open(filename, "w", encoding="utf-8") as f:
        f.write(pretty_xml)


if __name__ == "__main__":
    create_snake_xml("planar_snake_cars_servo.xml", num_segments=11)