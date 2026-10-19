# SPDX-License-Identifier: Apache-2.0
"""Portable workbench profiles and rigid tool assembly, independent of ROS.

Robot names, axes and geometry are retained exactly as imported.  The generated
URDF has absolute asset paths, a fixed instrument mount and an explicit TCP
frame.  Importing a model never grants it the LBR controller's seven-axis
interface: :attr:`RobotMetadata.control_compatible` reports that separately.
"""

from copy import deepcopy
from dataclasses import asdict, dataclass, field, fields
import json
import math
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET


TCP_LINK = "workbench_tcp"
TOOL_PREFIX = "workbench_tool_"
BUILTIN_TOOLS = ("none", "probe", "gripper")
_INITIAL_Q = [0.0, 0.35, 0.0, -0.9, 0.0, 0.65, 0.0]


@dataclass
class ProjectConfig:
    robot_urdf: str = ""
    flange_link: str = ""
    initial_positions: list | None = None
    tool: str = "none"
    mesh_scale: list = field(default_factory=lambda: [1.0, 1.0, 1.0])
    mount_xyz: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    mount_rpy: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    tcp_xyz: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    tcp_rpy: list = field(default_factory=lambda: [0.0, 0.0, 0.0])


@dataclass
class RobotMetadata:
    robot_name: str
    source_path: str
    root_link: str
    link_names: list
    joint_names: list
    joint_types: list
    movable_joint_indices: list
    lower_limits: list
    upper_limits: list
    effort_limits: list
    velocity_limits: list
    default_q: list
    flange_link: str
    control_compatible: bool
    control_reason: str

    @property
    def lower(self):
        return self.lower_limits

    @property
    def upper(self):
        return self.upper_limits

    @property
    def initial_positions(self):
        return self.default_q


def _vector(value, length, label):
    try:
        result = [float(item) for item in value]
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{label} must contain {length} finite numbers") from error
    if len(result) != length or not all(math.isfinite(item) for item in result):
        raise ValueError(f"{label} must contain {length} finite numbers")
    return result


def validate_project(config):
    """Return a normalized copy, without reading the robot or changing input."""
    if not isinstance(config, ProjectConfig):
        raise ValueError("Project must be a ProjectConfig")
    result = deepcopy(config)
    for name in ("robot_urdf", "flange_link", "tool"):
        if not isinstance(getattr(result, name), str):
            raise ValueError(f"{name} must be a string")
        setattr(result, name, getattr(result, name).strip())
    result.tool = result.tool or "none"
    for name in ("mesh_scale", "mount_xyz", "mount_rpy", "tcp_xyz", "tcp_rpy"):
        setattr(result, name, _vector(getattr(result, name), 3, name))
    if any(value <= 0.0 for value in result.mesh_scale):
        raise ValueError("Mesh scale values must be greater than zero")
    if result.initial_positions is not None:
        try:
            count = len(result.initial_positions)
        except TypeError as error:
            raise ValueError("Initial positions must be a list of finite numbers") from error
        result.initial_positions = _vector(result.initial_positions, count, "Initial positions")
    return result


def load_project(path):
    """Load JSON; relative model and instrument paths use the profile directory."""
    source = Path(path).expanduser().resolve()
    try:
        document = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise ValueError(f"Cannot read project profile {source}: {error}") from error
    if not isinstance(document, dict):
        raise ValueError("Project profile must contain a JSON object")
    allowed = {item.name for item in fields(ProjectConfig)}
    unknown = set(document) - allowed
    if unknown:
        raise ValueError("Unknown project fields: " + ", ".join(sorted(unknown)))
    config = validate_project(ProjectConfig(**document))
    for name in ("robot_urdf", "tool"):
        value = getattr(config, name)
        if value and not (name == "tool" and value in BUILTIN_TOOLS) and not value.startswith(("package://", "file://")):
            setattr(config, name, str((source.parent / Path(value).expanduser()).resolve()))
    return config


def save_project(config, path):
    """Atomically save a JSON profile with resolved model asset locations."""
    config = validate_project(config)
    for name in ("robot_urdf", "tool"):
        value = getattr(config, name)
        if value and not (name == "tool" and value in BUILTIN_TOOLS) and not value.startswith(("package://", "file://")):
            setattr(config, name, str(Path(value).expanduser().resolve()))
    target = Path(path).expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary = tempfile.mkstemp(prefix=f".{target.name}.", dir=target.parent)
    try:
        with os.fdopen(handle, "w", encoding="utf-8") as stream:
            json.dump(asdict(config), stream, indent=2, allow_nan=False)
            stream.write("\n")
        os.replace(temporary, target)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return str(target)


def active_project():
    """Use ``LBR_PYBULLET_PROJECT`` when set, otherwise the bundled iiwa."""
    path = os.environ.get("LBR_PYBULLET_PROJECT", "").strip()
    return load_project(path) if path else ProjectConfig()


def _resolve_asset(filename, directory):
    if not filename:
        raise ValueError("An asset is missing its filename")
    if filename.startswith("package://"):
        package, separator, relative = filename[len("package://"):].partition("/")
        if not separator or not package or not relative:
            raise ValueError(f"Invalid package asset path: {filename}")
        candidates = []
        try:
            from ament_index_python.packages import get_package_share_directory
            candidates.append(Path(get_package_share_directory(package)) / relative)
        except (ImportError, LookupError):
            pass
        for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
            if prefix:
                candidates.append(Path(prefix) / "share" / package / relative)
        for prefix in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
            if prefix:
                candidates.extend((Path(prefix) / package / relative, Path(prefix) / relative))
        resolved = next((candidate.resolve() for candidate in candidates if candidate.is_file()), None)
        if resolved is None:
            raise ValueError(f"Cannot resolve package asset: {filename}. Source the package workspace first")
        return resolved
    if filename.startswith("file://"):
        filename = filename[len("file://"):]
    if "://" in filename:
        raise ValueError(f"Only local files and package:// assets are supported: {filename}")
    target = Path(filename).expanduser()
    target = target if target.is_absolute() else directory / target
    target = target.resolve()
    if not target.is_file():
        raise ValueError(f"Asset does not exist: {target}")
    return target


def _robot_source(config):
    if config.robot_urdf:
        source = _resolve_asset(config.robot_urdf, Path.cwd())
    else:
        try:
            import pybullet_data
        except ImportError as error:
            raise ValueError("The bundled robot requires the pybullet package") from error
        source = Path(pybullet_data.getDataPath()) / "kuka_iiwa" / "model.urdf"
    if source.suffix.lower() != ".urdf":
        raise ValueError("Robot model must be a URDF file; export Xacro to URDF first")
    return source


def _read_urdf(source):
    try:
        text = source.read_text(encoding="utf-8")
        parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
        root = ET.fromstring(text, parser=parser)
    except (OSError, ET.ParseError) as error:
        raise ValueError(f"Cannot read URDF {source}: {error}") from error
    if root.tag != "robot":
        raise ValueError("URDF must have a robot root element")
    if any(isinstance(element.tag, str) and ("xacro" in element.tag or any("${" in value for value in element.attrib.values())) for element in root.iter()):
        raise ValueError("Unexpanded Xacro is not supported; export the model to URDF first")
    # ros2_control metadata belongs to the running controller, not imported XML.
    for element in list(root):
        if element.tag in ("ros2_control", "transmission", "gazebo"):
            root.remove(element)
    for mesh in root.findall(".//mesh"):
        mesh.set("filename", str(_resolve_asset(mesh.get("filename"), source.parent)))
        if mesh.get("scale"):
            scale = _vector(mesh.get("scale").split(), 3, "URDF mesh scale")
            if any(value <= 0 for value in scale):
                raise ValueError("URDF mesh scale values must be greater than zero")
    for texture in root.findall(".//texture"):
        texture.set("filename", str(_resolve_asset(texture.get("filename"), source.parent)))
    # Retain the source license, including comments before the robot element.
    preamble = text.split("<robot", 1)[0]
    if "<?xml" in preamble:
        start = preamble.index("<?xml")
        end = preamble.find("?>", start)
        if end >= 0:
            preamble = preamble[:start] + preamble[end + 2:]
    return root, preamble


def _tree(root):
    links = [link.get("name", "") for link in root.findall("link")]
    if not links or any(not name for name in links) or len(set(links)) != len(links):
        raise ValueError("URDF requires unique, nonempty link names")
    joints = root.findall("joint")
    names = [joint.get("name", "") for joint in joints]
    if any(not name for name in names) or len(set(names)) != len(names):
        raise ValueError("URDF requires unique, nonempty joint names")
    children = {}
    for joint in joints:
        parent, child = joint.find("parent"), joint.find("child")
        if parent is None or child is None:
            raise ValueError(f"Joint {joint.get('name')} requires parent and child links")
        parent, child = parent.get("link"), child.get("link")
        if parent not in links or child not in links:
            raise ValueError(f"Joint {joint.get('name')} references an unknown link")
        if child in children:
            raise ValueError(f"Link {child} has more than one parent joint")
        children[child] = (parent, joint)
    roots = set(links) - children.keys()
    if len(roots) != 1:
        raise ValueError("URDF must form one connected tree with exactly one root link")
    root_link = next(iter(roots))
    for name in links:
        seen = set()
        while name != root_link:
            if name in seen or name not in children:
                raise ValueError("URDF must form one connected tree without cycles")
            seen.add(name)
            name = children[name][0]
    return links, joints, root_link, children


def _number(element, name, default, label):
    try:
        value = float(element.get(name, default)) if element is not None else default
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{label} must be a finite number") from error
    if not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number")
    return value


def _inspect(config, root, source):
    links, joints, root_link, parents = _tree(root)
    actuated = [(index, joint) for index, joint in enumerate(joints) if joint.get("type") != "fixed"]
    if config.flange_link:
        flange = config.flange_link
    elif actuated:
        # URDF files need not list joints in kinematic order.  Prefer the leaf
        # after the longest actuated chain, independently of XML order.
        def depth(link):
            count = 0
            while link in parents:
                link, ancestor = parents[link]
                count += ancestor.get("type") != "fixed"
            return count
        flange = max((joint.find("child").get("link") for _, joint in actuated), key=depth)
        # Include fixed adapter links after the final axis when unambiguous.
        while True:
            descendants = [joint for joint in joints if joint.find("parent").get("link") == flange]
            if len(descendants) != 1 or descendants[0].get("type") != "fixed":
                break
            flange = descendants[0].find("child").get("link")
    else:
        flange = root_link
    if flange not in links:
        raise ValueError(f"Flange link does not exist in the robot: {flange}")
    lower, upper, effort, velocity, types, joint_names = [], [], [], [], [], []
    for _, joint in actuated:
        name, kind = joint.get("name"), joint.get("type")
        if kind not in ("revolute", "continuous", "prismatic", "planar", "floating"):
            raise ValueError(f"Joint {name} has an unsupported type: {kind}")
        limit = joint.find("limit")
        if kind in ("revolute", "prismatic"):
            if limit is None or limit.get("lower") is None or limit.get("upper") is None:
                raise ValueError(f"Joint {name} requires lower and upper limits")
            low = _number(limit, "lower", 0.0, f"{name} lower limit")
            high = _number(limit, "upper", 0.0, f"{name} upper limit")
            if low >= high:
                raise ValueError(f"Joint {name} lower limit must be smaller than upper limit")
        else:
            low, high = -math.pi, math.pi
        force = _number(limit, "effort", 100.0, f"{name} effort limit")
        speed = _number(limit, "velocity", 1.0, f"{name} velocity limit")
        if force <= 0 or speed <= 0:
            raise ValueError(f"Joint {name} effort and velocity limits must be greater than zero")
        lower.append(low)
        upper.append(high)
        effort.append(force)
        velocity.append(speed)
        types.append(kind)
        joint_names.append(name)
    if config.initial_positions is None:
        defaults = _INITIAL_Q if not config.robot_urdf and len(actuated) == 7 else [0.0] * len(actuated)
        default_q = [max(lo, min(hi, value)) for lo, hi, value in zip(lower, upper, defaults)]
    else:
        default_q = _vector(config.initial_positions, len(actuated), "Initial positions")
        for name, value, lo, hi in zip(joint_names, default_q, lower, upper):
            if not lo <= value <= hi:
                raise ValueError(f"Initial position for {name} must be within [{lo}, {hi}]")
    ancestors = set()
    cursor = flange
    while cursor in parents:
        cursor, joint = parents[cursor]
        ancestors.add(joint.get("name"))
    if len(actuated) != 7:
        reason = f"LBR control requires seven movable joints; this model has {len(actuated)}"
    elif any(kind != "revolute" for kind in types):
        reason = "LBR control requires seven bounded revolute joints"
    elif any(joint.find("mimic") is not None for _, joint in actuated):
        reason = "LBR control does not support mimic joints"
    elif not set(joint_names).issubset(ancestors):
        reason = "LBR control requires the flange to follow all seven joints in one serial chain"
    elif any(name in links and name != root_link for name in ("world", "lbr_link_0")):
        reason = "Imported model has an ambiguous reserved base frame (world or lbr_link_0); use unambiguous native base frame names for LBR control"
    else:
        reason = "Compatible with the seven-axis LBR position controller"
    return RobotMetadata(
        robot_name=root.get("name", source.stem), source_path=str(source), root_link=root_link,
        link_names=links, joint_names=joint_names, joint_types=types,
        movable_joint_indices=[index for index, _ in actuated], lower_limits=lower,
        upper_limits=upper, effort_limits=effort, velocity_limits=velocity,
        default_q=default_q, flange_link=flange,
        control_compatible=reason.startswith("Compatible"), control_reason=reason)


def inspect_robot(config):
    """Validate local assets and report native names, bounds and control support.

    Joint indices refer to document order; Bullet integrations should map by
    joint name because a loader may reorder a branched URDF tree.
    """
    config = validate_project(config)
    source = _robot_source(config)
    root, _ = _read_urdf(source)
    return _inspect(config, root, source)


def _origin(parent, xyz, rpy):
    ET.SubElement(parent, "origin", {"xyz": " ".join(map(str, xyz)), "rpy": " ".join(map(str, rpy))})


def _fixed(root, name, parent, child, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    joint = ET.SubElement(root, "joint", {"name": name, "type": "fixed"})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    _origin(joint, xyz, rpy)


def _inertial(link, mass=0.05):
    inertial = ET.SubElement(link, "inertial")
    _origin(inertial, (0, 0, 0), (0, 0, 0))
    ET.SubElement(inertial, "mass", {"value": str(mass)})
    ET.SubElement(inertial, "inertia", {"ixx": "0.0001", "iyy": "0.0001", "izz": "0.0001", "ixy": "0", "ixz": "0", "iyz": "0"})


def _shape(link, shape, attributes, xyz, color=(0.65, 0.68, 0.72, 1.0)):
    for kind in ("visual", "collision"):
        component = ET.SubElement(link, kind)
        _origin(component, xyz, (0, 0, 0))
        geometry = ET.SubElement(component, "geometry")
        ET.SubElement(geometry, shape, {name: str(value) for name, value in attributes.items()})
        if kind == "visual":
            material = ET.SubElement(component, "material", {"name": "workbench_steel"})
            ET.SubElement(material, "color", {"rgba": " ".join(map(str, color))})


def _builtin_tool(kind):
    root = ET.Element("robot", {"name": kind})
    link = ET.SubElement(root, "link", {"name": "body"})
    _inertial(link)
    if kind == "probe":
        _shape(link, "cylinder", {"radius": "0.012", "length": "0.035"}, (0, 0, 0.0175))
        _shape(link, "cylinder", {"radius": "0.003", "length": "0.14"}, (0, 0, 0.105))
        _shape(link, "sphere", {"radius": "0.004"}, (0, 0, 0.175))
    elif kind == "gripper":
        # A fixed, open demonstration instrument; there is no fake actuator.
        _shape(link, "box", {"size": "0.075 0.045 0.04"}, (0, 0, 0.02), (0.20, 0.25, 0.31, 1))
        for x in (-0.028, 0.028):
            _shape(link, "box", {"size": "0.012 0.025 0.08"}, (x, 0, 0.08))
    return root


def _tool_root(config):
    if config.tool in ("probe", "gripper"):
        root = _builtin_tool(config.tool)
    else:
        source = _resolve_asset(config.tool, Path.cwd())
        suffix = source.suffix.lower()
        if suffix == ".urdf":
            root, _ = _read_urdf(source)
        elif suffix in (".stl", ".obj"):
            root = ET.Element("robot", {"name": "mesh_instrument"})
            link = ET.SubElement(root, "link", {"name": "body"})
            _inertial(link)
            _shape(link, "mesh", {"filename": str(source), "scale": " ".join(map(str, config.mesh_scale))}, (0, 0, 0))
        else:
            raise ValueError("Instrument must be a rigid URDF, STL or OBJ file")
    links, joints, root_link, _ = _tree(root)
    if any(joint.get("type") != "fixed" for joint in joints):
        raise ValueError("Instrument URDF must be rigid: every instrument joint must be fixed")
    # Prefix every identifier to prevent tool names from shadowing robot names.
    for link in root.findall("link"):
        link.set("name", TOOL_PREFIX + link.get("name"))
    for joint in joints:
        joint.set("name", TOOL_PREFIX + joint.get("name"))
        for tag in ("parent", "child"):
            element = joint.find(tag)
            element.set("link", TOOL_PREFIX + element.get("link"))
    for material in root.findall(".//material"):
        if material.get("name"):
            material.set("name", TOOL_PREFIX + material.get("name"))
    return root, TOOL_PREFIX + root_link


def build_model(config, output_dir):
    """Build an absolute-path URDF with a rigid instrument and explicit TCP.

    Mount coordinates are relative to the selected robot flange.  TCP
    coordinates are relative to the instrument root, or the flange for an empty
    mount.  All positions are metres and all RPY angles are radians.
    """
    config = validate_project(config)
    source = _robot_source(config)
    root, preamble = _read_urdf(source)
    metadata = _inspect(config, root, source)
    occupied = {element.get("name") for element in root.iter() if element.get("name")}
    if TCP_LINK in occupied or "workbench_tcp_joint" in occupied or any(name.startswith(TOOL_PREFIX) for name in occupied):
        raise ValueError(f"Robot names must not use the reserved {TCP_LINK} or {TOOL_PREFIX} identifiers")
    tcp_parent = metadata.flange_link
    if config.tool != "none":
        tool, tcp_parent = _tool_root(config)
        for element in tool:
            root.append(element)
        _fixed(root, "workbench_tool_mount", metadata.flange_link, tcp_parent, config.mount_xyz, config.mount_rpy)
    tcp = ET.SubElement(root, "link", {"name": TCP_LINK})
    # A massless, collision-free coordinate frame must not change tool physics.
    inertial = ET.SubElement(tcp, "inertial")
    ET.SubElement(inertial, "mass", {"value": "0"})
    ET.SubElement(inertial, "inertia", {"ixx": "0", "iyy": "0", "izz": "0", "ixy": "0", "ixz": "0", "iyz": "0"})
    _fixed(root, "workbench_tcp_joint", tcp_parent, TCP_LINK, config.tcp_xyz, config.tcp_rpy)
    destination = Path(output_dir).expanduser().resolve()
    destination.mkdir(parents=True, exist_ok=True)
    target = destination / "workbench_robot.urdf"
    text = preamble + ET.tostring(root, encoding="unicode")
    handle, temporary = tempfile.mkstemp(prefix=".workbench_robot.", suffix=".urdf", dir=destination)
    try:
        with os.fdopen(handle, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.write("\n")
        os.replace(temporary, target)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return str(target)
