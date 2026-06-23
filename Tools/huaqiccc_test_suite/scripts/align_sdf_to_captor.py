#!/usr/bin/env python3
"""
Align Gazebo SDF motor joint positions with PX4 CA_ROTOR coordinates.

PX4 FRD:  X=Front, Y=Right, Z=Down
SDF/Gazebo: X=Front, Y=Left,  Z=Up

CA_ROTOR coordinates are relative to the vehicle CoM.
SDF motor joint positions are relative to parent link frames.
This script iteratively adjusts motor joint poses so that motor positions
in the SDF world frame equal: model_CoM + R * CA_ROTOR,
where R = diag(1, -1, -1).
"""
import xml.etree.ElementTree as ET
import numpy as np
import shutil
import sys
from pathlib import Path

SDF_PATH = Path('/home/a/Projects/PX4/SEU_MD_PX4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/huaqiccc/huaqiccc.sdf')
BACKUP_PATH = SDF_PATH.with_suffix('.sdf.backup')

# PX4 CA_ROTOR coordinates (motor 0=rb, 1=rf, 2=lb, 3=lf)
PX4_CA_ROTOR = {
    'rb_motor_joint': np.array([-0.1550,  0.2150, 0.0]),
    'rf_motor_joint': np.array([ 0.2050,  0.1650, 0.0]),
    'lb_motor_joint': np.array([-0.1550, -0.2150, 0.0]),
    'lf_motor_joint': np.array([ 0.2050, -0.1650, 0.0]),
}

# FRD -> SDF rotation
def px4_to_sdf(v):
    return np.array([v[0], -v[1], -v[2]])


def pose_to_vec(pose_str):
    parts = [float(x) for x in pose_str.split()]
    return np.array(parts[:3])


def vec_to_pose(v):
    return f"{v[0]:.6f} {v[1]:.6f} {v[2]:.6f} 0 -0 0"


def parse_inertial(link):
    inertial = link.find('inertial')
    if inertial is None:
        return None
    mass = float(inertial.find('mass').text)
    pose = inertial.find('pose')
    com = pose_to_vec(pose.text) if pose is not None else np.zeros(3)
    return mass, com


def parse_sdf(sdf_path):
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model = root.find('model')

    links = {}
    joints = {}

    for link in model.findall('link'):
        name = link.get('name')
        pose = link.find('pose')
        pose_vec = pose_to_vec(pose.text) if pose is not None else np.zeros(3)
        links[name] = {'elem': link, 'pose': pose_vec, 'inertial': parse_inertial(link)}

    for joint in model.findall('joint'):
        name = joint.get('name')
        parent = joint.find('parent').text
        child = joint.find('child').text
        pose = joint.find('pose')
        pose_vec = pose_to_vec(pose.text) if pose is not None else np.zeros(3)
        joints[name] = {'elem': joint, 'parent': parent, 'child': child, 'pose': pose_vec}

    return tree, model, links, joints


def compute_poses(links, joints):
    global_poses = {}

    def recurse(link_name, parent_pose=np.zeros(3)):
        if link_name not in links:
            return
        global_poses[link_name] = parent_pose + links[link_name]['pose']
        for j in joints.values():
            if j['parent'] == link_name:
                child_name = j['child']
                child_pose = parent_pose + links[link_name]['pose'] + j['pose']
                recurse(child_name, child_pose)

    recurse('base_link')
    return global_poses


def compute_com(links, global_poses):
    total_mass = 0.0
    weighted = np.zeros(3)
    for name, data in links.items():
        if data['inertial'] is not None:
            mass, local_com = data['inertial']
            world_com = global_poses[name] + local_com
            total_mass += mass
            weighted += mass * world_com
    return weighted / total_mass if total_mass > 0 else np.zeros(3), total_mass


def set_joint_pose(joint_elem, new_pose):
    pose = joint_elem.find('pose')
    if pose is None:
        pose = ET.SubElement(joint_elem, 'pose')
    pose.text = vec_to_pose(new_pose)


def main():
    if not SDF_PATH.exists():
        print(f"ERROR: {SDF_PATH} not found")
        sys.exit(1)

    # Backup original
    if not BACKUP_PATH.exists():
        shutil.copy2(SDF_PATH, BACKUP_PATH)
        print(f"[BACKUP] saved to {BACKUP_PATH}")

    tree, model, links, joints = parse_sdf(SDF_PATH)

    print("=" * 60)
    print("SDF motor alignment to PX4 CA_ROTOR")
    print("=" * 60)

    for iteration in range(5):
        global_poses = compute_poses(links, joints)
        com, mass = compute_com(links, global_poses)
        print(f"\n[Iter {iteration}] Total mass={mass:.4f} kg, CoM={com}")

        max_err = 0.0
        for joint_name, target_px4 in PX4_CA_ROTOR.items():
            j = joints[joint_name]
            parent_global = global_poses[j['parent']]
            current_world = parent_global + j['pose']
            desired_world = com + px4_to_sdf(target_px4)
            error = desired_world - current_world
            max_err = max(max_err, np.linalg.norm(error))

            # Update joint pose
            new_pose = j['pose'] + error
            set_joint_pose(j['elem'], new_pose)
            j['pose'] = new_pose

            print(f"  {joint_name}: current=({current_world[0]:.4f},{current_world[1]:.4f},{current_world[2]:.4f}) "
                  f"desired=({desired_world[0]:.4f},{desired_world[1]:.4f},{desired_world[2]:.4f}) "
                  f"err={np.linalg.norm(error):.4f}")

        if max_err < 1e-5:
            print(f"\n[CONVERGED] at iteration {iteration}")
            break

    # Final report
    global_poses = compute_poses(links, joints)
    com, mass = compute_com(links, global_poses)
    print(f"\n[FINAL] Total mass={mass:.4f} kg, CoM={com}")
    print("Motor positions relative to CoM (SDF frame -> converted to PX4 FRD):")
    for joint_name in PX4_CA_ROTOR.keys():
        j = joints[joint_name]
        world = global_poses[j['parent']] + j['pose']
        rel_sdf = world - com
        rel_px4 = np.array([rel_sdf[0], -rel_sdf[1], -rel_sdf[2]])
        print(f"  {joint_name}: rel_px4=({rel_px4[0]:.4f},{rel_px4[1]:.4f},{rel_px4[2]:.4f}) "
              f"target_px4=({PX4_CA_ROTOR[joint_name][0]:.4f},{PX4_CA_ROTOR[joint_name][1]:.4f},{PX4_CA_ROTOR[joint_name][2]:.4f})")

    # Preserve XML formatting roughly
    tree.write(SDF_PATH, encoding='utf-8', xml_declaration=True)
    print(f"\n[WRITE] updated {SDF_PATH}")


if __name__ == '__main__':
    main()
