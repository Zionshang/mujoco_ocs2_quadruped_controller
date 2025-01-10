import xml.etree.ElementTree as xml_et
import numpy as np
import math

INPUT_SCENE_PATH = "./scene_input.xml"
OUTPUT_SCENE_PATH = "./scene_output.xml"


# zyx euler angle to quaternion
def euler_to_quat(roll, pitch, yaw):
    cx = np.cos(roll / 2)
    sx = np.sin(roll / 2)
    cy = np.cos(pitch / 2)
    sy = np.sin(pitch / 2)
    cz = np.cos(yaw / 2)
    sz = np.sin(yaw / 2)

    return np.array(
        [
            cx * cy * cz + sx * sy * sz,
            sx * cy * cz - cx * sy * sz,
            cx * sy * cz + sx * cy * sz,
            cx * cy * sz - sx * sy * cz,
        ],
        dtype=np.float64,
    )


# zyx euler angle to rotation matrix
def euler_to_rot(roll, pitch, yaw):
    rot_x = np.array(
        [
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)],
        ],
        dtype=np.float64,
    )

    rot_y = np.array(
        [
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)],
        ],
        dtype=np.float64,
    )
    rot_z = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1],
        ],
        dtype=np.float64,
    )
    return rot_z @ rot_y @ rot_x


# 2d rotate
def rot2d(x, y, yaw):
    nx = x * np.cos(yaw) - y * np.sin(yaw)
    ny = x * np.sin(yaw) + y * np.cos(yaw)
    return nx, ny


# 3d rotate
def rot3d(pos, euler):
    R = euler_to_rot(euler[0], euler[1], euler[2])
    return R @ pos


def list_to_str(vec):
    return " ".join(str(s) for s in vec)


class TerrainGenerator:

    def __init__(self) -> None:
        self.scene = xml_et.parse(INPUT_SCENE_PATH)
        self.root = self.scene.getroot()
        self.worldbody = self.root.find("worldbody")
        self.asset = self.root.find("asset")

    # Add Box to scene
    def AddBox(
        self, position=[1.0, 0.0, 0.0], euler=[0.0, 0.0, 0.0], size=[0.1, 0.1, 0.1]
    ):
        geo = xml_et.SubElement(self.worldbody, "geom")
        geo.attrib["pos"] = list_to_str(position)
        geo.attrib["type"] = "box"
        geo.attrib["size"] = list_to_str(
            0.5 * np.array(size)
        )  # half size of box for mujoco
        quat = euler_to_quat(euler[0], euler[1], euler[2])
        geo.attrib["quat"] = list_to_str(quat)

    def AddGeometry(
        self,
        position=[1.0, 0.0, 0.0],
        euler=[0.0, 0.0, 0.0],
        size=[0.1, 0.1],
        geo_type="box",
    ):

        # geo_type supports "plane", "sphere", "capsule", "ellipsoid", "cylinder", "box"
        geo = xml_et.SubElement(self.worldbody, "geom")
        geo.attrib["pos"] = list_to_str(position)
        geo.attrib["type"] = geo_type
        geo.attrib["size"] = list_to_str(
            0.5 * np.array(size)
        )  # half size of box for mujoco
        quat = euler_to_quat(euler[0], euler[1], euler[2])
        geo.attrib["quat"] = list_to_str(quat)

    def AddStairs(
        self,
        init_pos=[1.0, 0.0, 0.0],
        yaw=0.0,
        width=0.2,
        height=0.15,
        length=1.5,
        stair_nums=10,
    ):

        local_pos = [0.0, 0.0, -0.5 * height]
        for i in range(stair_nums):
            local_pos[0] += width
            local_pos[2] += height
            x, y = rot2d(local_pos[0], local_pos[1], yaw)
            self.AddBox(
                [x + init_pos[0], y + init_pos[1], local_pos[2]],
                [0.0, 0.0, yaw],
                [width, length, height],
            )

    def AddSuspendStairs(
        self,
        init_pos=[1.0, 0.0, 0.0],
        yaw=1.0,
        width=0.2,
        height=0.15,
        length=1.5,
        gap=0.1,
        stair_nums=10,
    ):

        local_pos = [0.0, 0.0, -0.5 * height]
        for i in range(stair_nums):
            local_pos[0] += width
            local_pos[2] += height
            x, y = rot2d(local_pos[0], local_pos[1], yaw)
            self.AddBox(
                [x + init_pos[0], y + init_pos[1], local_pos[2]],
                [0.0, 0.0, yaw],
                [width, length, abs(height - gap)],
            )

    def AddRoughGround(
        self,
        init_pos=[1.0, 0.0, 0.0],
        euler=[0.0, -0.0, 0.0],
        nums=[10, 10],
        box_size=[0.5, 0.5, 0.5],
        box_euler=[0.0, 0.0, 0.0],
        separation=[0.2, 0.2],
        box_size_rand=[0.05, 0.05, 0.05],
        box_euler_rand=[0.2, 0.2, 0.2],
        separation_rand=[0.05, 0.05],
    ):

        local_pos = [0.0, 0.0, -0.5 * box_size[2]]
        new_separation = np.array(separation) + np.array(
            separation_rand
        ) * np.random.uniform(-1.0, 1.0, 2)
        for i in range(nums[0]):
            local_pos[0] += new_separation[0]
            local_pos[1] = 0.0
            for j in range(nums[1]):
                new_box_size = np.array(box_size) + np.array(
                    box_size_rand
                ) * np.random.uniform(-1.0, 1.0, 3)
                new_box_euler = np.array(box_euler) + np.array(
                    box_euler_rand
                ) * np.random.uniform(-1.0, 1.0, 3)
                new_separation = np.array(separation) + np.array(
                    separation_rand
                ) * np.random.uniform(-1.0, 1.0, 2)

                local_pos[1] += new_separation[1]
                pos = rot3d(local_pos, euler) + np.array(init_pos)
                self.AddBox(pos, new_box_euler, new_box_size)

    def Save(self):
        self.scene.write(OUTPUT_SCENE_PATH)


if __name__ == "__main__":
    tg = TerrainGenerator()

    # # Box obstacle
    # tg.AddBox(position=[1.5, 0.0, 0.1], euler=[0, 0, 0.0], size=[1, 1.5, 0.2])

    # # Geometry obstacle
    # # geo_type supports "plane", "sphere", "capsule", "ellipsoid", "cylinder", "box"
    # tg.AddGeometry(
    #     position=[1.5, 0.0, 0.25],
    #     euler=[0, 0, 0.0],
    #     size=[1.0, 0.5, 0.5],
    #     geo_type="cylinder",
    # )

    # Slope
    # length = 3
    # width = 1.5
    # thickness = 0.1
    # degree = 10 / 180 * math.pi
    # bottom_x = 1.0
    # bottom_y = 0.0

    # tg.AddBox(
    #     position=[
    #         bottom_x + length / 2 * math.cos(degree) + thickness / 2 * math.sin(degree),
    #         bottom_y,
    #         length / 2 * math.sin(degree) - thickness / 2 * math.cos(degree),
    #     ],
    #     euler=[0.0, -degree, 0.0],
    #     size=[length, width, thickness],
    # )

    length = 10
    width = 1.5
    thickness = 0.1
    initial_degree = 5 / 180 * math.pi
    bottom_x = 1.0
    bottom_y = 0.0

    # 循环次数
    num_iterations = 9

    for i in range(num_iterations):
        degree = initial_degree + (5 / 180 * math.pi) * i  # 每次增加 5 度
        tg.AddBox(
            position=[
                bottom_x
                + length / 2 * math.cos(degree)
                + thickness / 2 * math.sin(degree),
                bottom_y - 1.5 * width * i,  # 每次增加 1.5 倍的 width
                length / 2 * math.sin(degree) - thickness / 2 * math.cos(degree),
            ],
            euler=[0.0, -degree, 0.0],
            size=[length, width, thickness],
        )
        print(degree / math.pi * 180)

    # # Stairs
    # tg.AddStairs(init_pos=[1.0, 4.0, 0.0], yaw=0.0)

    # # Suspend stairs
    # tg.AddSuspendStairs(init_pos=[1.0, 6.0, 0.0], yaw=0.0)

    # # Rough ground
    # tg.AddRoughGround(init_pos=[-2.5, 5.0, 0.0], euler=[0, 0, 0.0], nums=[10, 8])

    tg.Save()
