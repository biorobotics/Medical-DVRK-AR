import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np
import copy
import os


def demo_crop_geometry(file):
    print("Demo for manual geometry cropping")
    print(
        "1) Press 'Y' twice to align geometry with negative direction of y-axis"
    )
    print("2) Press 'K' to lock screen and to switch to selection mode")
    print("3) Drag for rectangle selection,")
    print("   or use ctrl + left click for polygon selection")
    print("4) Press 'C' to get a selected geometry and to save it")
    print("5) Press 'F' to switch to freeview mode")
    # pcd = o3d.io.read_point_cloud(file)
    o3d.visualization.draw_geometries_with_editing([file])

if __name__ == "__main__":
  
    #===========
    # crop geometry
    #===========
    root_path = "/home/alex/Documents/bagfile/pcdfile/"
    file = o3d.io.read_point_cloud(os.path.join(root_path, "test20.pcd"))
    demo_crop_geometry(file)

