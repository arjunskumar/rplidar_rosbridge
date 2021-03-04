'''
Copyright (c) 2021, Arjun S Kumar
'''
#load("//engine/build:isaac.bzl", "isaac_app", "isaac_cc_module")

load("//bzl:module.bzl", "isaac_app", "isaac_cc_module" )



isaac_cc_module(
    name = "rplidar_ros_bridge_components",
    srcs = [
            "RplidarRosBridge.cpp",
    ],
    hdrs =
        ["RplidarRosBridge.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party:ros",
    ],
)


isaac_app(
    name = "rplidar_rosbridge",
    modules = [
        "navigation",
        "perception",
        "planner",
        "viewers",
        "flatsim",
        "//apps/samples/rplidar_rosbridge:rplidar_ros_bridge_components",
    ],

)