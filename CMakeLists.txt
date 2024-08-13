cmake_minimum_required(VERSION 3.26)
project(raytracer)

set(CMAKE_CXX_STANDARD 20)

add_executable(raytracer main.cpp
        vec3.h
        color.h
        hittable.h
        material.h
        ray.h
        triangle.h
        mat3.h
        light.h
        bounding_box.h
        obj_loader.h
        bvh_builder.h
        bvh_builder.h
        bvh_builder.h
        uniform_grid.h
        data_structure.h)