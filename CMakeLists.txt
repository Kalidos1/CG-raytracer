cmake_minimum_required(VERSION 3.26)
project(raytracer)

set(CMAKE_CXX_STANDARD 20)

add_executable(raytracer main.cpp
        vec3.h
        color.h
        hittable.h
        material.h
        ray.h
        sphere.h
        triangle.h
        floor.h
        mat3.h)