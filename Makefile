.PHONY: build run all
.DEFAULT_GOAL := help

help:
	@echo "Makefile help:"
	@echo "* build		to build and create executable"
	@echo "* run		to run the executable"

build:
	cmake --build build

run:
	./build/raytracer > image.ppm

all:
	cmake --build build && ./build/raytracer > image.ppm

